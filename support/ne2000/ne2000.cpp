/*
 * ne2000.cpp -- NE2000 network bridge for the ao486 core, integrated into
 * Main_MiSTer's poll loop.
 *
 * Ported from the standalone ao486_MiSTer/sw/ne2000d daemon. Same shared-window
 * ABI and byte conventions; the AF_PACKET layer (in-kernel BPF MAC filter,
 * promiscuous mode, SO_RCVBUFFORCE, GRO/LRO off) is adopted from the A2065
 * project. The standalone while(1)+sleep loop is replaced by ne2000_init()
 * (on core load) and ne2000_poll() (once per user_io_poll).
 *
 * Enable in MiSTer.ini:  ne2000=1  (optional ne2000_iface=eth0, ne2000_rxrate=0)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/filter.h>
#include <arpa/inet.h>

#include "ne2000.h"
#include "../../cfg.h"
#include "../../shmem.h"

// ---- shared window layout (must match rtl/soc/ne2000/ne2000_core.v) ----------
static const uint32_t SHM_BASE = 0x1FF00000;
static const uint32_t SHM_SIZE = 0x10000;

enum {
    OFF_FLAGS         = 0x1000,
    OFF_MAC           = 0x104C,
    OFF_STATUS        = 0x1052,
    OFF_HEARTBEAT     = 0x1088,
    OFF_SIGNATURE     = 0x108C,
    OFF_RX_QUEUE_HEAD = 0x2C04,
    OFF_RX_QUEUE_TAIL = 0x2C06,
    OFF_TX_REQ_SEQ    = 0x2C08,
    OFF_TX_DONE_SEQ   = 0x2C0A,
    OFF_RX_QUEUE_LEN  = 0x2C20,     // 16 x u16
    OFF_TX_SLOT_LEN   = 0x2C40,     // 8 x u16 (Fix B TX ring)
    OFF_TX_SLOT_DATA  = 0x3000,     // 8 x 1536
    OFF_RX_QUEUE_DATA = 0x9000,     // 16 x 1536
};

static const int      RX_SLOTS      = 16;
static const int      RX_SLOT_BYTES = 0x600;
static const int      TX_SLOTS      = 8;
static const int      TX_SLOT_BYTES = 0x600;
static const uint8_t  FLAG_RX_AVAIL = 0x04;
static const uint32_t SIGNATURE     = 0xCAFEBABEu;
static const int      MAX_FRAME     = 1536;

enum { ST_SAMPLED = 0x0100, ST_SIGNATURE = 0x0200, ST_HEARTBEAT = 0x0400,
       ST_HB_CHANGED = 0x0800, ST_COMM_OK = 0x1000 };

static const uint8_t VIRT_MAC_DEFAULT[6] = { 0x52, 0x54, 0x05, 0x04, 0x03, 0x02 };

// ---- state ------------------------------------------------------------------
static volatile uint8_t *win = NULL;
static int      sock_fd = -1;
static int      ifindex = 0;
static uint8_t  card_mac[6];
static uint32_t heartbeat = 0;
static uint16_t last_tx_seq = 0;
static double   rx_rate_bps = 0;      // 0 = unlimited
static double   rx_tokens = 0;
static struct timespec rx_last = { 0, 0 };
static const double RX_BURST = 32768;
static int      enabled = 0;          // -1 disabled, 0 uninit, 1 running
static char     iface_name[32] = "eth0";
static unsigned long rx_full = 0, tx_frames = 0, rx_frames = 0;

static inline uint16_t w_u16(int off) { return (uint16_t)(win[off] | (win[off + 1] << 8)); }
static inline void w_set_u16(int off, uint16_t v) { win[off] = (uint8_t)(v & 0xFF); win[off + 1] = (uint8_t)(v >> 8); }
static inline void w_set_u32(int off, uint32_t v) { w_set_u16(off, (uint16_t)(v & 0xFFFF)); w_set_u16(off + 2, (uint16_t)(v >> 16)); }

// ---- AF_PACKET (from A2065 ethernet.cpp) ------------------------------------
static int read_iface_mac(const char *iface, uint8_t *out) {
    memset(out, 0, 6);
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return 0;
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    int ok = 0;
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) { memcpy(out, ifr.ifr_hwaddr.sa_data, 6); ok = (out[3] | out[4] | out[5]) != 0; }
    close(fd);
    return ok;
}

static void iface_up(const char *iface) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return;
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0 && !(ifr.ifr_flags & IFF_UP)) {
        ifr.ifr_flags |= IFF_UP; ioctl(fd, SIOCSIFFLAGS, &ifr);
    }
    close(fd);
}

static int set_mac_filter(const uint8_t *mac) {
    if (sock_fd < 0) return 0;
    uint32_t macword = ((uint32_t)mac[0] << 24) | ((uint32_t)mac[1] << 16) | ((uint32_t)mac[2] << 8) | (uint32_t)mac[3];
    uint32_t machalf = ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];
    struct sock_filter code[] = {
        BPF_STMT(BPF_LD | BPF_B | BPF_ABS, (uint32_t)(SKF_AD_OFF + SKF_AD_PKTTYPE)),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, PACKET_OUTGOING, 8, 0),
        BPF_STMT(BPF_LD | BPF_W | BPF_ABS, 0),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, 0xFFFFFFFFu, 0, 2),
        BPF_STMT(BPF_LD | BPF_H | BPF_ABS, 4),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, 0xFFFFu, 3, 4),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, macword, 0, 3),
        BPF_STMT(BPF_LD | BPF_H | BPF_ABS, 4),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, machalf, 0, 1),
        BPF_STMT(BPF_RET | BPF_K, 0xFFFFFFFFu),
        BPF_STMT(BPF_RET | BPF_K, 0),
    };
    struct sock_fprog prog; prog.len = sizeof code / sizeof code[0]; prog.filter = code;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_ATTACH_FILTER, &prog, sizeof prog) < 0) { perror("[ne2000] SO_ATTACH_FILTER"); return 0; }
    return 1;
}

static int ethernet_open(const char *iface, const uint8_t *mac) {
    sock_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sock_fd < 0) { perror("[ne2000] socket"); return 0; }
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) { perror("[ne2000] SIOCGIFINDEX"); close(sock_fd); sock_fd = -1; return 0; }
    ifindex = ifr.ifr_ifindex;
    iface_up(iface);
    struct sockaddr_ll addr; memset(&addr, 0, sizeof addr);
    addr.sll_family = AF_PACKET; addr.sll_protocol = htons(ETH_P_ALL); addr.sll_ifindex = ifindex;
    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof addr) < 0) { perror("[ne2000] bind"); close(sock_fd); sock_fd = -1; return 0; }
    struct packet_mreq mreq; memset(&mreq, 0, sizeof mreq);
    mreq.mr_ifindex = ifindex; mreq.mr_type = PACKET_MR_PROMISC;
    if (setsockopt(sock_fd, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &mreq, sizeof mreq) < 0) perror("[ne2000] PACKET_ADD_MEMBERSHIP");
    int rcvbuf = 4 * 1024 * 1024;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUFFORCE, &rcvbuf, sizeof rcvbuf) < 0)
        setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);
    set_mac_filter(mac);
    char cmd[160];
    snprintf(cmd, sizeof cmd, "ethtool -K %s gro off lro off gso off tso off 2>/dev/null", iface);
    if (system(cmd) != 0) { /* MTU-sized frames still fit */ }
    return 1;
}

static int ethernet_recv_nb(uint8_t *buf, int maxlen) {
    ssize_t n = recv(sock_fd, buf, (size_t)maxlen, MSG_DONTWAIT);
    if (n < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; if (errno != EINTR) perror("[ne2000] recv"); return -1; }
    return (int)n;
}

// ---- window / MAC -----------------------------------------------------------
static void derive_mac(const char *iface) {
    memcpy(card_mac, VIRT_MAC_DEFAULT, 6);
    uint8_t hw[6];
    if (read_iface_mac(iface, hw)) { card_mac[4] = hw[4]; card_mac[5] = hw[5]; }
}

static void publish() {
    memset((void *)win, 0, SHM_SIZE);
    for (int i = 0; i < 6; i++) win[OFF_MAC + i] = card_mac[i];
    w_set_u32(OFF_SIGNATURE, SIGNATURE);
    w_set_u32(OFF_HEARTBEAT, 0);
    last_tx_seq = w_u16(OFF_TX_DONE_SEQ);
    printf("[ne2000] iface %s, card MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
           iface_name, card_mac[0], card_mac[1], card_mac[2], card_mac[3], card_mac[4], card_mac[5]);
}

// ---- TX ring drain (Fix B) --------------------------------------------------
static uint16_t tx_next_seq(uint16_t s) { return s == 0xFFFF ? 1 : (uint16_t)(s + 1); }

static int tx_send_one(uint16_t seq) {
    int slot = seq & (TX_SLOTS - 1);
    uint16_t length = w_u16(OFF_TX_SLOT_LEN + slot * 2);
    if (length == 0 || length > MAX_FRAME) return 1;
    uint8_t frame[MAX_FRAME];
    int base = OFF_TX_SLOT_DATA + slot * TX_SLOT_BYTES;
    for (int i = 0; i < length; i++) frame[i] = win[base + i];
    ssize_t sent = send(sock_fd, frame, length, 0);
    if (sent < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) return 0; perror("[ne2000] send"); return 1; }
    tx_frames++;
    return 1;
}

static void poll_tx() {
    uint16_t req = w_u16(OFF_TX_REQ_SEQ);
    if (req == 0) return;
    int guard = 0;
    while (last_tx_seq != req && guard++ < 64) {
        uint16_t s = tx_next_seq(last_tx_seq);
        if (!tx_send_one(s)) break;
        last_tx_seq = s;
    }
    w_set_u16(OFF_TX_DONE_SEQ, last_tx_seq);
}

// ---- RX -----------------------------------------------------------------
static int wanted(const uint8_t *f) {
    if (memcmp(f, card_mac, 6) == 0) return 1;
    static const uint8_t bcast[6] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
    return memcmp(f, bcast, 6) == 0;
}

static int enqueue_rx(const uint8_t *frame, int n) {
    uint16_t head = w_u16(OFF_RX_QUEUE_HEAD) & (RX_SLOTS - 1);
    uint16_t tail = w_u16(OFF_RX_QUEUE_TAIL) & (RX_SLOTS - 1);
    uint16_t nxt  = (uint16_t)((tail + 1) & (RX_SLOTS - 1));
    if (nxt == head) { rx_full++; return 0; }
    int len = n > RX_SLOT_BYTES ? RX_SLOT_BYTES : n;
    int base = OFF_RX_QUEUE_DATA + tail * RX_SLOT_BYTES;
    for (int i = 0; i < len; i++) win[base + i] = frame[i];
    w_set_u16(OFF_RX_QUEUE_LEN + tail * 2, (uint16_t)len);
    w_set_u16(OFF_RX_QUEUE_TAIL, nxt);
    win[OFF_FLAGS] = win[OFF_FLAGS] | FLAG_RX_AVAIL;
    rx_frames++;
    return 1;
}

static void poll_rx() {
    if (rx_rate_bps > 0) {
        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        if (rx_last.tv_sec || rx_last.tv_nsec) {
            double dt = (now.tv_sec - rx_last.tv_sec) + (now.tv_nsec - rx_last.tv_nsec) / 1e9;
            rx_tokens += dt * rx_rate_bps;
            if (rx_tokens > RX_BURST) rx_tokens = RX_BURST;
        }
        rx_last = now;
    }
    uint8_t frame[MAX_FRAME];
    for (int pass = 0; pass < 64; pass++) {
        if (rx_rate_bps > 0 && rx_tokens < MAX_FRAME) break;
        int n = ethernet_recv_nb(frame, sizeof frame);
        if (n <= 0) return;
        if (n < 14) continue;
        if (memcmp(frame + 6, card_mac, 6) == 0) continue;
        if (!wanted(frame)) continue;
        if (enqueue_rx(frame, n) && rx_rate_bps > 0) rx_tokens -= n;
    }
}

// ---- public API -------------------------------------------------------------
void ne2000_init() {
    if (enabled < 0) return;                       // disabled this session

    if (!cfg.ne2000) { enabled = -1; return; }     // not enabled in MiSTer.ini

    if (cfg.ne2000_iface[0]) { strncpy(iface_name, cfg.ne2000_iface, sizeof iface_name - 1); iface_name[sizeof iface_name - 1] = 0; }
    rx_rate_bps = cfg.ne2000_rxrate > 0 ? (double)cfg.ne2000_rxrate * 1.0e6 / 8.0 : 0;

    if (!win) {
        win = (volatile uint8_t *)shmem_map(SHM_BASE, SHM_SIZE);
        if (!win) { printf("[ne2000] shmem_map failed\n"); enabled = -1; return; }
    }
    derive_mac(iface_name);
    if (sock_fd < 0 && !ethernet_open(iface_name, card_mac)) { enabled = -1; return; }
    publish();                                      // re-publish (clear+MAC+sig) on every core load
    enabled = 1;
}

void ne2000_poll() {
    if (enabled != 1) return;
    heartbeat = (heartbeat + 1) & 0xFFFFFFFFu;
    w_set_u32(OFF_HEARTBEAT, heartbeat);
    poll_tx();
    poll_rx();
}
