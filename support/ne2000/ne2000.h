// NE2000 network bridge for the ao486 core -- Main_MiSTer integration.
//
// Bridges the guest's FPGA NE2000 (a shared DDR mailbox at 0x1FF00000) to the
// host's Ethernet via an AF_PACKET raw socket. Refactored from the standalone
// sw/ne2000d daemon (ao486_MiSTer/sw/ne2000d) into Main's poll loop: ne2000_init
// runs when the ao486 core loads, ne2000_poll runs once per user_io_poll.
//
// Enable in MiSTer.ini under [ao486]:  ne2000=1   (optional: ne2000_iface=eth0)

#ifndef __NE2000_H__
#define __NE2000_H__

// Called from user_io_init after x86_init() when the ao486 core is loaded.
// Idempotent: re-publishes the window (clear + MAC + signature) on a reload.
void ne2000_init();

// Called from user_io_poll (gated on is_x86()). One non-blocking pass:
// heartbeat, drain the TX ring, deliver RX frames. Does nothing until enabled.
void ne2000_poll();

#endif
