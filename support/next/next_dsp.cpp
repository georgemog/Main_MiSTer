// The NeXT-Color's DSP56001 on the ARM (see next_dsp.h).
//
// The DSP is Hatari's DSP56001 core (support/next/dsp56k, GPL v2 or later;
// with Previous's NeXT changes: the NeXT memory map, start modes and the
// bootstrap ROM, port C TXD).  This file is the link to the FPGA half of
// the host port, rtl/tc_dsp.sv in the NeXT-Color core, which is the other
// side of every rule below.
//
// The mailbox: DDR3 bytes 0x30400000..0x30403FFF, 64-bit little-endian
// words.
//   +0x0000  FPGA word  {0xD5F1, gen, D2H read index, H2D write index}
//   +0x0008  ARM word   {0xD5A1, gen, H2D read index, D2H write index}
//   +0x0010  debug      {instructions / 1024 : flags 4, messages 12, PC 16}
//   +0x1000  H2D ring   256 x {0:4, type:4, epoch:8, 0:24, data:24}
//   +0x2000  D2H ring   256 x {type:8, epoch:8, 0:24, data:24}
// Indices are 16-bit message counts (the ring slot is the low 8 bits).  gen
// changes at every core reset; the FPGA drops its messages until the ARM
// word names its gen, and the ARM resets the DSP when it sees a new one.
//
// To the ARM (the 68040's host-port accesses, in the order it made them):
//   T_TX     a word the host wrote to TX (TXL)
//   T_ICR    an ICR write (HF0/HF1, DMA mode, INIT)
//   T_CVR    a CVR write (HC: a host command)
//   T_RXACK  the host read a word from RX (RXL), a credit for one more
//   T_RESET  SCR2 DSP_RESET 1 -> 0: the DSP into reset
//   T_START  SCR2 DSP_RESET 0 -> 1: start in mode data[1:0]
//   T_MEM    SCR2 DSP_MEM_EN = data[0]: the external memory on / off
//   T_IRQB   a DMA block end with SCR2 DSP_BLK_END: the DSP's IRQB
// From the ARM:
//   R_RX      a word for the host's RX (at most DEPTH not yet acknowledged)
//   R_HRXACK  the DSP read HRX: one host word consumed (TXDE / TRDY)
//   R_FLAGS   HF2/HF3 (the DSP's HCR) as ISR bits 4:3
//   R_HCACK   a host command's handler returned
//   R_TXD     port C TXD low (data[0] = 1) / high, the host's TXD interrupt
//   R_HIRESET the RESET instruction reset the host port
// Every D2H message carries the epoch {DSP resets:4, TX INITs:2, RX
// INITs:2} of the last H2D message handled; the FPGA drops a message that
// answers something an INIT or a reset has since thrown away.
//
// The host-port rules the FPGA promises the 68040 and this side keeps:
//   - TRDY: a host command is taken only once the DSP has read every word
//     sent before it (the host sees TRDY as soon as its words are on the
//     link);
//   - a host command is taken in order: later messages wait until the DSP
//     has taken it (HC clears for the host at once), except a DSP reset;
//   - the host sees HF3 from a host command until its handler returns
//     (R_HCACK), so the Music Kit monitor never looks idle or aborted inside
//     a host message.

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

#include "next_dsp.h"
#include "../../shmem.h"
#include "dsp56k/dsp_core.h"
#include "dsp56k/dsp_cpu.h"

#define DL_BASE        0x30400000UL
#define DL_SIZE        0x4000UL
#define OFF_FPGA       0x0000
#define OFF_ARM        0x0008
#define OFF_DEBUG      0x0010
#define OFF_H2D        0x1000
#define OFF_D2H        0x2000
#define RING_N         256

#define FPGA_MAGIC     0xD5F1
#define ARM_MAGIC      0xD5A1

enum { T_TX = 1, T_ICR, T_CVR, T_RXACK, T_RESET, T_START, T_MEM, T_IRQB };
enum { R_RX = 1, R_HRXACK, R_FLAGS, R_HCACK, R_TXD, R_HIRESET };

#define DSPLINK_DEPTH  4                 // tc_dsp K: RX words waiting in the FPGA
#define DSP_HZ         25000000          // the DSP's clock; instr_cycle counts it
#define HC_WATCHDOG    (1u << 20)        // instructions before an unreturned handler is acked
#define DSP_RAM_WORDS  8192              // the NeXT's 8K words (24 KB); 32768 with the expansion

// the DSP's external RAM (DSP_EnableMemory)
static uint32_t dsp_ram[32768];
static uint32_t dsp_ram_words = DSP_RAM_WORDS;

//----------------------------------------------------------------------------
// the mailbox
//----------------------------------------------------------------------------

static volatile uint32_t *mb;            // mapped window (the thread)
static next_dsp_rd_t sim_rd;             // or the bench's callbacks
static next_dsp_wr_t sim_wr;

static uint64_t rd64(uint32_t off)
{
	if (sim_rd) return sim_rd(off);
	uint32_t lo = mb[off >> 2];
	uint32_t hi = mb[(off >> 2) + 1];
	return ((uint64_t)hi << 32) | lo;
}

static void wr64(uint32_t off, uint64_t v)
{
	if (sim_wr) { sim_wr(off, v); return; }
	mb[off >> 2] = (uint32_t)v;
	mb[(off >> 2) + 1] = (uint32_t)(v >> 32);
}

static void barrier(void)
{
	__sync_synchronize();
}

//----------------------------------------------------------------------------
// link state
//----------------------------------------------------------------------------

static int      linked;                  // following an FPGA generation
static uint16_t gen;
static uint16_t h2d_rd, d2h_wr;          // our indices
static uint16_t f_h2d_wr, f_d2h_rd;      // the FPGA's, from its word
static int      arm_dirty;               // the ARM word needs writing
static uint8_t  epoch;                   // of the last H2D message handled
static uint32_t msgs;                    // H2D messages handled (debug)
static uint64_t instrs;                  // DSP instructions run (debug)

// D2H messages not yet in the ring (the ring was full)
#define PENDQ_N 64
static uint64_t pendq[PENDQ_N];
static int pend_rd, pend_n;

// host words from T_TX not yet in the local host port's TX
#define TXQ_N 64
#define TX_EARLY_ACK 32                  // words acknowledged on arrival
static uint32_t txq[TXQ_N];
static int txq_rd, txq_n;
static int tx_outstanding;               // words the FPGA counts as with us (tc_dsp tx_out)

static int rx_out;                       // R_RX words not yet acknowledged
static uint8_t last_hf;                  // ISR HF3:HF2 last sent
static int last_txd;

// host command tracking
enum { HC_NONE, HC_PIPE, HC_LONG };
static int hc_state;
static uint32_t hc_sp;
static uint32_t hc_count;
static int hc_wait_taken;                // a T_CVR with HC applied, the DSP has not taken it
static int slice_break;
static uint32_t hc_hold_left;            // instructions the handler runs before the host's next access
#define HC_POST_HOLD   64                // a 56001 is well into a handler before the host's next
                                         // access (its HTX ack goes with the epoch before an INIT)
static int parked;                       // a host command waiting for the words before it
static uint8_t parked_cvr;
static uint64_t park_since;
static int hold_on;                      // an ICR / CVR waiting for the DSP to read HRX
static uint64_t hold_since;
#define ICR_HOLD       256               // instructions an ICR waits for unread host words                  // end the instruction slice (poll the ring now)

static void d2h_flush(void);

static void d2h(int type, uint32_t data)
{
	uint64_t m = ((uint64_t)(type & 0xFF) << 56) | ((uint64_t)epoch << 48) | (data & 0xFFFFFF);
	if (pend_n == 0 && (uint16_t)(d2h_wr - f_d2h_rd) < RING_N)
	{
		wr64(OFF_D2H + 8 * (d2h_wr & 0xFF), m);
		d2h_wr++;
		arm_dirty = 1;
		return;
	}
	if (pend_n < PENDQ_N)
	{
		pendq[(pend_rd + pend_n) % PENDQ_N] = m;
		pend_n++;
	}
	else
	{
		printf("[next-dsp] D2H overflow, message %d dropped\n", type);
	}
}

static void d2h_flush(void)
{
	while (pend_n && (uint16_t)(d2h_wr - f_d2h_rd) < RING_N)
	{
		wr64(OFF_D2H + 8 * (d2h_wr & 0xFF), pendq[pend_rd]);
		pend_rd = (pend_rd + 1) % PENDQ_N;
		pend_n--;
		d2h_wr++;
		arm_dirty = 1;
	}
}

static void arm_word(void)
{
	barrier();
	wr64(OFF_ARM, ((uint64_t)ARM_MAGIC << 48) | ((uint64_t)gen << 32) |
	              ((uint64_t)h2d_rd << 16) | d2h_wr);
	arm_dirty = 0;
}

//----------------------------------------------------------------------------
// the DSP
//----------------------------------------------------------------------------

static void hreq_none(int) { }

static void link_reset_counts(void)
{
	txq_rd = txq_n = 0;
	tx_outstanding = 0;
	rx_out = 0;
	last_hf = 0;
	last_txd = 0;
	hc_state = HC_NONE;
	hc_wait_taken = 0;
	hc_hold_left = 0;
	hold_on = 0;
	parked = 0;
}

static void dsp_full_reset(void)
{
	static int inited;
	if (!inited)
	{
		dsp_core_init(hreq_none);
		dsp56k_init_cpu();
		inited = 1;
	}
	dsp_core_reset();
	link_reset_counts();
}

static uint8_t hsr(void) { return (uint8_t)dsp_core.periph[DSP_SPACE_X][DSP_HOST_HSR]; }
static uint8_t isr(void) { return dsp_core.hostport[CPU_HOST_ISR]; }

// local TX empty: every word sent has been read by the DSP
static int tx_drained(void)
{
	return txq_n == 0 && (isr() & (1 << CPU_HOST_ISR_TXDE)) && !(hsr() & (1 << DSP_HOST_HSR_HRDF));
}

// host words into the local host port's TX as it empties
static void tx_feed(void)
{
	while (txq_n && (isr() & (1 << CPU_HOST_ISR_TXDE)))
	{
		uint32_t w = txq[txq_rd];
		txq_rd = (txq_rd + 1) % TXQ_N;
		txq_n--;
		dsp_core_write_host(CPU_HOST_TRXH, (uint8_t)(w >> 16));
		dsp_core_write_host(CPU_HOST_TRXM, (uint8_t)(w >> 8));
		dsp_core_write_host(CPU_HOST_TRXL, (uint8_t)w);
	}
}

// the ISR byte service() last saw: while it is unchanged and nothing is
// queued, parked or pending, service() would do nothing (link_busy)
static int isr_seen = -1;

// what the DSP did to its side of the port, after each instruction
static void service(void)
{
	tx_feed();

	if (parked && (tx_drained() || !dsp_core.running || instrs - park_since >= HC_WATCHDOG))
	{
		if (!tx_drained()) printf("[next-dsp] host command %02x applied with host words unread\n", parked_cvr);
		parked = 0;
		dsp_core_write_host(CPU_HOST_CVR, parked_cvr);
		hc_wait_taken = 1;
		slice_break = 1;
	}

	// RX: the word goes to the FPGA, then the local host takes it so the
	// DSP sees HTDE again
	while ((isr() & (1 << CPU_HOST_ISR_RXDF)) && rx_out < DSPLINK_DEPTH && pend_n < PENDQ_N / 2)
	{
		uint32_t w = ((uint32_t)dsp_core.hostport[CPU_HOST_RXH] << 16) |
		             ((uint32_t)dsp_core.hostport[CPU_HOST_RXM] << 8) |
		             dsp_core.hostport[CPU_HOST_RXL];
		d2h(R_RX, w);
		rx_out++;
		dsp_core_read_host(CPU_HOST_TRXL);
	}

	uint8_t hf = isr() & ((1 << CPU_HOST_ISR_HF2) | (1 << CPU_HOST_ISR_HF3));
	if (hf != last_hf)
	{
		last_hf = hf;
		d2h(R_FLAGS, hf);
	}

	switch (hc_state)
	{
	case HC_PIPE:
		hc_count++;
		// a long interrupt pushed PC/SR (Hatari pushes in the interrupt
		// pipeline, before the vector's JSR runs)
		if ((dsp_core.registers[DSP_REG_SP] & 0xF) > hc_sp) hc_state = HC_LONG;
		else if (dsp_core.interrupt_state == DSP_INTERRUPT_NONE)
		{
			hc_state = HC_NONE;                  // a fast interrupt: done
			d2h(R_HCACK, 0);
		}
		break;
	case HC_LONG:
		hc_count++;
		if ((dsp_core.registers[DSP_REG_SP] & 0xF) <= hc_sp)
		{
			hc_state = HC_NONE;                  // RTI back to where it was taken
			d2h(R_HCACK, 0);
		}
		break;
	default:
		break;
	}
	if (hc_hold_left)
	{
		if (hc_state == HC_NONE || !--hc_hold_left)
		{
			hc_hold_left = 0;
			slice_break = 1;                     // the host's next accesses now
		}
	}
	if (hc_state != HC_NONE && hc_count >= HC_WATCHDOG)
	{
		printf("[next-dsp] host command handler did not return (pc %04x): acked\n", dsp_core.pc);
		hc_state = HC_NONE;
		d2h(R_HCACK, 0);
	}
	isr_seen = isr();
}

// service() has work: host words queued, a command parked or in its
// handler, a post-command hold, or the DSP changed the host port's ISR
// (RXDF from HTX, HF2/HF3 from HCR, TXDE as it reads HRX).  Anything the
// FPGA sends comes through h2d_poll, which link_step follows with service().
static inline int link_busy(void)
{
	return txq_n || parked || hc_state != HC_NONE || hc_hold_left || isr() != isr_seen;
}

// hooks from the core (next_dsp_shim.h)
extern "C" void dsp_link_hrx_read(void)
{
	if (tx_outstanding > 0)
	{
		tx_outstanding--;
		d2h(R_HRXACK, 0);
	}
}

extern "C" void dsp_link_txd(int low)
{
	if (low != last_txd)
	{
		last_txd = low;
		d2h(R_TXD, low ? 1 : 0);
	}
}

extern "C" void dsp_link_hi_reset(void)
{
	// the RESET instruction: the host side goes back to ICR 0, CVR $12,
	// ISR TRDY|TXDE.  Words in TX are gone; HRX stays for the DSP to read.
	int keep = (hsr() & (1 << DSP_HOST_HSR_HRDF)) ? 1 : 0;
	txq_rd = txq_n = 0;
	d2h(R_HIRESET, 0);
	while (tx_outstanding > keep)
	{
		tx_outstanding--;
		d2h(R_HRXACK, 0);
	}
	hc_wait_taken = 0;
}

extern "C" void dsp_link_hc_taken(void)
{
	if (hc_state != HC_NONE) d2h(R_HCACK, 0);   // the previous one cannot still be running
	hc_state = HC_PIPE;
	hc_sp = dsp_core.registers[DSP_REG_SP] & 0xF;
	hc_count = 0;
	hc_hold_left = HC_POST_HOLD;
}

extern "C" void next_dsp_log(int level, const char *fmt, ...)
{
	if (level > 2) return;                       // LOG_WARN and worse
	va_list ap;
	va_start(ap, fmt);
	printf("[next-dsp] ");
	vprintf(fmt, ap);
	va_end(ap);
}

//----------------------------------------------------------------------------
// H2D messages
//----------------------------------------------------------------------------

static uint64_t h2d_at(uint16_t i) { return rd64(OFF_H2D + 8 * (i & 0xFF)); }
static int h2d_type(uint64_t m) { return (int)((m >> 56) & 0xF); }

// handle one message; 0 = it must wait (left in the ring)
static int h2d_handle(uint64_t m)
{
	int type = h2d_type(m);
	uint8_t ep = (uint8_t)(m >> 48);
	uint32_t data = (uint32_t)m & 0xFFFFFF;

	// The host's ICR / CVR writes reach the DSP after the words it wrote
	// before them have been read: the bootstrap ends with HF0 right after
	// the last word, and a host command follows its message's words.  A
	// host command waiting for its words parks (later words and commands
	// wait behind it) so the host's ICR writes still get through -- a DSP
	// in a handler spinning on HF1 cannot read them before.  An ICR waits
	// at most ICR_HOLD instructions; an INIT throws the words away instead.
	if (dsp_core.running && !tx_drained())
	{
		if (type == T_CVR && (data & 0x80) && !parked)
		{
			parked = 1; parked_cvr = (uint8_t)data; park_since = instrs;
			epoch = ep;
			msgs++;
			return 1;
		}
		if (type == T_ICR && !(data & 0x80))
		{
			if (!hold_on) { hold_on = 1; hold_since = instrs; }
			if (instrs - hold_since < ICR_HOLD) return 0;
		}
	}
	hold_on = 0;
	if (parked && (type == T_TX || type == T_CVR)) return 0;

	epoch = ep;
	msgs++;

	switch (type)
	{
	case T_TX:
		if (txq_n < TXQ_N)
		{
			txq[(txq_rd + txq_n) % TXQ_N] = data;
			txq_n++;
			// a 56001 reads a host word within microseconds: the host gets
			// TXDE for it at once while a few wait here (the DSP reads them
			// later, in order); beyond that only when the DSP reads one
			if (txq_n <= TX_EARLY_ACK) d2h(R_HRXACK, 0);
			else tx_outstanding++;
		}
		else printf("[next-dsp] host TX overrun\n");
		tx_feed();
		break;

	case T_ICR:
		if (data & 0x80)                         // INIT
		{
			if (data & 0x01) rx_out = 0;
			if (data & 0x02) { txq_rd = txq_n = 0; tx_outstanding = 0; }
		}
		dsp_core_write_host(CPU_HOST_ICR, (uint8_t)data);
		break;

	case T_CVR:
		dsp_core_write_host(CPU_HOST_CVR, (uint8_t)data);
		if (data & 0x80) hc_wait_taken = 1;
		break;

	case T_RXACK:
		if (rx_out > 0) rx_out--;
		break;

	case T_RESET:
		dsp_full_reset();
		last_txd = 0;
		break;

	case T_START:
		dsp_core_start((uint8_t)(data & 3), 1);
		break;

	case T_MEM:
		if (data & 1) dsp_core_config_ramext(dsp_ram, dsp_ram_words);
		else dsp_core_config_ramext(NULL, 0);
		break;

	case T_IRQB:
		dsp_set_interrupt(DSP_INTER_IRQB, 1);
		break;

	default:
		printf("[next-dsp] unknown message %016llx\n", (unsigned long long)m);
		break;
	}
	return 1;
}

// skip to a DSP reset waiting behind a held message (a reset is never held)
static int h2d_skip_to_reset(void)
{
	for (uint16_t i = h2d_rd; i != f_h2d_wr; i++)
	{
		if (h2d_type(h2d_at(i)) == T_RESET)
		{
			h2d_rd = i;
			arm_dirty = 1;
			hc_wait_taken = 0;
			hc_hold_left = 0;
			hold_on = 0;
			parked = 0;
			return 1;
		}
	}
	return 0;
}

static void h2d_poll(void)
{
	while (h2d_rd != f_h2d_wr)
	{
		// held behind a host command the DSP has not taken yet, and for
		// the first instructions of its handler
		if ((hc_wait_taken && (hsr() & (1 << DSP_HOST_HSR_HCP))) ||
		    (hc_state != HC_NONE && hc_hold_left))
		{
			if (!h2d_skip_to_reset()) return;
		}
		hc_wait_taken = 0;
		hc_hold_left = 0;

		if (!h2d_handle(h2d_at(h2d_rd)))
		{
			if (!h2d_skip_to_reset()) return;
			continue;
		}
		h2d_rd++;
		arm_dirty = 1;
	}
}

//----------------------------------------------------------------------------
// one step: the FPGA word, messages, up to n instructions
//----------------------------------------------------------------------------

static uint64_t step_cycles;             // DSP clocks run (pacing)

static void fpga_word(void)
{
	uint64_t f = rd64(OFF_FPGA);
	if ((f >> 48) != FPGA_MAGIC)
	{
		linked = 0;
		return;
	}
	uint16_t g = (uint16_t)(f >> 32);
	if (!linked || g != gen)
	{
		// the FPGA sends and reads nothing before the ARM word names its
		// gen, so a new generation's indices are 0: anything else is a
		// word left in DDR3 by an earlier core load (wait for the new one)
		if ((uint32_t)f != 0)
		{
			linked = 0;
			return;
		}
		// a new core generation: the FPGA restarted its indices
		gen = g;
		linked = 1;
		h2d_rd = d2h_wr = 0;
		pend_rd = pend_n = 0;
		epoch = 0;
		dsp_full_reset();
		dsp_core_config_ramext(NULL, 0);
		f_h2d_wr = 0;
		f_d2h_rd = 0;
		arm_word();
		printf("[next-dsp] linked to core generation %u\n", g);
		f = rd64(OFF_FPGA);
		if ((uint16_t)(f >> 32) != gen) return;
	}
	f_h2d_wr = (uint16_t)f;
	f_d2h_rd = (uint16_t)(f >> 16);
}

static void debug_word(void)
{
	uint32_t flags = (linked ? 4 : 0) | (dsp_core.running ? 2 : 0) | (dsp_core.mode == 1 ? 1 : 0);
	wr64(OFF_DEBUG, ((uint64_t)(instrs >> 10) << 32) | (flags << 28) | ((msgs & 0xFFF) << 16) | dsp_core.pc);
}

static int link_step(int n)
{
	int done = 0;
	fpga_word();
	if (!linked) return 0;

	d2h_flush();
	h2d_poll();
	service();

	slice_break = 0;
	while (done < n && dsp_core.running && pend_n < PENDQ_N / 2)
	{
		dsp56k_execute_instruction();
		step_cycles += dsp_core.instr_cycle;
		done++;
		instrs++;
		if (link_busy()) service();
		if (slice_break) break;
	}

	if (slice_break || hc_wait_taken)
	{
		// a host command just taken: the host's next accesses first
		fpga_word();
		h2d_poll();
		service();
	}
	d2h_flush();
	if (arm_dirty) arm_word();
	return done;
}

//----------------------------------------------------------------------------
// bench entry points
//----------------------------------------------------------------------------

void next_dsp_sim_attach(next_dsp_rd_t rd, next_dsp_wr_t wr)
{
	sim_rd = rd;
	sim_wr = wr;
	linked = 0;
}

int next_dsp_sim_step(int n)
{
	int r = link_step(n);
	debug_word();
	return r;
}

//----------------------------------------------------------------------------
// the thread (MiSTer)
//----------------------------------------------------------------------------

#ifdef __linux__

static pthread_t thr;
static volatile int thr_run;
static int thr_started;

static uint64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static void *dsp_thread(void *)
{
	uint64_t t0 = now_ns(), last_dbg = 0;
	step_cycles = 0;

	while (thr_run)
	{
		int n = link_step(256);
		uint64_t t = now_ns();

		if (t - last_dbg > 10000000ull)
		{
			last_dbg = t;
			debug_word();
		}

		if (!linked)
		{
			usleep(20000);
			t0 = now_ns(); step_cycles = 0;
			continue;
		}
		if (!dsp_core.running || n == 0)
		{
			// in reset, or held: the FPGA polls every 4 us while it runs
			usleep(dsp_core.running ? 20 : 500);
			t0 = now_ns(); step_cycles = 0;
			continue;
		}

		// never faster than the real DSP: DSP_HZ clocks per second
		uint64_t due = t0 + step_cycles * 1000000000ull / DSP_HZ;
		if (due > t + 200000)
		{
			struct timespec ts = { 0, (long)(due - t) };
			nanosleep(&ts, NULL);
		}
		else if (t > due + 50000000ull)
		{
			// far behind (the interpreter is slower than the DSP): don't bank it
			t0 = t; step_cycles = 0;
		}
	}
	return NULL;
}

void next_dsp_start(void)
{
	if (thr_started) return;
	if (!mb)
	{
		mb = (volatile uint32_t *)shmem_map(DL_BASE, DL_SIZE);
		if (!mb)
		{
			printf("[next-dsp] shmem_map failed\n");
			return;
		}
	}
	linked = 0;
	// an ARM word left by an earlier core load must not link the new FPGA
	wr64(OFF_ARM, 0);
	thr_run = 1;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0, &set);                        // Main runs on CPU 1
	pthread_attr_setaffinity_np(&attr, sizeof(set), &set);
	if (pthread_create(&thr, &attr, dsp_thread, NULL) == 0)
	{
		thr_started = 1;
		printf("[next-dsp] DSP56001 thread started, waiting for the core link\n");
	}
	else
	{
		thr_run = 0;
		printf("[next-dsp] pthread_create failed\n");
	}
	pthread_attr_destroy(&attr);
}

void next_dsp_stop(void)
{
	if (!thr_started) return;
	thr_run = 0;
	pthread_join(thr, NULL);
	wr64(OFF_ARM, 0);
	thr_started = 0;
	linked = 0;
	printf("[next-dsp] stopped\n");
}

#else  // not Linux: benches only
void next_dsp_start(void) { }
void next_dsp_stop(void) { }
#endif
