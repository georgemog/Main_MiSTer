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
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>

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
// RX words the ARM lets out before the host takes them (<= DSPLINK_DEPTH).
// A 56001 has two words between its program and the host (HTX and the
// host's RX); every word beyond that is one the host has not asked for
// yet, and at the end of a DMA block the host reads those as DSP messages.
// A debug knob: the file NEXT_DSP_DIR/rxdepth overrides it (see profiling).
static int rx_depth = DSPLINK_DEPTH;
#define DSP_HZ         25000000          // the DSP's clock; instr_cycle counts it
#define HC_WATCHDOG    (1u << 20)        // instructions before an unreturned handler is acked
#define DSP_RAM_WORDS  8192              // the NeXT's 8K words (24 KB); 32768 with the expansion

// log lines go out at once (Main's stdout is a file, fully buffered)
static void nlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void nlog(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	fputs("[next-dsp] ", stdout);
	vprintf(fmt, ap);
	va_end(ap);
	fflush(stdout);
}

// the link's event trace: the last TRACE_N host-port events, for a hang
// (written with the link state by the "trace" trigger file, see profiling)
enum { EV_H2D = 1, EV_HOLD, EV_PARK, EV_UNPARK, EV_D2H, EV_HCTAKEN, EV_HIRESET, EV_SKIPRESET };
struct trace_ev { uint64_t instrs; uint32_t us; uint16_t pc; uint8_t kind, type; uint32_t data; };
static int hf3_auto;                     // HF3 seen: dump the trace once (hf3_auto.txt)
#define TRACE_N 65536
static trace_ev trace_buf[TRACE_N];
static uint32_t trace_n;
static void trace(int kind, int type, uint32_t data);

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
// always-on load counters, logged while NEXT_DSP_DIR/stats exists (stats_poll)
static uint64_t st_cycles, st_sleep_ns, st_behind;

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
static uint64_t rx_words, tx_words;      // words to / from the host (debug, the trace)
static uint64_t starved_polls;           // ring polls while starved of RX credits (link_step)
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
	trace(EV_D2H, type, data & 0xFFFFFF);
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
		nlog("D2H overflow, message %d dropped\n", type);
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

static void trace(int kind, int type, uint32_t data)
{
	trace_ev &e = trace_buf[trace_n++ % TRACE_N];
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	e.instrs = instrs; e.us = (uint32_t)(ts.tv_sec * 1000000ull + ts.tv_nsec / 1000);
	e.pc = (uint16_t)dsp_core.pc; e.kind = (uint8_t)kind; e.type = (uint8_t)type; e.data = data;
}

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
		if (!tx_drained()) nlog("host command %02x applied with host words unread\n", parked_cvr);
		parked = 0;
		trace(EV_UNPARK, T_CVR, parked_cvr);
		dsp_core_write_host(CPU_HOST_CVR, parked_cvr);
		hc_wait_taken = 1;
		slice_break = 1;
	}

	// RX: the word goes to the FPGA, then the local host takes it so the
	// DSP sees HTDE again
	while ((isr() & (1 << CPU_HOST_ISR_RXDF)) && rx_out < rx_depth && pend_n < PENDQ_N / 2)
	{
		uint32_t w = ((uint32_t)dsp_core.hostport[CPU_HOST_RXH] << 16) |
		             ((uint32_t)dsp_core.hostport[CPU_HOST_RXM] << 8) |
		             dsp_core.hostport[CPU_HOST_RXL];
		d2h(R_RX, w);
		rx_out++;
		rx_words++;
		dsp_core_read_host(CPU_HOST_TRXL);
	}

	uint8_t hf = isr() & ((1 << CPU_HOST_ISR_HF2) | (1 << CPU_HOST_ISR_HF3));
	if (hf != last_hf)
	{
		last_hf = hf;
		d2h(R_FLAGS, hf);
		if ((hf & (1 << CPU_HOST_ISR_HF3)) && !hf3_auto) hf3_auto = 1;
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
		nlog("host command handler did not return (pc %04x): acked\n", dsp_core.pc);
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
	trace(EV_HIRESET, 0, keep);
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
	trace(EV_HCTAKEN, hc_state, dsp_core.registers[DSP_REG_SP] & 0xF);
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
	fputs("[next-dsp] ", stdout);
	vprintf(fmt, ap);
	va_end(ap);
	fflush(stdout);
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
			trace(EV_PARK, type, data);
			epoch = ep;
			msgs++;
			return 1;
		}
		if (type == T_ICR && !(data & 0x80))
		{
			if (!hold_on) { hold_on = 1; hold_since = instrs; trace(EV_HOLD, type, data); }
			if (instrs - hold_since < ICR_HOLD) return 0;
		}
	}
	hold_on = 0;
	if (parked && (type == T_TX || type == T_CVR)) return 0;

	epoch = ep;
	msgs++;
	trace(EV_H2D, type, data | ((uint32_t)ep << 24));

	switch (type)
	{
	case T_TX:
		if (txq_n < TXQ_N)
		{
			txq[(txq_rd + txq_n) % TXQ_N] = data;
			txq_n++;
			tx_words++;
			// a 56001 reads a host word within microseconds: the host gets
			// TXDE for it at once while a few wait here (the DSP reads them
			// later, in order); beyond that only when the DSP reads one
			if (txq_n <= TX_EARLY_ACK) d2h(R_HRXACK, 0);
			else tx_outstanding++;
		}
		else nlog("host TX overrun\n");
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
		nlog("unknown message %016llx\n", (unsigned long long)m);
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
			trace(EV_SKIPRESET, T_RESET, (uint16_t)(i - h2d_rd));
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
		// (not while the DSP cannot take it: in reset or in the bootstrap
		// ROM, a 56001's host still writes TX and ICR with HC pending, and
		// the bootstrap needs those words)
		if ((hc_wait_taken && (hsr() & (1 << DSP_HOST_HSR_HCP)) && dsp_core.running && dsp_core.mode != 1) ||
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
		nlog("linked to core generation %u\n", g);
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

//----------------------------------------------------------------------------
// profiling (JIT plan, phase 0): what the DSP runs, for offline analysis
//----------------------------------------------------------------------------
// On the MiSTer, in NEXT_DSP_DIR (on the USB disk, not the /tmp tmpfs):
//                 touch prof          counters on (cleared when armed)
//                 echo LABEL > dump   writes LABEL.bin
//                 rm prof             counters off
//                 echo LABEL > trace  writes LABEL.txt: the link state and
//                                     its last 64K host-port events
// The dump (little endian) is a prof_hdr, then registers[64], stack[2][16]
// (u16), rom[3][512], ramint[3][512], periph[2][64], ramext[ramext_words],
// the PC histogram[65536], the non-parallel opcode histogram[512] (Hatari's
// opcodes8h index), the parallel-move histogram[16*256] (pm type * ALU byte)
// and the interrupt vector histogram[64].  support/next/dsp56k/test/dspprof.c
// reads it.

struct prof_hdr
{
	char     magic[8];          // "NXDSPPR2"
	uint32_t version, hdr_size;
	uint64_t instrs, cycles, jumps, same_pc, wall_ns, sleep_ns, behind;
	uint32_t ramext_words, mode;
	uint16_t pc, pad;
	uint32_t interrupt_state, loop_rep;
	uint64_t boot_instrs;       // of instrs: in the bootstrap ROM (mode 1), not in the histograms
};

static int prof_on;
static uint32_t prof_pc[65536], prof_np[512], prof_par[16 * 256], prof_irq[64];
static uint64_t prof_instrs, prof_cycles, prof_jumps, prof_same, prof_t0, prof_sleep, prof_behind, prof_boot;

static uint64_t prof_now(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static uint32_t p_word(uint16_t pc)
{
	if (dsp_core.mode == 1) return dsp_core.rom[DSP_SPACE_P][pc & 0x1f];
	if (pc < 0x200) return dsp_core.ramint[DSP_SPACE_P][pc];
	return dsp_core.ramext && DSP_RAMSIZE ? dsp_core.ramext[pc & (DSP_RAMSIZE - 1)] : 0;
}

static void prof_reset(void)
{
	memset(prof_pc, 0, sizeof(prof_pc));
	memset(prof_np, 0, sizeof(prof_np));
	memset(prof_par, 0, sizeof(prof_par));
	memset(prof_irq, 0, sizeof(prof_irq));
	prof_instrs = prof_cycles = prof_jumps = prof_same = prof_sleep = prof_behind = prof_boot = 0;
	prof_t0 = prof_now();
}

// one instruction, run with the counters on (link_step's slow path)
static void prof_execute(void)
{
	if (dsp_core.mode == 1)
	{
		// the bootstrap ROM (loading a program, or idle waiting for one):
		// counted apart so its PCs don't mix with the program's
		dsp56k_execute_instruction();
		prof_boot++;
		prof_instrs++;
		prof_cycles += dsp_core.instr_cycle;
		return;
	}
	uint16_t pc0 = dsp_core.pc;
	uint32_t op = p_word(pc0) & 0xffffff;
	uint16_t ist = dsp_core.interrupt_state;

	dsp56k_execute_instruction();

	prof_pc[pc0]++;
	if (op < 0x100000) prof_np[((op >> 11) & (0x3f << 3)) + ((op >> 5) & 7)]++;
	else prof_par[((op >> 20) & 0xf) * 256 + (op & 0xff)]++;
	uint16_t d = (uint16_t)(dsp_core.pc - pc0);
	if (d == 0) prof_same++;
	else if (d > 2) prof_jumps++;
	if (ist == DSP_INTERRUPT_NONE && dsp_core.interrupt_state != DSP_INTERRUPT_NONE)
		prof_irq[(dsp_core.interrupt_instr_fetch >> 1) & 63]++;
	prof_instrs++;
	prof_cycles += dsp_core.instr_cycle;
}

static int prof_dump(const char *path)
{
	FILE *f = fopen(path, "wb");
	if (!f) return -1;
	prof_hdr h;
	memset(&h, 0, sizeof(h));
	memcpy(h.magic, "NXDSPPR2", 8);
	h.version = 2;
	h.hdr_size = sizeof(h);
	h.instrs = prof_instrs; h.cycles = prof_cycles; h.jumps = prof_jumps; h.same_pc = prof_same;
	h.wall_ns = prof_now() - prof_t0; h.sleep_ns = prof_sleep; h.behind = prof_behind;
	h.ramext_words = dsp_core.ramext ? DSP_RAMSIZE : 0;
	h.mode = dsp_core.mode; h.pc = dsp_core.pc;
	h.interrupt_state = dsp_core.interrupt_state; h.loop_rep = dsp_core.loop_rep;
	h.boot_instrs = prof_boot;
	fwrite(&h, sizeof(h), 1, f);
	fwrite(dsp_core.registers, sizeof(dsp_core.registers), 1, f);
	fwrite(dsp_core.stack, sizeof(dsp_core.stack), 1, f);
	fwrite(dsp_core.rom, sizeof(dsp_core.rom), 1, f);
	fwrite(dsp_core.ramint, sizeof(dsp_core.ramint), 1, f);
	fwrite(dsp_core.periph, sizeof(dsp_core.periph), 1, f);
	if (h.ramext_words) fwrite(dsp_core.ramext, 4, h.ramext_words, f);
	fwrite(prof_pc, sizeof(prof_pc), 1, f);
	fwrite(prof_np, sizeof(prof_np), 1, f);
	fwrite(prof_par, sizeof(prof_par), 1, f);
	fwrite(prof_irq, sizeof(prof_irq), 1, f);
	int r = ferror(f) ? -1 : 0;
	fclose(f);
	return r;
}

void next_dsp_prof(int on)
{
	if (on && !prof_on) prof_reset();
	prof_on = on;
}

int next_dsp_prof_dump(const char *path)
{
	return prof_dump(path);
}


// the link state and the trace, as text (the "trace" trigger file)
static int trace_dump(const char *path)
{
	static const char *hn[] = { "?", "T_TX", "T_ICR", "T_CVR", "T_RXACK", "T_RESET", "T_START", "T_MEM", "T_IRQB" };
	static const char *dn[] = { "?", "R_RX", "R_HRXACK", "R_FLAGS", "R_HCACK", "R_TXD", "R_HIRESET" };
	static const char *kn[] = { "?", "h2d", "HOLD", "PARK", "UNPARK", "d2h", "HC-TAKEN", "HI-RESET", "SKIP-TO-RESET" };
	FILE *f = fopen(path, "w");
	if (!f) return -1;
	fprintf(f, "linked %d gen %u running %d mode %u pc %04x instrs %llu msgs %u epoch %02x\n",
	        linked, gen, dsp_core.running, dsp_core.mode, dsp_core.pc, (unsigned long long)instrs, msgs, epoch);
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	fprintf(f, "monotonic %ld.%03ld s  words to host %llu, from host %llu, starved polls %llu\n",
	        (long)ts.tv_sec, ts.tv_nsec / 1000000, (unsigned long long)rx_words, (unsigned long long)tx_words,
	        (unsigned long long)starved_polls);
	fprintf(f, "h2d_rd %u fpga h2d_wr %u (%u waiting)  d2h_wr %u fpga d2h_rd %u  pend_n %d\n",
	        h2d_rd, f_h2d_wr, (uint16_t)(f_h2d_wr - h2d_rd), d2h_wr, f_d2h_rd, pend_n);
	fprintf(f, "rx_depth %d  ", rx_depth);
	fprintf(f, "txq_n %d tx_outstanding %d rx_out %d last_hf %02x parked %d (cvr %02x) hold_on %d\n",
	        txq_n, tx_outstanding, rx_out, last_hf, parked, parked_cvr, hold_on);
	fprintf(f, "hc_state %d hc_wait_taken %d hc_hold_left %u hc_count %u\n", hc_state, hc_wait_taken, hc_hold_left, hc_count);
	fprintf(f, "host side: ICR %02x CVR %02x ISR %02x; DSP side: HCR %02x HSR %02x; interrupt_state %u SR %06x SP %02x\n",
	        dsp_core.hostport[CPU_HOST_ICR], dsp_core.hostport[CPU_HOST_CVR], isr(),
	        dsp_core.periph[DSP_SPACE_X][DSP_HOST_HCR], hsr(), dsp_core.interrupt_state,
	        dsp_core.registers[DSP_REG_SR], dsp_core.registers[DSP_REG_SP]);
	fprintf(f, "\nH2D waiting in the ring:\n");
	for (uint16_t i = h2d_rd; i != f_h2d_wr && (uint16_t)(i - h2d_rd) < 64; i++)
	{
		uint64_t m = h2d_at(i);
		int t = h2d_type(m);
		fprintf(f, "  [%u] %-8s %06x epoch %02x\n", i, t < 9 ? hn[t] : "?", (unsigned)(m & 0xFFFFFF), (unsigned)(uint8_t)(m >> 48));
	}
	uint32_t n = trace_n < TRACE_N ? trace_n : TRACE_N;
	fprintf(f, "\nlast %u events (instrs, us, pc, event):\n", n);
	for (uint32_t k = trace_n - n; k != trace_n; k++)
	{
		const trace_ev &e = trace_buf[k % TRACE_N];
		const char *tn = e.kind == EV_D2H ? (e.type < 7 ? dn[e.type] : "?") : (e.type < 9 ? hn[e.type] : "?");
		if (e.kind == EV_HCTAKEN || e.kind == EV_HIRESET) tn = "";
		fprintf(f, "%12llu %10u %04x %-13s %-8s %06x\n", (unsigned long long)e.instrs, e.us, e.pc,
		        e.kind < 9 ? kn[e.kind] : "?", tn, e.data);
	}
	int r = ferror(f) ? -1 : 0;
	fclose(f);
	return r;
}

// the bootstrap ROM waiting for the host (p:$13 JCLR #3,X:$FFE9 / p:$17
// JCLR #0,X:(R2) with R2 = $FFE9): with HF0 and HRDF clear a pass changes
// nothing but the cycle count, and only a host write sets them, so don't
// spin there (the thread sleeps until the link brings something)
static int boot_idle(void)
{
	return dsp_core.mode == 1 && (dsp_core.pc == 0x13 || dsp_core.pc == 0x17) &&
	       dsp_core.registers[DSP_REG_R2] == 0xffe9 &&
	       !(dsp_core.periph[DSP_SPACE_X][DSP_HOST_HSR] & ((1 << DSP_HOST_HSR_HRDF) | (1 << DSP_HOST_HSR_HF0))) &&
	       !dsp_core.loop_rep && !dsp_core.mode_wait && dsp_core.interrupt_state == DSP_INTERRUPT_NONE &&
	       !dsp_core.agu_pipeline_reg[0] && !dsp_core.agu_pipeline_reg[1] &&
	       !(dsp_core.registers[DSP_REG_SR] & (1 << DSP_SR_T)) &&
	       !(dsp_core.interrupt_status & DSP_INTER_NMI_MASK) &&
	       !(dsp_core.interrupt_status & dsp_core.interrupt_enable & dsp_core.interrupt_mask);
}
static uint64_t st_boot_idle;            // link slices that found the bootstrap ROM idle

#define STARVED_POLL   16                // instructions between ring polls while starved (a power of 2)

static int link_step(int n)
{
	int done = 0;
	fpga_word();
	if (!linked) return 0;

	d2h_flush();
	h2d_poll();
	service();

	slice_break = 0;
	int last_poll = 0;
	while (done < n && dsp_core.running && pend_n < PENDQ_N / 2)
	{
		int k = 1;
		if (boot_idle()) { st_boot_idle++; break; }
		if (prof_on) { prof_execute(); step_cycles += dsp_core.instr_cycle; st_cycles += dsp_core.instr_cycle; }
		else
		{
			// one compiled block (dsp_jit.inc), or one instruction
			uint32_t c = 0;
			k = dsp56k_jit_step(&c);
			step_cycles += c;
			st_cycles += c;
		}
		done += k;
		instrs += k;
		if (link_busy()) service();
		// publish at once: the FPGA sees D2H messages (and our H2D read
		// index) only through the ARM word, so a word sent mid-slice would
		// otherwise wait for the slice to end
		if (arm_dirty) arm_word();
		if (slice_break) break;
		// starved: the DSP has a word for the host and every RX credit is
		// out.  The credits come back as T_RXACK, seen only when the ring is
		// polled, so poll it every STARVED_POLL instructions instead of once
		// a slice (the window is DSPLINK_DEPTH words per round trip)
		if (done - last_poll >= STARVED_POLL && rx_out >= rx_depth && (isr() & (1 << CPU_HOST_ISR_RXDF)))
		{
			last_poll = done;
			starved_polls++;
			fpga_word();
			if (f_h2d_wr != h2d_rd) { h2d_poll(); service(); if (arm_dirty) arm_word(); }
			if (slice_break) break;
		}
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

// the trigger files (see "profiling")
#ifndef NEXT_DSP_DIR
#define NEXT_DSP_DIR "/media/usb0/next_dsp"
#endif
static int prof_poll_n;
static void prof_poll(void)
{
	int armed = access(NEXT_DSP_DIR "/prof", F_OK) == 0;
	if (armed != prof_on) next_dsp_prof(armed);
	FILE *f = fopen(NEXT_DSP_DIR "/dump", "r");
	if (!f) return;
	char label[64] = "dump";
	if (fscanf(f, "%40[A-Za-z0-9_.-]", label) != 1) strcpy(label, "dump");
	fclose(f);
	unlink(NEXT_DSP_DIR "/dump");
	char path[128];
	snprintf(path, sizeof(path), NEXT_DSP_DIR "/%s.bin", label);
	nlog("profile %s: %s (%llu instructions)\n", path,
	       prof_dump(path) ? "write failed" : "written", (unsigned long long)prof_instrs);
}

static void knob_poll(void)
{
	FILE *f = fopen(NEXT_DSP_DIR "/rxdepth", "r");
	int d = DSPLINK_DEPTH;
	if (f)
	{
		if (fscanf(f, "%d", &d) != 1 || d < 1 || d > DSPLINK_DEPTH) d = DSPLINK_DEPTH;
		fclose(f);
	}
	if (d != rx_depth)
	{
		rx_depth = d;
		nlog("RX depth %d\n", d);
	}
}

static void trace_poll(void)
{
	// HF3 seen (the Music Kit's TMQ full) while NEXT_DSP_DIR/hf3arm exists:
	// the trace up to it, once
	if (hf3_auto == 1)
	{
		hf3_auto = 2;
		if (access(NEXT_DSP_DIR "/hf3arm", F_OK) == 0)
		{
			unlink(NEXT_DSP_DIR "/hf3arm");
			trace_dump(NEXT_DSP_DIR "/hf3_auto.txt");
		}
		else hf3_auto = 0;
	}
	FILE *f = fopen(NEXT_DSP_DIR "/trace", "r");
	if (!f) return;
	char label[64] = "trace";
	if (fscanf(f, "%40[A-Za-z0-9_.-]", label) != 1) strcpy(label, "trace");
	fclose(f);
	unlink(NEXT_DSP_DIR "/trace");
	char path[128];
	snprintf(path, sizeof(path), NEXT_DSP_DIR "/%s.txt", label);
	nlog("trace %s: %s\n", path, trace_dump(path) ? "write failed" : "written");
}

// every 2 s while NEXT_DSP_DIR/stats exists: how much of the time the DSP
// thread sleeps ahead of the real DSP's pace (its headroom), its CPU use,
// the instruction and cycle rates, fall-behind events, JIT activity
static void stats_poll(void)
{
	static int n, on;
	static uint64_t w0, cpu0, i0, c0, s0, b0;
	static uint32_t j0[8], y0[20];
	static uint64_t bi0;
	struct timespec ts;
	if (++n < 20) return;		// (called every ~100 ms)
	n = 0;
	int armed = access(NEXT_DSP_DIR "/stats", F_OK) == 0;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
	uint64_t cpu = (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec, w = now_ns();
	if (armed && on && w > w0)
	{
		double dw = (double)(w - w0);
		FILE *f = fopen(NEXT_DSP_DIR "/stats.log", "a");	// (Main's stdout is /dev/null unless debug is on)
		if (!f) goto out;
		fprintf(f, "%llu stats: asleep %4.1f%%  cpu %4.1f%%  %5.2f MIPS  %5.2f Mcycles/s  behind %llu  jit blocks %u interp %u compiles %u%s\n",
		     (unsigned long long)(w / 1000000000ull), 100.0 * (st_sleep_ns - s0) / dw, 100.0 * (cpu - cpu0) / dw,
		     (instrs - i0) * 1e3 / dw, (st_cycles - c0) * 1e3 / dw,
		     (unsigned long long)(st_behind - b0),
		     dsp_jit_stats[0] - j0[0], dsp_jit_stats[2] - j0[2], dsp_jit_stats[3] - j0[3],
		     !dsp_core.running ? "  (DSP in reset)" : st_boot_idle != bi0 ? "  (bootstrap ROM idle)" : "");
		/* why steps were interpreted, recompile reasons, and the most
		   interpreted PCs since the last line */
		fprintf(f, "      interp why: boot %u rep %u omrwait %u fastint %u agu %u trace %u intpend %u noblock %u agumis %u  recompile: start %u changed %u lainside %u clip %u  kept %u  irq fast %u\n      chain: %u, no: loop end %u state %u block %u\n",
		        dsp_jit_why[0] - y0[0], dsp_jit_why[1] - y0[1], dsp_jit_why[2] - y0[2], dsp_jit_why[3] - y0[3],
		        dsp_jit_why[4] - y0[4], dsp_jit_why[5] - y0[5], dsp_jit_why[6] - y0[6], dsp_jit_why[7] - y0[7],
		        dsp_jit_why[8] - y0[8], dsp_jit_stats[4] - j0[4], dsp_jit_stats[5] - j0[5], dsp_jit_stats[6] - j0[6],
		        dsp_jit_stats[7] - j0[7], dsp_jit_why[11] - y0[11], dsp_jit_why[10] - y0[10],
		        dsp_jit_why[12] - y0[12], dsp_jit_why[13] - y0[13], dsp_jit_why[14] - y0[14], dsp_jit_why[15] - y0[15]);
		if (dsp_jit_ihist) {
			fprintf(f, "      top interp:");
			for (int t = 0; t < 6; t++) {
				uint32_t best = 0, bp = 0;
				for (uint32_t p = 0; p < 65536; p++) if (dsp_jit_ihist[p] > best) { best = dsp_jit_ihist[p]; bp = p; }
				if (!best) break;
				fprintf(f, " %04x:%u", bp, best);
				dsp_jit_ihist[bp] = 0;
			}
			fprintf(f, "\n");
			memset(dsp_jit_ihist, 0, 65536 * 4);
		}
		fclose(f);
	}
	if (armed && !dsp_jit_ihist) dsp_jit_ihist = (uint32_t *)calloc(65536, 4);
out:
	on = armed;
	w0 = w; cpu0 = cpu; i0 = instrs; c0 = st_cycles; s0 = st_sleep_ns; b0 = st_behind; bi0 = st_boot_idle;
	memcpy(j0, dsp_jit_stats, sizeof(j0));
	memcpy(y0, dsp_jit_why, sizeof(y0));
}

static void *dsp_thread(void *)
{
	uint64_t t0 = now_ns(), last_dbg = 0;
	step_cycles = 0;
	uint32_t idle_run = 0;

	// CPU 0 is shared with the rest of Linux (sshd, offload, USB writes);
	// a busy task there at the same priority starved this thread, the
	// DSP fell behind and the Music Kit aborted the score and hung waiting
	// for a DSP left in the bootstrap ROM.  At nice -20 it still sleeps
	// when ahead, but ordinary work can no longer push it behind.
	if (setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), -20))
		nlog("dsp thread: setpriority failed\n");

	while (thr_run)
	{
		int n = link_step(256);
		uint64_t t = now_ns();

		if (t - last_dbg > 10000000ull)
		{
			last_dbg = t;
			debug_word();
			if (++prof_poll_n >= 10) { prof_poll_n = 0; prof_poll(); trace_poll(); knob_poll(); stats_poll(); }   // every ~100 ms
		}

		if (!linked)
		{
			usleep(20000);
			t0 = now_ns(); step_cycles = 0;
			continue;
		}
		if (!dsp_core.running || n == 0)
		{
			// in reset, or held: the FPGA polls every 4 us while it runs.
			// Idle in the bootstrap ROM for a while (no program being
			// loaded): poll the link as seldom as in reset
			idle_run = n == 0 && dsp_core.running && boot_idle() ? idle_run + 1 : 0;
			usleep(!dsp_core.running || idle_run > 200 ? 500 : 20);
			t0 = now_ns(); step_cycles = 0;
			continue;
		}

		// never faster than the real DSP: DSP_HZ clocks per second
		uint64_t due = t0 + step_cycles * 1000000000ull / DSP_HZ;
		if (due > t + 200000)
		{
			struct timespec ts = { 0, (long)(due - t) };
			nanosleep(&ts, NULL);
			if (prof_on) prof_sleep += due - t;
			st_sleep_ns += due - t;
		}
		else if (t > due + 50000000ull)
		{
			// far behind (the interpreter is slower than the DSP): don't bank it
			t0 = t; step_cycles = 0;
			if (prof_on) prof_behind++;
			st_behind++;
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
			nlog("shmem_map failed\n");
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
		nlog("DSP56001 thread started, waiting for the core link\n");
	}
	else
	{
		thr_run = 0;
		nlog("pthread_create failed\n");
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
	nlog("stopped\n");
}

#else  // not Linux: benches only
void next_dsp_start(void) { }
void next_dsp_stop(void) { }
#endif
