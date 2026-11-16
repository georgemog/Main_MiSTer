#ifndef NEXT_DSP_H
#define NEXT_DSP_H

#include <stdint.h>

// The NeXT-Color's DSP56001, run on the ARM.
//
// The core keeps the 68040's side of the DSP host port in the FPGA
// (rtl/tc_dsp.sv: ICR, CVR, ISR, IVR, TX, RX, the DMA byte pump, the
// interrupt), so the CPU never waits for the ARM.  The DSP itself -- its
// side of the host port (HCR, HSR, HRX, HTX), memories and instruction
// set -- runs here, in Hatari's DSP56001 core (support/next/dsp56k) with
// the NeXT changes Previous carries.  The two halves trade host-port events
// through a mailbox in DDR3 at 0x30400000 (next_dsp.cpp has the protocol).
//
// A core without the link (the mono NeXT core, NeXT-Color builds with the
// DSP in the FPGA) never writes the FPGA word's magic, so the thread just
// waits.

void next_dsp_start(void);
void next_dsp_stop(void);

// For benches (NeXT-Color verilator/unit/tb_tc_dsp): the mailbox through
// callbacks (byte offsets from 0x30400000), stepped from the bench instead
// of a thread.  next_dsp_sim_step runs up to n DSP instructions and returns
// how many it ran.
typedef uint64_t (*next_dsp_rd_t)(uint32_t off);
typedef void (*next_dsp_wr_t)(uint32_t off, uint64_t v);
void next_dsp_sim_attach(next_dsp_rd_t rd, next_dsp_wr_t wr);
int  next_dsp_sim_step(int n);

// Profiling (JIT plan phase 0; on the MiSTer also via the prof and dump
// files in /media/usb0/next_dsp, see next_dsp.cpp): counters on/off, and a dump of the
// DSP's memories, registers and the counters for support/next/dsp56k/test/dspprof.
void next_dsp_prof(int on);
int  next_dsp_prof_dump(const char *path);

// Host-stall diagnosis, called from Main's thread: one NeXT disk request
// (times from next_dsp_now_us, CLOCK_MONOTONIC), and one call per
// user_io_poll pass.  They go into the link trace's timeline and stats.log.
uint32_t next_dsp_now_us(void);
void next_dsp_note_disk(int disk, int op, uint32_t lba, uint32_t sz, uint32_t t0_us, uint32_t t1_us);
void next_dsp_main_tick(void);

#endif
