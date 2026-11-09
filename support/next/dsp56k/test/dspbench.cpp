// dspbench: DSP56001 interpreter speed on the MiSTer A9 (thread CPU time).
// Build (build box, from Main_MiSTer support/next):
//   CC=arm-none-linux-gnueabihf-; for f in dsp_core dsp_cpu dsp_disasm; do ${CC}gcc -O3 -Idsp56k -c dsp56k/$f.c; done
//   ${CC}g++ -O3 -Idsp56k -I. -c next_dsp.cpp dspbench.cpp
//   echo 'void *shmem_map(unsigned,unsigned){return 0;}' > stub.cpp && ${CC}g++ -c stub.cpp
//   ${CC}g++ -o dspbench *.o -lpthread
// Run on the MiSTer: ./dspbench [kernel] [link] [n]
//   kernel boot: the bootstrap ROM's idle loop (JCLR on the host port)
//   kernel fir:  16-tap FIR, MAC with XY moves, modulo 16, DO loop (P:$40)
//   link 0: the interpreter alone; 1: through next_dsp_sim_step (in-memory mailbox)
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
extern "C" {
#include "dsp_core.h"
#include "dsp_cpu.h"
}
#include "next_dsp.h"

static double cpu_s() { timespec t; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void hi(int) {}
static uint64_t mb[0x4000 / 8];
static uint64_t rd(uint32_t o) { return mb[o / 8]; }
static void wr(uint32_t o, uint64_t v) { mb[o / 8] = v; }

static void load_fir(void)
{
	static const uint32_t prog[] = {
		0xf09813,            // 40 clr a        x:(r0)+,x0 y:(r4)+,y0
		0x061080, 0x000043,  // 41 do #16,p:$43
		0xf098d2,            // 43 mac +y0,x0,a x:(r0)+,x0 y:(r4)+,y0
		0x200011,            // 44 rnd a
		0x565900,            // 45 move a,x:(r1)+
		0x0c0040,            // 46 jmp p:$40
	};
	for (unsigned i = 0; i < sizeof(prog) / 4; i++) dsp_core.ramint[DSP_SPACE_P][0x40 + i] = prog[i];
	for (int i = 0; i < 256; i++) { dsp_core.ramint[DSP_SPACE_X][i] = (i * 0x12345) & 0xffffff; dsp_core.ramint[DSP_SPACE_Y][i] = (i * 0x54321) & 0xffffff; }
	dsp_core.registers[DSP_REG_M0] = dsp_core.registers[DSP_REG_M4] = dsp_core.registers[DSP_REG_M1] = 15;
	dsp_core.registers[DSP_REG_R0] = 0; dsp_core.registers[DSP_REG_R4] = 0; dsp_core.registers[DSP_REG_R1] = 0x80;
	dsp_core.registers[DSP_REG_M1] = 0xffff;
	dsp_core.mode = 0;
	dsp_core.pc = 0x40;
}

int main(int argc, char **argv)
{
	const char *k = argc > 1 ? argv[1] : "boot";
	int link = argc > 2 ? atoi(argv[2]) : 0;
	long n = argc > 3 ? atol(argv[3]) : 20000000;
	int fir = !strcmp(k, "fir");
	double t0, t1;
	long done = 0;
	if (link) {
		next_dsp_sim_attach(rd, wr);
		mb[0] = (0xD5F1ull << 48) | (1ull << 32);           // gen 1, indices 0
		next_dsp_sim_step(1);                               // link
		mb[0x1000 / 8] = (6ull << 56) | 1;                  // T_START mode 1
		mb[0] = (0xD5F1ull << 48) | (1ull << 32) | 1;       // h2d_wr = 1
		next_dsp_sim_step(16);
		if (fir) load_fir();
		t0 = cpu_s();
		while (done < n) { int s = next_dsp_sim_step(256); if (!s) { printf("stalled\n"); break; } done += s; }
		t1 = cpu_s();
	} else {
		dsp_core_init(hi);
		dsp_core_reset();
		dsp_core_start(1, 1);
		if (fir) load_fir();
		for (int i = 0; i < 1000; i++) dsp56k_execute_instruction();
		t0 = cpu_s();
		for (; done < n; done++) dsp56k_execute_instruction();
		t1 = cpu_s();
	}
	printf("%-4s link %d: %ld instr in %.3f s CPU = %5.2f MIPS  pc %04x a %06x\n", k, link, done, t1 - t0,
	       done / (t1 - t0) / 1e6, dsp_core.pc, dsp_core.registers[DSP_REG_A1]);
	return 0;
}
