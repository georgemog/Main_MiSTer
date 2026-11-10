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
//   DSPPROF=file (with link 1): profile the run and write a dump for dspprof
// ./dspbench replay DUMP [n] [pc]: load a next_dsp profile dump (memories,
//   registers) and run its code from pc (default: the dump's) -- the hot
//   loops of a captured workload, offline.  Code that waits on the host port
//   spins, so pick a pc inside a compute loop (dspprof lists them).
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <ucontext.h>
#if defined(__linux__)
#include <linux/perf_event.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

// PERF=1: hardware counters over the timed run (perf_event_open)
#if defined(__linux__)
static const struct { uint32_t type; uint64_t cfg; const char *name; } perf_ev[] = {
	{PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES, "cycles"},
	{PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS, "instructions"},
	{PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_MISSES, "branch-misses"},
	{PERF_TYPE_HW_CACHE, PERF_COUNT_HW_CACHE_L1I | (PERF_COUNT_HW_CACHE_OP_READ << 8) | (PERF_COUNT_HW_CACHE_RESULT_MISS << 16), "L1I-misses"},
	{PERF_TYPE_HW_CACHE, PERF_COUNT_HW_CACHE_L1D | (PERF_COUNT_HW_CACHE_OP_READ << 8) | (PERF_COUNT_HW_CACHE_RESULT_MISS << 16), "L1D-read-misses"},
	{PERF_TYPE_HW_CACHE, PERF_COUNT_HW_CACHE_DTLB | (PERF_COUNT_HW_CACHE_OP_READ << 8) | (PERF_COUNT_HW_CACHE_RESULT_MISS << 16), "dTLB-misses"},
	{PERF_TYPE_HW_CACHE, PERF_COUNT_HW_CACHE_ITLB | (PERF_COUNT_HW_CACHE_OP_READ << 8) | (PERF_COUNT_HW_CACHE_RESULT_MISS << 16), "iTLB-misses"},
};
static int perf_fd[8];
static void perf_start(void)
{
	if (!getenv("PERF")) return;
	for (unsigned i = 0; i < sizeof(perf_ev) / sizeof(perf_ev[0]); i++) {
		struct perf_event_attr a; memset(&a, 0, sizeof(a));
		a.size = sizeof(a); a.type = perf_ev[i].type; a.config = perf_ev[i].cfg;
		perf_fd[i] = syscall(__NR_perf_event_open, &a, 0, -1, -1, 0);
		if (perf_fd[i] < 0) perror(perf_ev[i].name);
	}
}
static void perf_stop(long n)
{
	if (!getenv("PERF")) return;
	for (unsigned i = 0; i < sizeof(perf_ev) / sizeof(perf_ev[0]); i++) {
		uint64_t v = 0;
		if (perf_fd[i] >= 0 && read(perf_fd[i], &v, 8) == 8) printf("  %-16s %12llu  %7.2f per DSP instruction\n", perf_ev[i].name, (unsigned long long)v, (double)v / n);
		if (perf_fd[i] >= 0) close(perf_fd[i]);
	}
}
#else
static void perf_start(void) {}
static void perf_stop(long) {}
#endif

// PCSAMPLE=file: SIGPROF PC histogram (raw "addr count" lines, for addr2line)
static uint32_t *pcs_hist; static uintptr_t pcs_lo, pcs_n;
extern "C" char __executable_start[], etext[];
static void pcs_sig(int, siginfo_t *, void *uc)
{
#if defined(__arm__)
	uintptr_t pc = ((ucontext_t *)uc)->uc_mcontext.arm_pc;
#else
	uintptr_t pc = 0; (void)uc;
#endif
	if (pc >= pcs_lo && pc < pcs_lo + pcs_n * 4) pcs_hist[(pc - pcs_lo) / 4]++;
}
static void pcs_start(void)
{
	if (!getenv("PCSAMPLE")) return;
	pcs_lo = (uintptr_t)__executable_start; pcs_n = ((uintptr_t)etext - pcs_lo) / 4 + 1;
	pcs_hist = (uint32_t *)calloc(pcs_n, 4);
	struct sigaction sa; memset(&sa, 0, sizeof(sa)); sa.sa_sigaction = pcs_sig; sa.sa_flags = SA_SIGINFO | SA_RESTART;
	sigaction(SIGPROF, &sa, 0);
	struct itimerval it = {{0, 1000}, {0, 1000}}; setitimer(ITIMER_PROF, &it, 0);
}
static void pcs_stop(void)
{
	const char *f = getenv("PCSAMPLE");
	if (!f || !pcs_hist) return;
	struct itimerval it = {{0, 0}, {0, 0}}; setitimer(ITIMER_PROF, &it, 0);
	FILE *o = fopen(f, "w");
	for (uintptr_t i = 0; i < pcs_n; i++) if (pcs_hist[i]) fprintf(o, "%lx %u\n", (unsigned long)(pcs_lo + i * 4), pcs_hist[i]);
	fclose(o);
}
extern "C" {
#include "dsp_core.h"
#include "dsp_cpu.h"
}
#include "next_dsp.h"

static int jit_on() { const char *e = getenv("JIT"); return e && *e == '1'; }
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

struct prof_hdr {
	char     magic[8];
	uint32_t version, hdr_size;
	uint64_t instrs, cycles, jumps, same_pc, wall_ns, sleep_ns, behind;
	uint32_t ramext_words, mode;
	uint16_t pc, pad;
	uint32_t interrupt_state, loop_rep;
	uint64_t boot_instrs;
};

static int replay(const char *path, long n, int pc)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); return 1; }
	prof_hdr h;
	static uint32_t ram[32768];
	if (fread(&h, sizeof(h), 1, f) != 1 || memcmp(h.magic, "NXDSPPR2", 8)) { printf("not a dump\n"); return 1; }
	dsp_core_init(hi);
	dsp_core_reset();
	size_t ok = fread(dsp_core.registers, sizeof(dsp_core.registers), 1, f);
	ok += fread(dsp_core.stack, sizeof(dsp_core.stack), 1, f);
	ok += fread(dsp_core.rom, sizeof(dsp_core.rom), 1, f);
	ok += fread(dsp_core.ramint, sizeof(dsp_core.ramint), 1, f);
	ok += fread(dsp_core.periph, sizeof(dsp_core.periph), 1, f);
	if (h.ramext_words) { dsp_core_config_ramext(ram, h.ramext_words); ok += fread(ram, 4 * h.ramext_words, 1, f) ? 1 : 0; }
	fclose(f);
	dsp_core.mode = h.mode;
	dsp_core.pc = pc >= 0 ? pc : h.pc;
	dsp_core.running = 1;
	for (int i = 0; i < 1000; i++) dsp56k_execute_instruction();
	if (getenv("JITHIST")) { dsp_jit_ihist = (uint32_t *)calloc(65536, 4); dsp_jit_khist = (uint32_t *)calloc(65536, 4); }
	pcs_start(); perf_start();
	double t0 = cpu_s();
	if (jit_on()) { uint32_t c = 0; for (long i = 0; i < n; ) i += dsp56k_jit_step(&c); }
	else for (long i = 0; i < n; i++) dsp56k_execute_instruction();
	double t1 = cpu_s();
	pcs_stop(); perf_stop(n);
	if (jit_on() && dsp_jit_khist) {
		uint64_t tot = 0;
		for (uint32_t p = 0; p < 65536; p++) tot += dsp_jit_khist[p];
		printf("  handlers in blocks: %llu\n", (unsigned long long)tot);
		for (int t = 0; t < 8; t++) {
			uint32_t best = 0, bp = 0;
			for (uint32_t p = 0; p < 65536; p++) if (dsp_jit_khist[p] > best) { best = dsp_jit_khist[p]; bp = p; }
			if (!best) break;
			uint32_t w = bp < 0x200 ? dsp_core.ramint[DSP_SPACE_P][bp] : (dsp_core.ramext ? dsp_core.ramext[bp & (DSP_RAMSIZE - 1)] : 0);
			printf("  handler p:%04x %06x %u\n", bp, w, best);
			dsp_jit_khist[bp] = 0;
		}
	}
	if (jit_on() && dsp_jit_ihist) {
		/* JITHIST=1: the most interpreted PCs */
		for (int t = 0; t < 12; t++) {
			uint32_t best = 0, bp = 0;
			for (uint32_t p = 0; p < 65536; p++) if (dsp_jit_ihist[p] > best) { best = dsp_jit_ihist[p]; bp = p; }
			if (!best) break;
			uint32_t w = bp < 0x200 ? dsp_core.ramint[DSP_SPACE_P][bp] : (dsp_core.ramext ? dsp_core.ramext[bp & (DSP_RAMSIZE - 1)] : 0);
			printf("  interpreted p:%04x %06x %u\n", bp, w, best);
			dsp_jit_ihist[bp] = 0;
		}
	}
	if (jit_on()) printf("jit: %u blocks, %u instructions in blocks, %u interpreted, %u compiles (other start %u, changed %u, LA inside %u, clip %u)\n", dsp_jit_stats[0], dsp_jit_stats[1], dsp_jit_stats[2], dsp_jit_stats[3], dsp_jit_stats[4], dsp_jit_stats[5], dsp_jit_stats[6], dsp_jit_stats[7]);
	printf("replay %s from %04x: %ld instr in %.3f s CPU = %5.2f MIPS  pc %04x\n", path, pc >= 0 ? pc : h.pc, n, t1 - t0,
	       n / (t1 - t0) / 1e6, dsp_core.pc);
	return 0;
}

int main(int argc, char **argv)
{
	if (argc > 2 && !strcmp(argv[1], "replay"))
		return replay(argv[2], argc > 3 ? atol(argv[3]) : 20000000, argc > 4 ? (int)strtol(argv[4], 0, 16) : -1);
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
		const char *pf = getenv("DSPPROF");
		if (pf) next_dsp_prof(1);
		t0 = cpu_s();
		while (done < n) { int s = next_dsp_sim_step(256); if (!s) { printf("stalled\n"); break; } done += s; }
		t1 = cpu_s();
		if (pf) printf("profile %s: %s\n", pf, next_dsp_prof_dump(pf) ? "failed" : "written");
	} else {
		dsp_core_init(hi);
		dsp_core_reset();
		dsp_core_start(1, 1);
		if (fir) load_fir();
		for (int i = 0; i < 1000; i++) dsp56k_execute_instruction();
		perf_start();
		t0 = cpu_s();
		if (jit_on()) { uint32_t c = 0; while (done < n) done += dsp56k_jit_step(&c); }
		else for (; done < n; done++) dsp56k_execute_instruction();
		t1 = cpu_s();
		perf_stop(n);
	}
	printf("%-4s link %d: %ld instr in %.3f s CPU = %5.2f MIPS  pc %04x a %06x\n", k, link, done, t1 - t0,
	       done / (t1 - t0) / 1e6, dsp_core.pc, dsp_core.registers[DSP_REG_A1]);
	return 0;
}
