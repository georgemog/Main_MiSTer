/* lockstep: two builds of the DSP core in one process, compared after every
   instruction (JIT plan, phase 1/4).  Built by lockstep.sh: each core's
   objects are linked into one relocatable and its globals renamed ref_* /
   new_*.  Modes:
     lockstep fuzz SEED ROUNDS      random programs as in fuzz.c, with
                                    interrupts sometimes unmasked
     lockstep replay DUMP N [pc]    a next_dsp profile dump, N instructions
   Compared every instruction: registers, pc, instr_cycle, interrupt and
   loop state, AGU pipeline, stack; memories every 64 and at the end. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "dsp_core.h"
#include "dsp_cpu.h"

#define CORE(p) \
	extern dsp_core_t p##dsp_core; \
	void p##dsp_core_init(void (*)(int)); void p##dsp_core_reset(void); \
	void p##dsp_core_config_ramext(uint32_t *, uint32_t); void p##dsp56k_execute_instruction(void); int p##dsp56k_jit_step(uint32_t *); void p##dsp56k_jit_flush(void);
CORE(ref_)
CORE(new_)
extern uint32_t new_dsp_jit_dbg[68];	/* NEXT_DSP_JIT_DBG=1: the new side's cycles per instruction of the last block */
extern uint32_t new_dsp_jit_why[12];	/* why steps went to the interpreter; [11] blocks kept after a code page write */

static void hi(int x) { (void)x; }
static uint64_t s;
static uint32_t rnd(void) { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (uint32_t)s; }
static uint32_t ram_r[32768], ram_n[32768];
static long steps, fails;

static void cmp_mem(void)
{
	if (memcmp(ref_dsp_core.ramint, new_dsp_core.ramint, sizeof ref_dsp_core.ramint) ||
	    memcmp(ref_dsp_core.periph, new_dsp_core.periph, sizeof ref_dsp_core.periph) ||
	    memcmp(ram_r, ram_n, sizeof ram_r)) {
		printf("MEMORY MISMATCH at step %ld pc %04x\n", steps, ref_dsp_core.pc);
		fails++;
	}
}

static int cmp_state(uint32_t pc0, uint32_t inst)
{
	dsp_core_t *a = &ref_dsp_core, *b = &new_dsp_core;
	const char *what = 0;
	if (memcmp(a->registers, b->registers, sizeof a->registers)) what = "registers";
	else if (a->pc != b->pc) what = "pc";
	else if (a->instr_cycle != b->instr_cycle) what = "instr_cycle";
	else if (a->interrupt_status != b->interrupt_status || a->interrupt_state != b->interrupt_state ||
	         a->interrupt_pipeline_count != b->interrupt_pipeline_count || a->interrupt_save_pc != b->interrupt_save_pc)
		what = "interrupt state";
	else if (a->loop_rep != b->loop_rep || a->pc_on_rep != b->pc_on_rep) what = "rep state";
	else if (memcmp(a->agu_pipeline_reg, b->agu_pipeline_reg, sizeof a->agu_pipeline_reg) ||
	         memcmp(a->agu_pipeline_val, b->agu_pipeline_val, sizeof a->agu_pipeline_val)) what = "agu pipeline";
	else if (memcmp(a->stack, b->stack, sizeof a->stack)) what = "stack";
	else if (memcmp(a->hostport, b->hostport, sizeof a->hostport)) what = "hostport";
	if (!what) return 0;
	printf("MISMATCH (%s) at step %ld: pc %04x inst %06x\n", what, steps, pc0, inst);
	for (int i = 0; i < 64; i++)
		if (a->registers[i] != b->registers[i]) printf("  reg %2d ref %06x new %06x\n", i, a->registers[i], b->registers[i]);
	printf("  pc ref %04x new %04x  cycles ref %d new %d\n", a->pc, b->pc, a->instr_cycle, b->instr_cycle);
	fails++;
	return 1;
}

static int use_jit;	/* LOCKSTEP_JIT=1: new core runs dsp56k_jit_step (blocks) */
static long blocks, in_blocks;

static uint32_t lp[64], lw[64], lc[64];
static int lk;

static int step(void)
{
	uint32_t pc0 = ref_dsp_core.pc;
	uint32_t inst = pc0 < 0x200 ? ref_dsp_core.ramint[2][pc0] : ram_r[pc0 & 32767];
	if (use_jit) {
		uint32_t cn = 0, cr = 0;
		int k = new_dsp56k_jit_step(&cn);
		lk = k;
		for (int i = 0; i < k; i++) {
			uint32_t p = ref_dsp_core.pc;
			if (i < 64) { lp[i] = p; lw[i] = p < 0x200 ? ref_dsp_core.ramint[2][p] : ram_r[p & 32767]; }
			ref_dsp56k_execute_instruction(); cr += ref_dsp_core.instr_cycle;
			if (i < 64) lc[i] = ref_dsp_core.instr_cycle;
		}
		if (cn != cr) for (int i = 0; i < k && i < 64; i++)
			printf("  ref %04x %06x %u cycles  (new %u, total %u)\n", lp[i], lw[i], lc[i], i < 32 ? new_dsp_jit_dbg[i] : 0, i < 32 ? new_dsp_jit_dbg[32 + i] : 0);
		steps += k;
		if (k > 1) { blocks++; in_blocks += k; }
		if (cn != cr) printf("  new block %04x: cycles %u n %u pc %04x len %u\n", new_dsp_jit_dbg[67], new_dsp_jit_dbg[64], new_dsp_jit_dbg[65], new_dsp_jit_dbg[66] & 0xffff, new_dsp_jit_dbg[66] >> 16);
		if (cn != cr) { printf("CYCLES MISMATCH at step %ld (block of %d from %04x): ref %u new %u\n", steps, k, pc0, cr, cn); fails++; return 1; }
	} else {
		ref_dsp56k_execute_instruction();
		new_dsp56k_execute_instruction();
		steps++;
	}
	if (cmp_state(pc0, inst)) {
		if (use_jit) for (int i = 0; i < lk && i < 64; i++) printf("  ref %04x %06x %u cycles\n", lp[i], lw[i], lc[i]);
		return 1;
	}
	if (!(steps & 63)) cmp_mem();
	return fails > 0;
}

static int fuzz(long rounds)
{
	ref_dsp_core_init(hi); new_dsp_core_init(hi);
	for (long r = 0; r < rounds; r++) {
		ref_dsp_core_reset(); new_dsp_core_reset();
		ref_dsp_core.mode = 0;
		ref_dsp_core_config_ramext(ram_r, 8192); new_dsp_core_config_ramext(ram_n, 8192);
		for (int i = 0; i < 512; i++) {
			ref_dsp_core.ramint[0][i] = rnd() & 0xffffff; ref_dsp_core.ramint[1][i] = rnd() & 0xffffff;
			uint32_t op = rnd() & 0xffffff; if (rnd() % 4) op |= 0x100000; ref_dsp_core.ramint[2][i] = op;
		}
		for (int i = 0; i < 8192; i++) ram_r[i] = rnd() & 0xffffff;
		for (int i = 0; i < 8; i++) {
			ref_dsp_core.registers[DSP_REG_R0+i] = rnd() & 0x1ff; ref_dsp_core.registers[DSP_REG_N0+i] = rnd() % 5;
			uint32_t m = rnd() % 4;
			ref_dsp_core.registers[DSP_REG_M0+i] = m == 0 ? 0xffff : m == 1 ? (rnd() % 40) + 1 : m == 2 ? 0 : (rnd() & 0x7fff);
		}
		ref_dsp_core.registers[DSP_REG_X0] = rnd() & 0xffffff; ref_dsp_core.registers[DSP_REG_X1] = rnd() & 0xffffff;
		ref_dsp_core.registers[DSP_REG_Y0] = rnd() & 0xffffff; ref_dsp_core.registers[DSP_REG_Y1] = rnd() & 0xffffff;
		ref_dsp_core.registers[DSP_REG_A0] = rnd() & 0xffffff; ref_dsp_core.registers[DSP_REG_A1] = rnd() & 0xffffff; ref_dsp_core.registers[DSP_REG_A2] = rnd() & 0xff;
		ref_dsp_core.registers[DSP_REG_B0] = rnd() & 0xffffff; ref_dsp_core.registers[DSP_REG_B1] = rnd() & 0xffffff; ref_dsp_core.registers[DSP_REG_B2] = rnd() & 0xff;
		/* random scaling; interrupts masked 3 rounds in 4, else open with
		   the host and SSI interrupt sources pending as they come */
		ref_dsp_core.registers[DSP_REG_SR] = (rnd() & 0x0c00) | ((r & 3) ? 0x0300 : 0);
		ref_dsp_core.pc = rnd() & 0x1ff;
		if (!(r & 3)) {
			ref_dsp_core.periph[0][0x3f] = rnd() & 0xffff;	/* IPR */
			ref_dsp_core.interrupt_status = rnd() & 0x7fff0;
			ref_dsp_core.interrupt_enable = rnd(); ref_dsp_core.interrupt_mask = rnd();
			ref_dsp_core.interrupt_mask_level[0] = rnd(); ref_dsp_core.interrupt_mask_level[1] = rnd(); ref_dsp_core.interrupt_mask_level[2] = rnd();
		}
		/* identical start */
		memcpy(ram_n, ram_r, sizeof ram_r);
		uint32_t *keep = new_dsp_core.ramext;
		memcpy(&new_dsp_core, &ref_dsp_core, sizeof new_dsp_core);
		new_dsp_core.ramext = keep;
		new_dsp56k_jit_flush();
		for (int k = 0; k < 400; k++) {
			if (step()) return 1;
			if (ref_dsp_core.pc >= 0x200) { ref_dsp_core.pc &= 0x1ff; new_dsp_core.pc &= 0x1ff; }
		}
		cmp_mem();
		if (fails) return 1;
		if ((r + 1) % (rounds / 10 ? rounds / 10 : 1) == 0) printf("round %ld: %ld instructions, no mismatch\n", r + 1, steps);
	}
	if (getenv("LOCKSTEP_JIT")) printf("jit: %u blocks kept after a code page write\n", new_dsp_jit_why[11]);
	return 0;
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
	struct prof_hdr h;
	if (fread(&h, sizeof h, 1, f) != 1 || memcmp(h.magic, "NXDSPPR2", 8)) { printf("not a dump\n"); return 1; }
	ref_dsp_core_init(hi); new_dsp_core_init(hi);
	ref_dsp_core_reset(); new_dsp_core_reset();
	size_t ok = fread(ref_dsp_core.registers, sizeof ref_dsp_core.registers, 1, f);
	ok += fread(ref_dsp_core.stack, sizeof ref_dsp_core.stack, 1, f);
	ok += fread(ref_dsp_core.rom, sizeof ref_dsp_core.rom, 1, f);
	ok += fread(ref_dsp_core.ramint, sizeof ref_dsp_core.ramint, 1, f);
	ok += fread(ref_dsp_core.periph, sizeof ref_dsp_core.periph, 1, f);
	if (h.ramext_words) {
		ref_dsp_core_config_ramext(ram_r, h.ramext_words); new_dsp_core_config_ramext(ram_n, h.ramext_words);
		ok += fread(ram_r, 4 * h.ramext_words, 1, f);
	}
	fclose(f);
	(void)ok;
	ref_dsp_core.mode = h.mode;
	ref_dsp_core.pc = pc >= 0 ? pc : h.pc;
	ref_dsp_core.running = 1;
	memcpy(ram_n, ram_r, sizeof ram_r);
	uint32_t *keep = new_dsp_core.ramext;
	memcpy(&new_dsp_core, &ref_dsp_core, sizeof new_dsp_core);
	new_dsp_core.ramext = keep;
	new_dsp56k_jit_flush();
	for (long i = 0; i < n; i++) if (step()) return 1;
	cmp_mem();
	if (use_jit) printf("jit: %ld blocks of more than 1, %ld instructions in them\n", blocks, in_blocks);
	if (use_jit) printf("jit: %u blocks kept after a code page write\n", new_dsp_jit_why[11]);
	printf("replay %s from %04x: %ld instructions, %s (pc %04x)\n", path, pc >= 0 ? pc : h.pc, steps, fails ? "MISMATCH" : "no mismatch", ref_dsp_core.pc);
	return fails > 0;
}

int main(int argc, char **argv)
{
	use_jit = getenv("LOCKSTEP_JIT") != NULL;
	if (argc > 3 && !strcmp(argv[1], "fuzz")) { s = strtoull(argv[2], 0, 0) | 1; return fuzz(atol(argv[3])); }
	if (argc > 3 && !strcmp(argv[1], "replay")) return replay(argv[2], atol(argv[3]), argc > 4 ? (int)strtol(argv[4], 0, 16) : -1);
	printf("usage: lockstep fuzz SEED ROUNDS | replay DUMP N [pc]\n");
	return 2;
}
