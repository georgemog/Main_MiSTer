/*
 * dspprof DUMP [ntop] [CODEDUMP...]: read a next_dsp profile dump (/media/usb0/next_dsp/LABEL.bin)
 * and print where the DSP's time went, for the JIT plan (phase 0).
 *
 *   cc -O2 -w -I.. -o dspprof dspprof.c hooks.c ../dsp_core.c ../dsp_cpu.c ../dsp_disasm.c
 *
 * Sections: totals and real-time margin; interrupts; instruction mix by
 * mnemonic (non-parallel), by ALU op x parallel-move kind; the hottest PCs
 * with their disassembly (from the P memory in the dump, which is a snapshot
 * at dump time: code loaded later than the counted run may differ); the AGU
 * registers; and the share of executed instructions that touch peripherals.
 * CODEDUMP: take the memories and registers for the listing from another
 * dump (one written while the workload ran; the final dump often shows the
 * bootstrap ROM again, after the program's DSP reset).  Percentages in the
 * mix and the hot list are of the instructions outside the bootstrap ROM.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "dsp_core.h"
#include "dsp_cpu.h"
#include "dsp_disasm.h"

struct prof_hdr {
	char     magic[8];
	uint32_t version, hdr_size;
	uint64_t instrs, cycles, jumps, same_pc, wall_ns, sleep_ns, behind;
	uint32_t ramext_words, mode;
	uint16_t pc, pad;
	uint32_t interrupt_state, loop_rep;
	uint64_t boot_instrs;
};

static uint32_t pc_h[65536], np_h[512], par_h[16 * 256], irq_h[64];
static uint32_t *ramext;
static char *ptxt[65536];   /* text of each PC that ran */
#define TXT(pc) (ptxt[pc] ? ptxt[pc] : "?")

static void hi(int x) { (void)x; }

static void rd(FILE *f, void *p, size_t n)
{
	if (fread(p, 1, n, f) != n) { fprintf(stderr, "short dump\n"); exit(1); }
}

/* the instruction text at pc, whitespace squeezed */
static const char *dis(uint16_t pc, char *out, size_t n)
{
	dsp_core.pc = pc;
	dsp56k_disasm(DSP_DISASM_MODE, NULL);
	const char *t = dsp56k_getInstructionText();
	const char *q = strstr(t, "cyc)");
	q = q ? q + 4 : t;
	size_t j = 0; int sp = 0;
	for (; *q && j + 2 < n; q++) {
		if (*q == ' ' || *q == '\t' || *q == '\n') { sp = 1; continue; }
		if (sp && j) out[j++] = ' ';
		sp = 0; out[j++] = *q;
	}
	out[j] = 0;
	return out;
}

/* the text of a representative opcode (not from the dump's memory) */
static const char *dis_op(uint32_t op, char *out, size_t n)
{
	uint32_t save0 = dsp_core.ramint[DSP_SPACE_P][0x1f0], save1 = dsp_core.ramint[DSP_SPACE_P][0x1f1];
	uint32_t mode = dsp_core.mode;
	dsp_core.mode = 0;
	dsp_core.ramint[DSP_SPACE_P][0x1f0] = op;
	dsp_core.ramint[DSP_SPACE_P][0x1f1] = 0;
	dis(0x1f0, out, n);
	dsp_core.ramint[DSP_SPACE_P][0x1f0] = save0;
	dsp_core.ramint[DSP_SPACE_P][0x1f1] = save1;
	dsp_core.mode = mode;
	return out;
}

struct row { char name[64]; uint64_t n; };
static int byn(const void *a, const void *b)
{
	uint64_t x = ((const struct row *)a)->n, y = ((const struct row *)b)->n;
	return x < y ? 1 : x > y ? -1 : 0;
}
static void add(struct row *r, int *nr, const char *name, uint64_t n)
{
	for (int i = 0; i < *nr; i++) if (!strcmp(r[i].name, name)) { r[i].n += n; return; }
	snprintf(r[*nr].name, sizeof(r[*nr].name), "%s", name);
	r[(*nr)++].n = n;
}

static void load_state(FILE *f, const struct prof_hdr *h)
{
	rd(f, dsp_core.registers, sizeof(dsp_core.registers));
	rd(f, dsp_core.stack, sizeof(dsp_core.stack));
	rd(f, dsp_core.rom, sizeof(dsp_core.rom));
	rd(f, dsp_core.ramint, sizeof(dsp_core.ramint));
	rd(f, dsp_core.periph, sizeof(dsp_core.periph));
	if (h->ramext_words) {
		free(ramext);
		ramext = calloc(h->ramext_words, 4);
		dsp_core_config_ramext(ramext, h->ramext_words);   /* clears it, so before the read */
		rd(f, ramext, 4 * h->ramext_words);
	}
}

static FILE *open_dump(const char *path, struct prof_hdr *h)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); exit(1); }
	rd(f, h, sizeof(*h));
	if (memcmp(h->magic, "NXDSPPR2", 8) || h->hdr_size != sizeof(*h)) { fprintf(stderr, "%s: not a v2 dump\n", path); exit(1); }
	return f;
}

int main(int argc, char **argv)
{
	if (argc < 2) { fprintf(stderr, "usage: dspprof DUMP [ntop]\n"); return 2; }
	int ntop = argc > 2 ? atoi(argv[2]) : 40;
	struct prof_hdr h, hc;
	dsp_core_init(hi);
	dsp_core_reset();
	FILE *f = open_dump(argv[1], &h);
	load_state(f, &h);
	rd(f, pc_h, sizeof(pc_h));
	rd(f, np_h, sizeof(np_h));
	rd(f, par_h, sizeof(par_h));
	rd(f, irq_h, sizeof(irq_h));
	fclose(f);
	/* the text of each PC that ran, from the code dumps (argv[3..], in
	 * order; else this dump): the first dump where it is not a nop wins,
	 * since Music Kit loads and drops unit-generator code as it plays */
	dsp56k_disasm_init();
	int ncode = argc > 3 ? argc - 3 : 1;
	for (int d = 0; d < ncode; d++) {
		f = open_dump(argc > 3 ? argv[3 + d] : argv[1], &hc);
		load_state(f, &hc);
		fclose(f);
		dsp_core.mode = hc.mode == 1 ? 0 : hc.mode;   /* list the program, not the ROM */
		char t0[256];
		for (int pc = 0; pc < 65536; pc++) if (pc_h[pc] && (!ptxt[pc] || !strcmp(ptxt[pc], "nop"))) {
			dis((uint16_t)pc, t0, sizeof(t0));
			if (!ptxt[pc] || strcmp(t0, "nop")) { free(ptxt[pc]); ptxt[pc] = strdup(t0); }
		}
	}
	f = open_dump(argv[1], &hc);   /* back to this dump's registers */
	load_state(f, &hc);
	fclose(f);

	double wall = h.wall_ns / 1e9, n = (double)(h.instrs > h.boot_instrs ? h.instrs - h.boot_instrs : 1);
	printf("# %s\n\n", argv[1]);
	printf("instructions %llu, cycles %llu (%.2f per instruction, bootstrap included), wall %.2f s\n",
	       (unsigned long long)h.instrs, (unsigned long long)h.cycles, h.cycles / (double)(h.instrs ? h.instrs : 1), wall);
	printf("rate %.2f M instr/s; a 25 MHz 56001 would take %.2f s for these cycles (%.0f%% of wall)\n",
	       h.instrs / wall / 1e6, h.cycles / 25e6, 100 * h.cycles / 25e6 / (wall > 0 ? wall : 1));
	printf("bootstrap ROM (idle or loading): %llu instructions (%.1f%%); the rest below is the program\n",
	       (unsigned long long)h.boot_instrs, 100.0 * h.boot_instrs / (h.instrs ? h.instrs : 1));
	printf("pacing: slept %.2f s (%.0f%% of wall), fell behind %llu times\n",
	       h.sleep_ns / 1e9, 100 * h.sleep_ns / 1e9 / (wall > 0 ? wall : 1), (unsigned long long)h.behind);
	printf("control transfers %llu (one per %.1f instructions), PC unchanged %llu (REP, loops on self)\n",
	       (unsigned long long)h.jumps, n / (h.jumps + 1), (unsigned long long)h.same_pc);
	printf("at dump: mode %u, pc %04x, interrupt_state %u, rep %u, SR %06x, LA %04x LC %04x SP %02x\n\n",
	       h.mode, h.pc, h.interrupt_state, h.loop_rep, dsp_core.registers[DSP_REG_SR],
	       dsp_core.registers[DSP_REG_LA], dsp_core.registers[DSP_REG_LC], dsp_core.registers[DSP_REG_SP]);

	printf("## interrupts taken (vector address)\n\n");
	for (int i = 0; i < 64; i++) if (irq_h[i]) printf("  p:$%02x  %u\n", i * 2, irq_h[i]);
	printf("\n");

	char t[256], name[64];
	static struct row mn[1024], alu[1024], mv[32];
	int nmn = 0, nalu = 0, nmv = 0;
	static const char *pmk[16] = { "pm0 (A/B swap)", "pm1 (R: X:/Y: + reg)", "pm2 (none / reg / ea)", "pm3 (#imm)",
		"pm4 (L:)", "pm5 (X: or Y:)", "pm5 (X: or Y:)", "pm5 (X: or Y:)",
		"pm8 (XY)", "pm8 (XY)", "pm8 (XY)", "pm8 (XY)", "pm8 (XY)", "pm8 (XY)", "pm8 (XY)", "pm8 (XY)" };
	uint64_t npar = 0, nnp = 0;
	for (int i = 0; i < 512; i++) if (np_h[i]) {
		uint32_t op = ((uint32_t)(i >> 3) << 14) | ((uint32_t)(i & 7) << 5);
		sscanf(dis_op(op, t, sizeof(t)), "%63s", name);
		add(mn, &nmn, name, np_h[i]);
		nnp += np_h[i];
	}
	for (int i = 0; i < 16 * 256; i++) if (par_h[i]) {
		uint32_t op = ((uint32_t)(i >> 8) << 20) | (i & 0xff);
		sscanf(dis_op(op, t, sizeof(t)), "%63s", name);
		char k[64];
		snprintf(k, sizeof(k), "%s", name);
		add(alu, &nalu, k, par_h[i]);
		add(mv, &nmv, pmk[i >> 8], par_h[i]);
		npar += par_h[i];
	}
	qsort(mn, nmn, sizeof(mn[0]), byn);
	qsort(alu, nalu, sizeof(alu[0]), byn);
	qsort(mv, nmv, sizeof(mv[0]), byn);
	printf("## instruction mix\n\nparallel-move form %.1f%%, other %.1f%%\n\n", 100 * npar / n, 100 * nnp / n);
	printf("other instructions (by mnemonic):\n");
	for (int i = 0; i < nmn && i < 30; i++) printf("  %-10s %6.2f%%  %llu\n", mn[i].name, 100 * mn[i].n / n, (unsigned long long)mn[i].n);
	printf("\nparallel-move form, ALU part:\n");
	for (int i = 0; i < nalu && i < 30; i++) printf("  %-10s %6.2f%%  %llu\n", alu[i].name, 100 * alu[i].n / n, (unsigned long long)alu[i].n);
	printf("\nparallel-move form, move kind:\n");
	for (int i = 0; i < nmv; i++) printf("  %-24s %6.2f%%\n", mv[i].name, 100 * mv[i].n / n);

	/* hottest PCs, and the peripheral share, from the P snapshot */
	static uint32_t idx[65536];
	int ni = 0;
	uint64_t periph = 0, counted = 0;
	for (int pc = 0; pc < 65536; pc++) if (pc_h[pc]) {
		idx[ni++] = pc;
		counted += pc_h[pc];
		snprintf(t, sizeof(t), "%s", TXT(pc));
		if (strstr(t, ":$ff") || strstr(t, "x:$ffe") || strstr(t, "movep")) periph += pc_h[pc];
	}
	for (int i = 0; i < ni; i++) for (int j = i + 1; j < ni; j++)
		if (pc_h[idx[j]] > pc_h[idx[i]]) { uint32_t x = idx[i]; idx[i] = idx[j]; idx[j] = x; }
	/* the mix again, by disassembling each PC that ran: exact where the
	 * code in the listing dump is the code that ran (the opcode buckets
	 * above are named from one opcode each, which can mislabel) */
	{
		static struct row pm[1024];
		int npm = 0;
		for (int i = 0; i < ni; i++) {
			snprintf(t, sizeof(t), "%s", TXT(idx[i]));
			sscanf(t, "%63s", name);
			add(pm, &npm, name, pc_h[idx[i]]);
		}
		qsort(pm, npm, sizeof(pm[0]), byn);
		printf("\n## mix by PC (first word of each PC that ran, disassembled from the code dumps)\n\n");
		printf("PC histogram total %.1f%% of the program's instructions\n", 100 * counted / n);
		for (int i = 0; i < npm && i < 30; i++) printf("  %-10s %6.2f%%  %u\n", pm[i].name, 100 * pm[i].n / n, (unsigned)pm[i].n);
	}
	printf("\n## hottest PCs (%d PCs ran; listing from %d code dump%s)\n\n", ni, ncode, ncode > 1 ? "s" : "");
	printf("instructions naming a peripheral ($ffc0-$ffff, movep): %.1f%%\n\n", 100.0 * periph / (counted ? counted : 1));
	int m = ni < ntop ? ni : ntop;
	/* print in address order */
	for (int i = 0; i < m; i++) for (int j = i + 1; j < m; j++)
		if (idx[j] < idx[i]) { uint32_t x = idx[i]; idx[i] = idx[j]; idx[j] = x; }
	for (int i = 0; i < m; i++)
		printf("  p:%04x %6.2f%%  %s\n", idx[i], 100.0 * pc_h[idx[i]] / n, TXT(idx[i]));

	printf("\n## AGU at dump time\n\n");
	for (int r = 0; r < 8; r++) {
		uint32_t mr = dsp_core.registers[DSP_REG_M0 + r];
		printf("  r%d %04x  n%d %04x  m%d %04x  (%s)\n", r, dsp_core.registers[DSP_REG_R0 + r], r,
		       dsp_core.registers[DSP_REG_N0 + r], r, mr,
		       mr == 0xffff ? "linear" : mr == 0 ? "bit-reverse" : mr < 0x8000 ? "modulo" : "reserved");
	}
	return 0;
}
