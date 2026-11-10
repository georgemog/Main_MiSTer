/*
 * NeXT-Color soft DSP: ARMv7 (A32) block compiler for the DSP56001
 * (JIT plan, phase 2).  #included at the end of dsp_cpu.c (named .inc so the Makefile wildcard skips it), so it uses the
 * interpreter's statics (opcodes_alu, dsp_update_rn, read_memory, ...).
 *
 * A block is a straight run of instructions that the compiler handles
 * natively, started at a PC and ended at the first instruction it does not
 * handle, at LA when a DO loop is active (so the loop end is the block's
 * last instruction), or after JIT_MAX instructions.  The runner,
 * dsp56k_jit_step(), executes one block, or one instruction through the
 * interpreter where no block applies, and then the interpreter's own
 * end-of-instruction work (PC update with REP/DO, interrupts) for the
 * block's last instruction.  Blocks are only entered when that is the
 * same as stepping: no REP, no interrupt pipeline in flight, no pending
 * AGU delay, no trace, no OMR change pending, not in the bootstrap ROM.
 *
 * Native (version 1): the parallel-move forms (none, register, X: or Y:
 * with a post-update/no-update address or a short absolute address, XY),
 * whose moves only name X0..Y1, A, B (and R/N as move sources).  The ALU
 * part calls Hatari's own opcodes_alu function, with its side effects in
 * the same order as dsp_pm_*.  Memory: internal X/Y RAM and external RAM
 * inline; anything else (peripherals, internal ROM space) through
 * read_memory/write_memory, and a peripheral access ends the block after
 * that instruction, so the interrupt and host-link checks see it at once.
 * AGU: linear (M = $ffff) inline, other modes call dsp_update_rn.
 * Cycles: 2 per instruction plus Hatari's external-memory wait states,
 * computed per instruction.
 *
 * Native ALU (step 2): MPY/MPYR/MAC/MACR, ADD/SUB/CMP (A/B, X, Y, X0..Y1),
 * TFR, TST, CLR, RND with exact CCR, in scaling mode 0 (a block keeps the
 * mode it was compiled for).  NEXT_DSP_JIT_ALU=0 calls Hatari's functions
 * for every ALU op instead.
 *
 * Step 3: parallel moves to R/N (register, memory, #xx, R update) are
 * native, with the one-instruction AGU delay resolved at compile time
 * (jit_prev_agu); nop and Tcc S1,D1 are native; and any other instruction
 * except a lone one can end a block as its tail, through Hatari's handler
 * (jit_tail), so jumps, bit tests, movep, do and so on no longer need the
 * interpreter's full dispatch.
 *
 * Step 4: DO #n / DO S runs inside the block (Hatari's handler) and its
 * body follows inline; at LA the block branches back while LC > 1 (with a
 * budget) and pops the loop (jit_loopend) at LC = 1, then goes on.  With
 * no loop active, JMP (short and absolute) continues the block at its
 * target, a JMP or Jcc to the block's start loops in place, and a taken
 * Jcc exits.  Such blocks keep up to JIT_PV code pages and JIT_NR
 * external code ranges, and are only entered with LF clear.
 *
 * Step 5: a block is compiled for the M registers in force at entry
 * (recorded, checked at entry): linear updates lose their runtime check,
 * and modulo +-1 is inline with its bounds folded.  A or B as a move
 * source is read inline (limiting on the slow path).  Instructions
 * without memory accesses skip the access bits, wait states and exit
 * test.  An ALU op whose flags the next ALU op overwrites defers them
 * (S_FLO/S_FHI/S_FPSR), rebuilt only if the block exits right after it.
 *
 * Code changes: a block keeps the P words it was compiled from and checks
 * them on entry.  Changing the external RAM (pointer or size) drops all.
 */

/* generated code is A32; C may be Thumb-2 (blx/ldm pc interwork) */
#if defined(__arm__)
#define DSP_JIT_ARM 1
#else
#define DSP_JIT_ARM 0
#endif

#ifndef NEXT_DSP_JIT
#define NEXT_DSP_JIT 1
#endif

#if DSP_JIT_ARM && NEXT_DSP_JIT
#include <stdlib.h>
#include <sys/mman.h>
#include <stddef.h>

#define JIT_MAX		32		/* instructions per block */
#define JIT_CODE_SIZE	(2 << 20)
#define JIT_HASH	65536		/* one slot per P address */
#define JIT_PV		8		/* code pages per block */
#define JIT_NR		3		/* external code ranges per block */

typedef struct jit_block_s {
	uint16_t start, last;		/* first and last instruction address */
	uint16_t n;			/* instructions; 0: the first one ends blocks */
	uint16_t clip;			/* compiled to stop at this LA (0xffff: none) */
	uint16_t nw;			/* P words */
	uint16_t scal;			/* SR scaling bits it was compiled for */
	uint8_t npv;			/* code pages */
	uint8_t jumps;			/* follows jumps: only entered with no loop active */
	uint8_t mmask;			/* M registers its address updates were compiled for */
	uint16_t m[8];
	const uint32_t *pv[JIT_PV];	/* code version counters of its pages */
	uint32_t ver[JIT_PV];
	void (*code)(void);
	const uint32_t *src[2 * JIT_MAX];	/* where each P word came from */
	uint32_t word[2 * JIT_MAX];
} jit_block_t;

static struct { uint32_t n, pc, len, cycles; } jit_out;
static uint8_t *jit_mem, *jit_ptr;
static jit_block_t *jit_tab[JIT_HASH];
static uint8_t jit_cc[16][128];		/* condition code x SR bits 0..6 (dsp_calc_cc) */

static int jit_cc_eval(int cc, uint32_t sr)
{
	int c = sr & 1, v = (sr >> 1) & 1, z = (sr >> 2) & 1, n = (sr >> 3) & 1;
	int u = (sr >> 4) & 1, e = (sr >> 5) & 1, l = (sr >> 6) & 1, r = 0;
	switch (cc & 7) {
	case 0: r = !c; break;
	case 1: r = !(n ^ v); break;
	case 2: r = !z; break;
	case 3: r = !n; break;
	case 4: r = !(z | (!u & !e)); break;
	case 5: r = !e; break;
	case 6: r = !l; break;
	case 7: r = !(z | (n ^ v)); break;
	}
	return cc & 8 ? !r : r;
}
static jit_block_t *jit_pool;
static int jit_pool_n;
#define JIT_POOL	8192
static uint32_t *jit_ramext;
static uint32_t jit_ramsize;
static int jit_off;			/* NEXT_DSP_JIT=0 in the environment */
static int jit_saw_boot;
static int jit_max = JIT_MAX;		/* NEXT_DSP_JIT_MAX=n: shorter blocks (debugging) */
static const uint8_t jit_wait[8] = { 0, 0, 0, 2, 0, 2, 2, 4 };	/* X=1 Y=2 P=4 */
uint32_t dsp_jit_stats[8];		/* blocks run, instructions in blocks, interpreted, compiles;
					   recompiles: other start, code changed, LA inside, clip */

#define JIT_IDX(pc)	(pc)

/*--------------------------------------------------------------------
 * helpers called from generated code
 *------------------------------------------------------------------*/

#define JF_PERIPH	0x100
#define JF_SMC		0x200	/* wrote this block's own code: end after this instruction */
#define JF_EXIT		(JF_PERIPH | JF_SMC)

/* returns value | flags << 32 (access bits X=1 Y=2 P=4, JF_PERIPH) */
static uint64_t jit_rd(uint32_t space, uint32_t address)
{
	uint32_t v;
	access_to_ext_memory = 0;
	v = read_memory(space, address);
	return v | ((uint64_t)(access_to_ext_memory | (address >= 0xffc0 ? JF_PERIPH : 0)) << 32);
}

static uint32_t jit_wr(uint32_t space, uint32_t address, uint32_t value)
{
	access_to_ext_memory = 0;
	write_memory(space, address, value);
	/* (an external write here may be to code: end the block, rare path) */
	return access_to_ext_memory | (address >= 0xffc0 ? JF_PERIPH : 0) | (access_to_ext_memory ? JF_SMC : 0);
}

static uint32_t jit_accu(uint32_t numreg)
{
	uint32_t v;
	dsp_pm_read_accu24(numreg, &v);
	return v;
}

static void jit_urn(uint32_t numreg, uint32_t reg_value, int32_t mod, uint32_t m)
{
	dsp_update_rn(numreg, reg_value, (int16_t)mod, (uint16_t)m);
}

/*--------------------------------------------------------------------
 * A32 emitter
 *------------------------------------------------------------------*/

enum { R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, IP, SP, LR, PC };
enum { EQ = 0, NE, HS, LO, MI, PL, VS, VC, HI, LS, GE, LT, GT, LE, AL };
enum { OP_AND = 0, OP_EOR, OP_SUB, OP_RSB, OP_ADD, OP_ADC, OP_SBC, OP_RSC,
       OP_TST, OP_TEQ, OP_CMP, OP_CMN, OP_ORR, OP_MOV, OP_BIC, OP_MVN };

static uint32_t *ep;			/* emit pointer */
static uint32_t jit_rlo[JIT_NR], jit_rhi[JIT_NR];	/* external RAM index ranges of the block's code */
static int jit_nrange;
static uint32_t *ep_end;
static int ep_overflow;

static void emit(uint32_t w)
{
	if (ep >= ep_end) { ep_overflow = 1; return; }
	*ep++ = w;
}

/* rotated 8-bit immediate, or -1 */
static int imm12(uint32_t v)
{
	for (int r = 0; r < 16; r++) {
		uint32_t x = (v << (2 * r)) | (r ? v >> (32 - 2 * r) : 0);
		if (x < 256) return (r << 8) | x;
	}
	return -1;
}

static void dpi(int cond, int op, int s, int rd, int rn, uint32_t imm)
{
	int e = imm12(imm);
	emit((uint32_t)cond << 28 | 1 << 25 | op << 21 | s << 20 | rn << 16 | rd << 12 | (uint32_t)e);
}

/* register operand, shifted by an immediate (type 0 LSL, 1 LSR, 2 ASR) */
static void dpr(int cond, int op, int s, int rd, int rn, int rm, int type, int sh)
{
	emit((uint32_t)cond << 28 | op << 21 | s << 20 | rn << 16 | rd << 12 | sh << 7 | type << 5 | rm);
}

static void movw(int rd, uint32_t v) { emit(0xE3000000 | (v >> 12 & 0xf) << 16 | rd << 12 | (v & 0xfff)); }
static void movt(int rd, uint32_t v) { emit(0xE3400000 | (v >> 12 & 0xf) << 16 | rd << 12 | (v & 0xfff)); }

static void mov32(int rd, uint32_t v)
{
	if (imm12(v) >= 0) { dpi(AL, OP_MOV, 0, rd, 0, v); return; }
	if (imm12(~v) >= 0) { dpi(AL, OP_MVN, 0, rd, 0, ~v); return; }
	movw(rd, v & 0xffff);
	if (v >> 16) movt(rd, v >> 16);
}

/* rd = rn op imm, through IP when the immediate does not encode */
static void opi(int op, int rd, int rn, uint32_t v)
{
	if (imm12(v) >= 0) dpi(AL, op, 0, rd, rn, v);
	else { mov32(IP, v); dpr(AL, op, 0, rd, rn, IP, 0, 0); }
}

static void cmpi(int rn, uint32_t v)
{
	if (imm12(v) >= 0) dpi(AL, OP_CMP, 1, 0, rn, v);
	else { mov32(IP, v); dpr(AL, OP_CMP, 1, 0, rn, IP, 0, 0); }
}

static void ldr(int rd, int rn, int off) { emit(0xE5900000 | rn << 16 | rd << 12 | off); }
static void str(int rd, int rn, int off) { emit(0xE5800000 | rn << 16 | rd << 12 | off); }
static void ldr_x(int rd, int rn, int rm) { emit(0xE7900100 | rn << 16 | rd << 12 | rm); }	/* [rn, rm, lsl #2] */
static void str_x(int rd, int rn, int rm) { emit(0xE7800100 | rn << 16 | rd << 12 | rm); }
static void ldrb_x(int rd, int rn, int rm) { emit(0xE7D00000 | rn << 16 | rd << 12 | rm); }
static void ubfx(int rd, int rn, int lsb, int width) { emit(0xE7E00050 | (width - 1) << 16 | rd << 12 | lsb << 7 | rn); }
static void blx(int rm) { emit(0xE12FFF30 | rm); }

static void call(const void *fn)
{
	mov32(IP, (uint32_t)(uintptr_t)fn);
	blx(IP);
}

/* branches: emit with a placeholder, patch later */
static uint32_t *bcc(int cond)
{
	uint32_t *at = ep;
	emit((uint32_t)cond << 28 | 0x0A000000);
	return at;
}

static void patch(uint32_t *at, uint32_t *target)
{
	if (!at || ep_overflow) return;
	*at = (*at & 0xff000000) | (((uint32_t)(target - at - 2)) & 0x00ffffff);
}

static void bto(int cond, uint32_t *target)
{
	uint32_t *at = bcc(cond);
	patch(at, target);
}

/*--------------------------------------------------------------------
 * code generation
 *
 * r4 &dsp_core, r5 cycles of the block, r6 X internal RAM, r7 Y internal
 * RAM, r8 external RAM, r9 this instruction's access bits, r10 this
 * instruction's cycles (and a scratch before that), r11 scratch.
 * Stack: [sp] x/single address, [sp+4] y address, [sp+8] value 1,
 * [sp+12] value 2.
 *------------------------------------------------------------------*/

#define OFF_REG(n)	((int)(offsetof(dsp_core_t, registers) + 4 * (n)))
#define S_ADDR1	0
#define S_ADDR2	4
#define S_VAL1	8
#define S_VAL2	12
#define S_CNT	16	/* instructions of earlier DO-loop passes */
#define S_AGUV	20	/* old value of the R/N register the previous instruction moved to */
#define S_FLO	24	/* deferred flags: the ALU result (lo, hi) and the ARM flags */
#define S_FHI	28
#define S_FPSR	32
#define S_FRAME	40

/* AGU pipeline at compile time: a parallel move to R/N takes effect for
   addressing one instruction later (Hatari's agu_pipeline).  jit_prev_agu
   is the register the previous instruction in the block moved to (0:
   none); its address computations use the old value, kept in S_AGUV.
   The move also stages agu_pipeline_reg/val[1] in dsp_core, which the
   next native instruction clears, jit_interp/jit_tail consume, and
   jit_post moves on when the block ends after it. */
static uint32_t jit_prev_agu, jit_cur_agu;

static void strh_core(int rt, uint32_t off)
{
	if (off < 256) emit(0xE1C000B0 | R4 << 16 | rt << 12 | (off >> 4 & 0xf) << 8 | (off & 0xf));
	else { mov32(IP, off); emit(0xE18000B0 | R4 << 16 | rt << 12 | IP); }
}

/* R[n] / N[n] as the address unit sees them */
static void ldr_rn(int rd, uint32_t numreg)
{
	if (jit_prev_agu == numreg) ldr(rd, SP, S_AGUV);
	else ldr(rd, R4, OFF_REG(numreg));
}
#define JIT_LOOP_BUDGET	256	/* instructions before a looping block returns */

/* address of R[n] per ea mode (1 (Rn)+Nn, 0 (Rn)-Nn, 2 (Rn)-, 3 (Rn)+, 4 (Rn)) to slot */
/* M registers a block was compiled for (step 5): a block records the ones
   its address updates use, and is recompiled when one differs at entry
   (only movec and the like change M, and they end blocks) */
static uint32_t jit_m[8];
static uint8_t jit_mmask;
static int jit_scal;			/* (defined with the native ALU below) */

static void bfc(int rd, int width) { emit(0xE7C0001F | (width - 1) << 16 | rd << 12); }

static void gen_ea(uint32_t ea_mode, int slot)
{
	uint32_t mode = (ea_mode >> 3) & 7, n = ea_mode & 7, m;

	ldr_rn(R10, DSP_REG_R0 + n);
	str(R10, SP, slot);
	if (mode == 4) return;
	jit_mmask |= 1 << n;
	m = jit_m[n];
	if (m == 0xffff) {
		switch (mode) {
		case 0: ldr_rn(R1, DSP_REG_N0 + n); dpr(AL, OP_SUB, 0, R0, R10, R1, 0, 0); break;
		case 1: ldr_rn(R1, DSP_REG_N0 + n); dpr(AL, OP_ADD, 0, R0, R10, R1, 0, 0); break;
		case 2: dpi(AL, OP_SUB, 0, R0, R10, 1); break;
		case 3: dpi(AL, OP_ADD, 0, R0, R10, 1); break;
		}
		ubfx(R0, R0, 0, 16);
		str(R0, R4, OFF_REG(DSP_REG_R0 + n));
		return;
	}
	if (m >= 1 && m <= 32767 && (mode == 2 || mode == 3)) {
		/* modulo, +-1 (dsp_update_rn_modulo with the bounds folded) */
		uint32_t k = 32 - __builtin_clz(m), modulo = m + 1;
		uint32_t *notabove, *done;
		dpi(AL, OP_ORR, 0, R0, R10, 0x10000);		/* r_reg */
		dpr(AL, OP_MOV, 0, R1, 0, R0, 0, 0);
		bfc(R1, k);					/* lobound */
		opi(OP_ADD, R2, R1, m);				/* hibound */
		dpi(AL, mode == 3 ? OP_ADD : OP_SUB, 0, R0, R0, 1);
		mov32(R3, modulo);
		dpr(AL, OP_CMP, 1, 0, R0, R2, 0, 0);
		notabove = bcc(LS);
		dpr(AL, OP_SUB, 0, R0, R0, R3, 0, 0);
		done = bcc(AL);
		patch(notabove, ep);
		dpr(AL, OP_CMP, 1, 0, R0, R1, 0, 0);
		dpr(LO, OP_ADD, 0, R0, R0, R3, 0, 0);
		patch(done, ep);
		ubfx(R0, R0, 0, 16);
		str(R0, R4, OFF_REG(DSP_REG_R0 + n));
		return;
	}
	/* other modes (modulo +-N, bit reverse): Hatari's update */
	dpi(AL, OP_MOV, 0, R0, 0, n);
	dpr(AL, OP_MOV, 0, R1, 0, R10, 0, 0);
	switch (mode) {
	case 0: ldr_rn(R2, DSP_REG_N0 + n); dpi(AL, OP_RSB, 0, R2, R2, 0); break;
	case 1: ldr_rn(R2, DSP_REG_N0 + n); break;
	case 2: dpi(AL, OP_MVN, 0, R2, 0, 0); break;
	case 3: dpi(AL, OP_MOV, 0, R2, 0, 1); break;
	}
	mov32(R3, m);
	call((const void *)jit_urn);
}

/* inline external-RAM index of the address in ra, for space, into IP;
   branches to *helper when the address is not plain external RAM */
static void gen_ext_index(int ra, int space, uint32_t **to_helper1, uint32_t **to_helper2)
{
	uint32_t rs = jit_ramsize;
	cmpi(ra, 0x200);
	*to_helper1 = bcc(LO);			/* $100-$1ff: ROM space rules */
	cmpi(ra, 0xffc0);
	*to_helper2 = bcc(HS);			/* peripherals */
	dpi(AL, OP_TST, 1, 0, ra, 0x8000);
	/* A15 low: index = address & (rs-1); high: Y lower half, X upper half */
	opi(OP_AND, R11, ra, rs - 1);
	{
		uint32_t *lo = bcc(EQ);
		opi(OP_AND, R11, ra, (rs >> 1) - 1);
		if (space == DSP_SPACE_X) opi(OP_ORR, R11, R11, rs >> 1);
		patch(lo, ep);
	}
	dpi(AL, OP_ORR, 0, R9, R9, 1 << space);
}

/* value of space:[address in R10] to slot (masked to 24 bits) */
static void gen_read(int space, int slot)
{
	uint32_t *helper0, *helper1 = 0, *helper2 = 0, *done1, *done2 = 0;

	cmpi(R10, 0x100);
	{
		uint32_t *ext = bcc(HS);
		ldr_x(R0, space == DSP_SPACE_X ? R6 : R7, R10);
		done1 = bcc(AL);
		patch(ext, ep);
	}
	if (jit_ramext) {
		gen_ext_index(R10, space, &helper1, &helper2);
		ldr_x(R0, R8, R11);
		done2 = bcc(AL);
	}
	helper0 = ep;
	patch(helper1, helper0);
	patch(helper2, helper0);
	dpi(AL, OP_MOV, 0, R0, 0, space);
	dpr(AL, OP_MOV, 0, R1, 0, R10, 0, 0);
	call((const void *)jit_rd);
	dpr(AL, OP_ORR, 0, R9, R9, R1, 0, 0);
	patch(done1, ep);
	patch(done2, ep);
	dpi(AL, OP_BIC, 0, R0, R0, 0xff000000);
	str(R0, SP, slot);
}

/* space:[address in R10] = R11 (24 bits) */
static void gen_write(int space)
{
	uint32_t *helper1 = 0, *helper2 = 0, *done1, *done2 = 0;

	dpi(AL, OP_BIC, 0, R2, R11, 0xff000000);
	cmpi(R10, 0x100);
	{
		uint32_t *ext = bcc(HS);
		str_x(R2, space == DSP_SPACE_X ? R6 : R7, R10);
		done1 = bcc(AL);
		patch(ext, ep);
	}
	if (jit_ramext) {
		gen_ext_index(R10, space, &helper1, &helper2);
		str_x(R2, R8, R11);
		/* jit_pver[index >> 6]++ */
		mov32(R3, (uint32_t)(uintptr_t)jit_pver);
		dpr(AL, OP_MOV, 0, R1, 0, R11, 1, 6);
		ldr_x(R0, R3, R1);
		dpi(AL, OP_ADD, 0, R0, R0, 1);
		str_x(R0, R3, R1);
		for (int r = 0; r < jit_nrange; r++) {
			uint32_t *below;
			cmpi(R11, jit_rlo[r]);
			below = bcc(LO);
			cmpi(R11, jit_rhi[r]);
			dpi(LS, OP_ORR, 0, R9, R9, JF_SMC);
			patch(below, ep);
		}
		done2 = bcc(AL);
	}
	patch(helper1, ep);
	patch(helper2, ep);
	dpi(AL, OP_MOV, 0, R0, 0, space);
	dpr(AL, OP_MOV, 0, R1, 0, R10, 0, 0);
	call((const void *)jit_wr);
	dpr(AL, OP_ORR, 0, R9, R9, R0, 0, 0);
	patch(done1, ep);
	patch(done2, ep);
}

/* register source (24 bits, A/B limited) to slot */
static void gen_src(uint32_t numreg, int slot)
{
	if ((numreg == DSP_REG_A || numreg == DSP_REG_B) && jit_scal == 0) {
		/* A1/B1, limited when A2 is not A1's sign extension
		   (dsp_pm_read_accu24, scaling mode 0) */
		int r = numreg & 1;
		uint32_t *ok;
		ldr(R0, R4, OFF_REG(DSP_REG_A1 + r));
		ldr(R1, R4, OFF_REG(DSP_REG_A2 + r));
		dpr(AL, OP_ORR, 0, R2, R0, R1, 0, 24);
		dpr(AL, OP_MOV, 0, R3, 0, R2, 0, 8);
		dpr(AL, OP_CMP, 1, 0, R2, R3, 2, 8);	/* cmp r2, r3, asr #8 */
		ok = bcc(EQ);
		dpi(AL, OP_MOV, 0, R0, 0, 0x800000);
		dpi(AL, OP_TST, 1, 0, R1, 0x80);
		dpi(EQ, OP_SUB, 0, R0, R0, 1);
		ldr(R2, R4, OFF_REG(DSP_REG_SR));
		dpi(AL, OP_ORR, 0, R2, R2, 1 << DSP_SR_L);
		str(R2, R4, OFF_REG(DSP_REG_SR));
		patch(ok, ep);
	} else if (numreg == DSP_REG_A || numreg == DSP_REG_B) {
		dpi(AL, OP_MOV, 0, R0, 0, numreg);
		call((const void *)jit_accu);
	} else {
		ldr(R0, R4, OFF_REG(numreg));
	}
	str(R0, SP, slot);
}

/* register destination from slot: X0..Y1 (24 bits) or A/B (sign extended) */
static void gen_dst(uint32_t numreg, int slot, int mask)
{
	ldr(R0, SP, slot);
	if (numreg == DSP_REG_A || numreg == DSP_REG_B) {
		int r = numreg & 1;
		if (mask) dpi(AL, OP_BIC, 0, R0, R0, 0xff000000);
		dpi(AL, OP_MOV, 0, R1, 0, 0);
		str(R1, R4, OFF_REG(DSP_REG_A0 + r));
		str(R0, R4, OFF_REG(DSP_REG_A1 + r));
		dpi(AL, OP_TST, 1, 0, R0, 0x800000);
		dpi(NE, OP_MOV, 0, R1, 0, 0xff);
		str(R1, R4, OFF_REG(DSP_REG_A2 + r));
	} else if (numreg >= DSP_REG_R0 && numreg <= DSP_REG_N7) {
		/* delayed for addressing: stage the old value (dsp_write_reg) */
		ldr(R1, R4, OFF_REG(numreg));
		ubfx(R1, R1, 0, 16);		/* (agu_pipeline_val is 16 bits) */
		str(R1, SP, S_AGUV);
		ubfx(R0, R0, 0, 16);
		str(R0, R4, OFF_REG(numreg));
		dpi(AL, OP_MOV, 0, R2, 0, numreg);
		strh_core(R2, offsetof(dsp_core_t, agu_pipeline_reg[1]));
		strh_core(R1, offsetof(dsp_core_t, agu_pipeline_val[1]));
		jit_cur_agu = numreg;
	} else if (numreg == DSP_REG_A2 || numreg == DSP_REG_B2) {
		dpi(AL, OP_AND, 0, R0, R0, 0xff);
		str(R0, R4, OFF_REG(numreg));
	} else {
		if (mask) dpi(AL, OP_BIC, 0, R0, R0, 0xff000000);
		str(R0, R4, OFF_REG(numreg));
	}
}

/*--------------------------------------------------------------------
 * native data ALU (step 2), scaling mode 0 only (a block remembers the
 * mode it was compiled for; nothing inside a block changes SR's S bits)
 *
 * An accumulator is held as its 56-bit value shifted left by 8 in a
 * 64-bit pair: lo = A0 << 8, hi = A1 | A2 << 24.  Then ARM adds/adcs
 * (subs/sbcs) give the 56-bit carry (borrow: inverted) and overflow
 * directly, N is hi bit 31, and the low byte of lo is always 0.
 *------------------------------------------------------------------*/

#define F_C	0x01
#define F_V	0x02
#define F_Z	0x04
#define F_N	0x08
#define F_U	0x10
#define F_E	0x20
#define F_L	0x40

static int jit_scal;			/* SR scaling bits of the block being compiled */
static int jit_dbg;			/* NEXT_DSP_JIT_DBG=1: record each instruction's cycles (tests) */
uint32_t dsp_jit_dbg[2 * JIT_MAX + 4];	/* [i]: cycles of the block's instruction i, [JIT_MAX + i] the block's cycles so far */
static void jit_dbg_rec(uint32_t i, uint32_t cycles, uint32_t bits)
{
	if (i < JIT_MAX) { dsp_jit_dbg[i] = cycles; dsp_jit_dbg[JIT_MAX + i] = bits; }
}
static int jit_alu_off;			/* NEXT_DSP_JIT_ALU=0: call Hatari for every ALU op */

static void smull(int lo, int hi, int rn, int rm) { emit(0xE0C00090 | hi << 16 | lo << 12 | rm << 8 | rn); }

static void acc_load(int acc, int lo, int hi)
{
	ldr(lo, R4, OFF_REG(DSP_REG_A0 + acc));
	dpr(AL, OP_MOV, 0, lo, 0, lo, 0, 8);
	ldr(hi, R4, OFF_REG(DSP_REG_A1 + acc));
	ldr(IP, R4, OFF_REG(DSP_REG_A2 + acc));
	dpr(AL, OP_ORR, 0, hi, hi, IP, 0, 24);
}

static void acc_store(int acc, int lo, int hi)
{
	dpr(AL, OP_MOV, 0, IP, 0, lo, 1, 8);
	str(IP, R4, OFF_REG(DSP_REG_A0 + acc));
	dpi(AL, OP_BIC, 0, IP, hi, 0xff000000);
	str(IP, R4, OFF_REG(DSP_REG_A1 + acc));
	dpr(AL, OP_MOV, 0, IP, 0, hi, 1, 24);
	str(IP, R4, OFF_REG(DSP_REG_A2 + acc));
}

/* hi = a 24-bit register sign extended to the accumulator's top word */
static void reg_hi(int hi, uint32_t numreg)
{
	ldr(hi, R4, OFF_REG(numreg));
	dpr(AL, OP_MOV, 0, hi, 0, hi, 0, 8);
	dpr(AL, OP_MOV, 0, hi, 0, hi, 2, 8);
}

/* sr |= E U N Z of (hi:lo), scaling mode 0 (dsp_ccr_update_e_u_n_z) */
static void gen_eunz(int lo, int hi, int sr)
{
	dpr(AL, OP_ORR, 1, R3, lo, hi, 0, 0);
	dpi(EQ, OP_ORR, 0, sr, sr, F_Z);
	dpi(AL, OP_TST, 1, 0, hi, 0x80000000);
	dpi(NE, OP_ORR, 0, sr, sr, F_N);
	/* E: bits 55..47 not all equal */
	dpi(AL, OP_ADD, 0, R3, hi, 0x800000);
	dpr(AL, OP_MOV, 1, R3, 0, R3, 1, 24);
	dpi(NE, OP_ORR, 0, sr, sr, F_E);
	/* U: bits 47 and 46 equal */
	dpr(AL, OP_EOR, 0, R3, hi, hi, 0, 1);
	dpi(AL, OP_TST, 1, 0, R3, 0x800000);
	dpi(EQ, OP_ORR, 0, sr, sr, F_U);
}

/* convergent rounding, scaling mode 0 (dsp_rnd56) */
static void gen_rnd(int lo, int hi)
{
	dpi(AL, OP_ADD, 1, lo, lo, 0x80000000);
	dpi(AL, OP_ADC, 0, hi, hi, 0);
	dpi(AL, OP_CMP, 1, 0, lo, 0);
	dpi(EQ, OP_BIC, 0, hi, hi, 1);
	dpi(AL, OP_MOV, 0, lo, 0, 0);
}

static void sr_load(int sr, uint32_t clear)
{
	ldr(sr, R4, OFF_REG(DSP_REG_SR));
	dpi(AL, OP_BIC, 0, sr, sr, clear);
}

/* deferred flags (step 5): an ALU op whose flags the next instruction's
   ALU op overwrites skips computing them.  It still sets L (sticky), and
   keeps its result and the ARM flags in S_FLO/S_FHI/S_FPSR, so that an
   exit right after it can build SR (gen_lazy_sr).  Classes: 1 MAC/MACR,
   2 MPY/MPYR, 3 ADD, 4 SUB/CMP, 5 TST, 6 RND. */
static int jit_lazy;

static int alu_cls(uint32_t b)
{
	if (jit_scal != 0 || jit_alu_off) return 0;
	if (b & 0x80) return (b & 2) ? 1 : 2;
	switch (b) {
	case 0x03: case 0x0b: return 5;
	case 0x11: case 0x19: return 6;
	case 0x10: case 0x18: case 0x20: case 0x28: case 0x30: case 0x38: return 3;
	case 0x05: case 0x0d: case 0x14: case 0x1c: case 0x24: case 0x2c: case 0x34: case 0x3c: return 4;
	}
	if (b >= 0x40) switch (b & 7) { case 0: return 3; case 4: case 5: return 4; }
	return 0;
}

/* CCR bits an ALU op always writes (natively) */
static uint32_t alu_wmask(uint32_t b)
{
	static const uint8_t w[7] = { 0, 0x3e, 0x3e, 0x3f, 0x3f, 0x3e, 0x3c };
	if (jit_scal == 0 && !jit_alu_off && (b == 0x13 || b == 0x1b)) return 0x3e;	/* CLR */
	return w[alu_cls(b)];
}

static void condw(int cond) { ep[-1] = (ep[-1] & 0x0fffffff) | (uint32_t)cond << 28; }

/* after the op's adds/adcs: keep the ARM flags, and set L if V */
static void lazy_vl(void)
{
	emit(0xE10F0000 | R3 << 12);			/* mrs r3, apsr */
	str(R3, SP, S_FPSR);
	ldr(R2, R4, OFF_REG(DSP_REG_SR)); condw(VS);
	dpi(VS, OP_ORR, 0, R2, R2, F_L);
	str(R2, R4, OFF_REG(DSP_REG_SR)); condw(VS);
}

static void lazy_keep(int lo, int hi)
{
	str(lo, SP, S_FLO);
	str(hi, SP, S_FHI);
}

/* SR from deferred flags of class cls */
static void gen_lazy_sr(int cls)
{
	static const uint8_t clr[7] = { 0, 0x3e, 0x3e, 0x3f, 0x3f, 0x3e, 0x3c };
	ldr(R0, SP, S_FLO);
	ldr(R1, SP, S_FHI);
	ldr(R2, R4, OFF_REG(DSP_REG_SR));
	dpi(AL, OP_BIC, 0, R2, R2, clr[cls]);
	if (cls == 1 || cls == 3 || cls == 4) {
		ldr(R3, SP, S_FPSR);
		dpi(AL, OP_TST, 1, 0, R3, 1u << 28);
		dpi(NE, OP_ORR, 0, R2, R2, F_V);
		if (cls != 1) {
			dpi(AL, OP_TST, 1, 0, R3, 1u << 29);
			dpi(cls == 3 ? NE : EQ, OP_ORR, 0, R2, R2, F_C);
		}
	}
	gen_eunz(R0, R1, R2);
	str(R2, R4, OFF_REG(DSP_REG_SR));
}

/* ALU byte b natively; returns 0 (nothing emitted) for the ones left to
   Hatari: logic, shifts, ADC/SBC, ADDL/ADDR/SUBL/SUBR, CMPM, ABS/NEG/NOT */
static int gen_alu_native(uint32_t b)
{
	static const uint8_t qqq[8][2] = {
		{ DSP_REG_X0, DSP_REG_X0 }, { DSP_REG_Y0, DSP_REG_Y0 }, { DSP_REG_X1, DSP_REG_X0 }, { DSP_REG_Y1, DSP_REG_Y0 },
		{ DSP_REG_X0, DSP_REG_Y1 }, { DSP_REG_Y0, DSP_REG_X0 }, { DSP_REG_X1, DSP_REG_Y0 }, { DSP_REG_Y1, DSP_REG_X1 } };
	static const uint8_t jj[4] = { DSP_REG_X0, DSP_REG_Y0, DSP_REG_X1, DSP_REG_Y1 };
	int d = (b >> 3) & 1;
	int op;			/* 0 add, 1 sub, 2 cmp, 3 tfr */
	int src;		/* 0..7 reg number (24-bit), 8 X, 9 Y, 10 other accumulator */

	if (jit_scal != 0 || jit_alu_off) return 0;

	if (b & 0x80) {		/* MPY MPYR MAC MACR */
		int kind = b & 3, minus = b & 4, lo = R10, hi = R11;
		ldr(R0, R4, OFF_REG(qqq[(b >> 4) & 7][0]));
		ldr(R1, R4, OFF_REG(qqq[(b >> 4) & 7][1]));
		dpr(AL, OP_MOV, 0, R0, 0, R0, 0, 8);
		dpr(AL, OP_MOV, 0, R1, 0, R1, 0, 8);
		dpr(AL, OP_MOV, 0, R1, 0, R1, 2, 7);
		smull(R10, R11, R0, R1);	/* 2xy << 8 */
		if (!jit_lazy) sr_load(R2, F_V | F_Z | F_N | F_U | F_E);
		if (kind < 2) {
			if (minus) {
				dpi(AL, OP_RSB, 1, R10, R10, 0);
				dpi(AL, OP_RSC, 0, R11, R11, 0);
			}
		} else {
			acc_load(d, R0, R1);
			dpr(AL, minus ? OP_SUB : OP_ADD, 1, R0, R0, R10, 0, 0);
			dpr(AL, minus ? OP_SBC : OP_ADC, 1, R1, R1, R11, 0, 0);
			if (jit_lazy) lazy_vl();
			else dpi(VS, OP_ORR, 0, R2, R2, F_V | F_L);
			lo = R0; hi = R1;
		}
		if (kind & 1) gen_rnd(lo, hi);
		acc_store(d, lo, hi);
		if (jit_lazy) { lazy_keep(lo, hi); return 1; }
		gen_eunz(lo, hi, R2);
		str(R2, R4, OFF_REG(DSP_REG_SR));
		return 1;
	}

	switch (b) {
	case 0x01: case 0x09:	/* TFR B,A / A,B */
		ldr(R0, R4, OFF_REG(DSP_REG_A0 + !d));
		ldr(R1, R4, OFF_REG(DSP_REG_A1 + !d));
		ldr(R2, R4, OFF_REG(DSP_REG_A2 + !d));
		str(R0, R4, OFF_REG(DSP_REG_A0 + d));
		str(R1, R4, OFF_REG(DSP_REG_A1 + d));
		str(R2, R4, OFF_REG(DSP_REG_A2 + d));
		return 1;
	case 0x03: case 0x0b:	/* TST */
		acc_load(d, R0, R1);
		if (jit_lazy) { lazy_keep(R0, R1); return 1; }
		sr_load(R2, F_V | F_Z | F_N | F_U | F_E);
		gen_eunz(R0, R1, R2);
		str(R2, R4, OFF_REG(DSP_REG_SR));
		return 1;
	case 0x11: case 0x19:	/* RND (V and C kept) */
		acc_load(d, R0, R1);
		gen_rnd(R0, R1);
		acc_store(d, R0, R1);
		if (jit_lazy) { lazy_keep(R0, R1); return 1; }
		sr_load(R2, F_Z | F_N | F_U | F_E);
		gen_eunz(R0, R1, R2);
		str(R2, R4, OFF_REG(DSP_REG_SR));
		return 1;
	case 0x13: case 0x1b:	/* CLR */
		dpi(AL, OP_MOV, 0, R0, 0, 0);
		str(R0, R4, OFF_REG(DSP_REG_A0 + d));
		str(R0, R4, OFF_REG(DSP_REG_A1 + d));
		str(R0, R4, OFF_REG(DSP_REG_A2 + d));
		sr_load(R2, F_V | F_Z | F_N | F_U | F_E);
		dpi(AL, OP_ORR, 0, R2, R2, F_U | F_Z);
		str(R2, R4, OFF_REG(DSP_REG_SR));
		return 1;
	case 0x05: case 0x0d: op = 2; src = 10; break;	/* CMP B,A / A,B */
	case 0x10: case 0x18: op = 0; src = 10; break;	/* ADD B,A / A,B */
	case 0x14: case 0x1c: op = 1; src = 10; break;	/* SUB */
	case 0x20: case 0x28: op = 0; src = 8; break;	/* ADD X */
	case 0x30: case 0x38: op = 0; src = 9; break;	/* ADD Y */
	case 0x24: case 0x2c: op = 1; src = 8; break;	/* SUB X */
	case 0x34: case 0x3c: op = 1; src = 9; break;	/* SUB Y */
	default:
		if (b < 0x40) return 0;
		src = jj[(b >> 4) & 3];
		switch (b & 7) {
		case 0: op = 0; break;
		case 4: op = 1; break;
		case 5: op = 2; break;
		case 1:		/* TFR reg,D: A0 = 0, A1 = reg, A2 = sign */
			reg_hi(R1, src);
			dpi(AL, OP_MOV, 0, R0, 0, 0);
			acc_store(d, R0, R1);
			return 1;
		default: return 0;
		}
	}

	/* ADD / SUB / CMP: source in R10 (lo) R11 (hi), or R11 alone (lo 0) */
	if (src == 10) acc_load(!d, R10, R11);
	else if (src >= 8) {
		uint32_t r1 = src == 8 ? DSP_REG_X1 : DSP_REG_Y1, r0 = src == 8 ? DSP_REG_X0 : DSP_REG_Y0;
		ldr(R10, R4, OFF_REG(r0));
		dpr(AL, OP_MOV, 0, R10, 0, R10, 0, 8);
		reg_hi(R11, r1);
	} else reg_hi(R11, src);
	acc_load(d, R0, R1);
	if (jit_lazy) {
		if (op == 0) {
			if (src < 8) dpi(AL, OP_ADD, 1, R0, R0, 0);
			else dpr(AL, OP_ADD, 1, R0, R0, R10, 0, 0);
			dpr(AL, OP_ADC, 1, R1, R1, R11, 0, 0);
		} else {
			if (src < 8) dpi(AL, OP_SUB, 1, R0, R0, 0);
			else dpr(AL, OP_SUB, 1, R0, R0, R10, 0, 0);
			dpr(AL, OP_SBC, 1, R1, R1, R11, 0, 0);
		}
		lazy_vl();
		if (op != 2) acc_store(d, R0, R1);
		lazy_keep(R0, R1);
		return 1;
	}
	sr_load(R2, F_C | F_V | F_Z | F_N | F_U | F_E);
	if (op == 0) {
		if (src < 8) dpi(AL, OP_ADD, 1, R0, R0, 0);
		else dpr(AL, OP_ADD, 1, R0, R0, R10, 0, 0);
		dpr(AL, OP_ADC, 1, R1, R1, R11, 0, 0);
		dpi(HS, OP_ORR, 0, R2, R2, F_C);
	} else {
		if (src < 8) dpi(AL, OP_SUB, 1, R0, R0, 0);
		else dpr(AL, OP_SUB, 1, R0, R0, R10, 0, 0);
		dpr(AL, OP_SBC, 1, R1, R1, R11, 0, 0);
		dpi(LO, OP_ORR, 0, R2, R2, F_C);
	}
	dpi(VS, OP_ORR, 0, R2, R2, F_V | F_L);
	if (op != 2) acc_store(d, R0, R1);
	gen_eunz(R0, R1, R2);
	str(R2, R4, OFF_REG(DSP_REG_SR));
	return 1;
}

static void gen_alu(uint32_t inst)
{
	dsp_emul_t f = opcodes_alu[inst & 0xff];
	if (f == dsp_move) return;
	if (gen_alu_native(inst & 0xff)) return;
	call((const void *)f);
}

static int jit_dst_ok(uint32_t r)  { return r >= DSP_REG_X0 && r <= DSP_REG_N7; }	/* X0..B, R0..N7 */
static int jit_tcc_ok(uint32_t inst) { return (inst & 0xff0f87) == 0x020000 && registers_tcc[(inst >> 3) & 15][0] != DSP_REG_NULL; }
static int jit_src_ok(uint32_t r)  { return (r >= DSP_REG_X0 && r <= DSP_REG_B) || (r >= DSP_REG_R0 && r <= DSP_REG_N7); }

/* can this word be compiled? (mirrors the gen_inst cases) */
static int jit_native(uint32_t inst)
{
	if (inst == 0) return 1;				/* nop */
	if (inst < 0x100000) return jit_tcc_ok(inst);
	if (opcodes_alu[inst & 0xff] == dsp_undefined) return 0;
	switch ((inst >> 20) & 0xf) {
	case 2: case 3:
		if ((inst & 0xffff00) == 0x200000) return 1;
		if ((inst & 0xffe000) == 0x204000) return 1;	/* R update */
		if ((inst & 0xfc0000) == 0x200000)
			return jit_src_ok((inst >> 13) & 0x1f) && jit_dst_ok((inst >> 8) & 0x1f);
		return jit_dst_ok((inst >> 16) & 0x1f);		/* #xx,D */
	case 4:
		if ((inst & 0xf40000) == 0x400000) return 0;
		/* fall through */
	case 5: case 6: case 7: {
		uint32_t numreg = ((inst >> 16) & 7) | ((inst >> 17) & (3 << 3));
		if ((inst & (1 << 14)) && ((inst >> 11) & 7) > 4) return 0;
		return (inst & (1 << 15)) ? jit_dst_ok(numreg) : jit_src_ok(numreg);
	}
	case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15:
		return 1;
	}
	return 0;
}

/* does a native instruction access X/Y memory (and so set r9)? */
static int jit_mem_inst(uint32_t inst)
{
	return inst >= 0x100000 && ((inst >> 20) & 0xf) >= 4;
}

/* one instruction (after jit_native said yes) */
static void gen_inst(uint32_t inst)
{
	static const uint8_t xreg[4] = { DSP_REG_X0, DSP_REG_X1, DSP_REG_A, DSP_REG_B };
	static const uint8_t yreg[4] = { DSP_REG_Y0, DSP_REG_Y1, DSP_REG_A, DSP_REG_B };

	if (inst == 0) return;					/* nop */
	if (inst < 0x100000) {					/* Tcc S1,D1 */
		uint32_t s1 = registers_tcc[(inst >> 3) & 15][0], d1 = registers_tcc[(inst >> 3) & 15][1] & 1;
		uint32_t *skip;
		ldr(R0, R4, OFF_REG(DSP_REG_SR));
		dpi(AL, OP_AND, 0, R0, R0, 0x7f);
		mov32(IP, (uint32_t)(uintptr_t)jit_cc[(inst >> 12) & 15]);
		ldrb_x(R0, IP, R0);
		dpi(AL, OP_CMP, 1, 0, R0, 0);
		skip = bcc(EQ);
		if (s1 == DSP_REG_A || s1 == DSP_REG_B) {
			ldr(R1, R4, OFF_REG(DSP_REG_A0 + (s1 & 1)));
			ldr(R2, R4, OFF_REG(DSP_REG_A1 + (s1 & 1)));
			ldr(R3, R4, OFF_REG(DSP_REG_A2 + (s1 & 1)));
		} else {
			dpi(AL, OP_MOV, 0, R1, 0, 0);
			ldr(R2, R4, OFF_REG(s1));
			dpi(AL, OP_MOV, 0, R3, 0, 0);
			dpi(AL, OP_TST, 1, 0, R2, 0x800000);
			dpi(NE, OP_MOV, 0, R3, 0, 0xff);
		}
		str(R1, R4, OFF_REG(DSP_REG_A0 + d1));
		str(R2, R4, OFF_REG(DSP_REG_A1 + d1));
		str(R3, R4, OFF_REG(DSP_REG_A2 + d1));
		patch(skip, ep);
		return;
	}

	switch ((inst >> 20) & 0xf) {
	case 2: case 3:
		if ((inst & 0xffff00) == 0x200000) {
			gen_alu(inst);
		} else if ((inst & 0xffe000) == 0x204000) {	/* R update, then the ALU */
			gen_ea((inst >> 8) & 0x1f, S_ADDR1);
			gen_alu(inst);
		} else if ((inst & 0xfc0000) != 0x200000) {	/* #xx,D after the ALU */
			uint32_t dst = (inst >> 16) & 0x1f, v = (inst >> 8) & 0xff;
			gen_alu(inst);
			if ((dst >= DSP_REG_X0 && dst <= DSP_REG_Y1) || dst == DSP_REG_A || dst == DSP_REG_B) v <<= 16;
			mov32(R0, v);
			str(R0, SP, S_VAL1);
			gen_dst(dst, S_VAL1, 0);
		} else {
			uint32_t src = (inst >> 13) & 0x1f, dst = (inst >> 8) & 0x1f;
			gen_src(src, S_VAL1);
			gen_alu(inst);
			gen_dst(dst, S_VAL1, 1);
		}
		break;
	case 4: case 5: case 6: case 7: {
		uint32_t space = (inst >> 19) & 1;
		uint32_t numreg = ((inst >> 16) & 7) | ((inst >> 17) & (3 << 3));
		uint32_t a = (inst >> 8) & 0x3f;
		if (inst & (1 << 14)) {
			gen_ea(a, S_ADDR1);
		} else {
			dpi(AL, OP_MOV, 0, R0, 0, a);
			str(R0, SP, S_ADDR1);
		}
		if (inst & (1 << 15)) {
			ldr(R10, SP, S_ADDR1);
			gen_read(space, S_VAL1);
		} else {
			gen_src(numreg, S_VAL1);
		}
		gen_alu(inst);
		if (inst & (1 << 15)) {
			gen_dst(numreg, S_VAL1, 1);
		} else {
			ldr(R10, SP, S_ADDR1);
			ldr(R11, SP, S_VAL1);
			gen_write(space);
		}
		break;
	}
	default: {	/* XY */
		uint32_t ea1, ea2, r1, r2;
		ea1 = (inst >> 8) & 0x1f;
		if ((ea1 >> 3) == 0) ea1 |= (1 << 5);
		ea2 = (inst >> 13) & 3;
		ea2 |= (inst >> 17) & (3 << 3);
		if ((ea1 & (1 << 2)) == 0) ea2 |= 1 << 2;
		if ((ea2 >> 3) == 0) ea2 |= (1 << 5);
		r1 = xreg[(inst >> 18) & 3];
		r2 = yreg[(inst >> 16) & 3];
		gen_ea(ea1, S_ADDR1);
		gen_ea(ea2, S_ADDR2);
		if (inst & (1 << 15)) { ldr(R10, SP, S_ADDR1); gen_read(DSP_SPACE_X, S_VAL1); }
		else gen_src(r1, S_VAL1);
		if (inst & (1 << 22)) { ldr(R10, SP, S_ADDR2); gen_read(DSP_SPACE_Y, S_VAL2); }
		else gen_src(r2, S_VAL2);
		gen_alu(inst);
		if (inst & (1 << 15)) gen_dst(r1, S_VAL1, 0);
		else { ldr(R10, SP, S_ADDR1); ldr(R11, SP, S_VAL1); gen_write(DSP_SPACE_X); }
		if (inst & (1 << 22)) gen_dst(r2, S_VAL2, 0);
		else { ldr(R10, SP, S_ADDR2); ldr(R11, SP, S_VAL2); gen_write(DSP_SPACE_Y); }
		break;
	}
	}
}

/* Hatari's handler for one instruction inside a block (a form the code
   generator does not cover, but which only moves data: a parallel move,
   nop, Tcc); returns its cycles | exit flags << 16.  A pending AGU delay
   (a move to R/N/M) ends the block too: the next instruction must see it. */
static uint32_t jit_interp(uint32_t inst, uint32_t addr)
{
	uint32_t value, f = 0, is = dsp_core.interrupt_status;

	dsp_core.pc = addr;
	cur_inst = inst;
	cur_inst_len = 1;
	dsp_core.instr_cycle = 2;
	access_to_ext_memory = addr >= 0x200 ? 1 << DSP_SPACE_P : 0;
	dsp_core.agu_move_indirect_instr = 0;
	jit_periph = jit_extw = 0;
	/* the previous (native) instruction's delayed R/N move, as the
	   interpreter's end of instruction would have staged it */
	dsp_core.agu_pipeline_reg[0] = dsp_core.agu_pipeline_reg[1];
	dsp_core.agu_pipeline_val[0] = dsp_core.agu_pipeline_val[1];
	dsp_core.agu_pipeline_reg[1] = 0;
	if (inst < 0x100000) {
		value = (inst >> 11) & (BITMASK(6) << 3);
		value += (inst >> 5) & BITMASK(3);
		opcodes8h[value]();
	} else {
		opcodes_parmove[(inst >> 20) & BITMASK(4)]();
	}
	if (access_to_ext_memory != 0) {
		value  = (access_to_ext_memory >> DSP_SPACE_X) & 1;
		value += (access_to_ext_memory >> DSP_SPACE_Y) & 1;
		value += (access_to_ext_memory >> DSP_SPACE_P) & 1;
		if (value > 1)
			dsp_core.instr_cycle += (value - 1) * 2;
	}
	if (jit_periph || dsp_core.agu_pipeline_reg[1] || dsp_core.interrupt_status != is) f |= JF_PERIPH;
	if (jit_extw) f |= JF_SMC;
	dsp_core.agu_pipeline_reg[0] = 0;
	return dsp_core.instr_cycle | f << 16;
}

/* any other instruction, as the block's last: Hatari's handler, leaving
   the PC and length it sets in jit_out for jit_post (which then does the
   interpreter's end of instruction: REP/DO, AGU pipeline, interrupts);
   returns its cycles */
static uint32_t jit_tail(uint32_t inst, uint32_t addr)
{
	uint32_t value;

	dsp_core.pc = addr;
	cur_inst = inst;
	cur_inst_len = 1;
	dsp_core.instr_cycle = 2;
	access_to_ext_memory = addr >= 0x200 ? 1 << DSP_SPACE_P : 0;
	dsp_core.agu_move_indirect_instr = 0;
	dsp_core.agu_pipeline_reg[0] = dsp_core.agu_pipeline_reg[1];
	dsp_core.agu_pipeline_val[0] = dsp_core.agu_pipeline_val[1];
	dsp_core.agu_pipeline_reg[1] = 0;
	if (inst < 0x100000) {
		value = (inst >> 11) & (BITMASK(6) << 3);
		value += (inst >> 5) & BITMASK(3);
		opcodes8h[value]();
	} else {
		opcodes_parmove[(inst >> 20) & BITMASK(4)]();
	}
	if (access_to_ext_memory != 0) {
		value  = (access_to_ext_memory >> DSP_SPACE_X) & 1;
		value += (access_to_ext_memory >> DSP_SPACE_Y) & 1;
		value += (access_to_ext_memory >> DSP_SPACE_P) & 1;
		if (value > 1)
			dsp_core.instr_cycle += (value - 1) * 2;
	}
	jit_out.pc = dsp_core.pc;
	jit_out.len = cur_inst_len;
	return dsp_core.instr_cycle;
}

/* 1: compiled; 2: through jit_interp inside a block; 3: through jit_tail,
   as a block's last instruction; 0: not in blocks */
static int jit_kind(uint32_t inst)
{
	if (jit_native(inst)) return 1;
	if (inst < 0x100000) {
		uint32_t value = ((inst >> 11) & (BITMASK(6) << 3)) + ((inst >> 5) & BITMASK(3));
		if (opcodes8h[value] == dsp_tcc) return 2;
		return opcodes8h[value] == dsp_undefined ? 0 : 3;
	}
	if (opcodes_alu[inst & 0xff] == dsp_undefined) return 0;
	return 2;
}

/* words of a jit_kind 1 or 2 instruction (an absolute address or an
   immediate in the second word: ea mode 6) */
static int jit_len(uint32_t inst)
{
	uint32_t g = (inst >> 20) & 0xf, ea = (inst >> 8) & 0x3f;
	if (inst < 0x100000) return 1;
	if (g == 0 || g == 1) return (ea >> 3) == 6 ? 2 : 1;
	if (g >= 4 && g <= 7) return ((inst & (1 << 14)) && (ea >> 3) == 6) ? 2 : 1;
	return 1;
}

/* P word at pc and where it lives (NULL: none) */
static const uint32_t *jit_psrc(uint16_t pc)
{
	if (pc < 0x200) return &dsp_core.ramint[DSP_SPACE_P][pc];
	if (dsp_core.ramext) return &dsp_core.ramext[pc & (DSP_RAMSIZE - 1)];
	return NULL;
}

/* the code version counter of the page holding P address pc */
static const uint32_t *jit_pver_of(uint32_t pc)
{
	static const uint32_t none;
	if (pc < 0x200) return &jit_iver[pc >> 6];
	if (!dsp_core.ramext || !DSP_RAMSIZE) return &none;	/* (a change of RAM flushes) */
	return &jit_pver[(pc & (DSP_RAMSIZE - 1)) >> 6];
}

static void jit_flush(void)
{
	memset(jit_tab, 0, sizeof(jit_tab));
	jit_pool_n = 0;
	jit_ptr = jit_mem;
	jit_ramext = dsp_core.ramext;
	jit_ramsize = DSP_RAMSIZE;
}

static int jit_init(void)
{
	const char *e = getenv("NEXT_DSP_JIT");
	if (e && *e == '0') { jit_off = 1; return -1; }
	e = getenv("NEXT_DSP_JIT_MAX");
	if (e && atoi(e) > 0 && atoi(e) < JIT_MAX) jit_max = atoi(e);
	e = getenv("NEXT_DSP_JIT_DBG");
	if (e && *e >= '1' && *e <= '2') jit_dbg = *e - '0';	/* 2: also exit after every deferred-flags op */
	e = getenv("NEXT_DSP_JIT_ALU");
	if (e && *e == '0') jit_alu_off = 1;
	jit_mem = mmap(NULL, JIT_CODE_SIZE, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (jit_mem == MAP_FAILED) { jit_mem = NULL; jit_off = 1; return -1; }
	for (int cc = 0; cc < 16; cc++)
		for (int sr = 0; sr < 128; sr++) jit_cc[cc][sr] = jit_cc_eval(cc, sr);
	jit_pool = calloc(JIT_POOL, sizeof(jit_block_t));
	if (!jit_pool) { jit_off = 1; return -1; }
	jit_flush();
	return 0;
}

/* control instructions compiled into blocks (step 4) */
enum { K_END = 0, K_NAT = 1, K_INT = 2, K_TAIL = 3, K_DO = 4, K_JMP = 5, K_JCC = 6 };

/* a jump, Jcc or DO the compiler handles: kind, target (DO: LA), words,
   cycles; 0 for anything else */
static int jit_ctl(uint32_t inst, uint32_t a, uint32_t *tgt, int *len, int *cyc, int *cc)
{
	dsp_emul_t f;
	const uint32_t *s2;
	if (inst >= 0x100000) return 0;
	f = opcodes8h[((inst >> 11) & (BITMASK(6) << 3)) + ((inst >> 5) & BITMASK(3))];
	if (f == dsp_jmp_imm || f == dsp_jcc_imm) {
		*tgt = inst & BITMASK(12); *len = 1; *cyc = 4; *cc = (inst >> 12) & 15;
		return f == dsp_jmp_imm ? K_JMP : K_JCC;
	}
	if (a >= 0xffff || !(s2 = jit_psrc(a + 1))) return 0;
	if ((f == dsp_jmp_ea || f == dsp_jcc_ea) && ((inst >> 8) & 0x3f) == 0x30) {	/* absolute */
		*tgt = *s2 & 0xffff; *len = 2; *cyc = 6; *cc = inst & 15;
		return f == dsp_jmp_ea ? K_JMP : K_JCC;
	}
	if (f == dsp_do_imm || f == dsp_do_reg) {
		*tgt = *s2 & 0xffff; *len = 2; *cyc = 6;
		return K_DO;
	}
	return 0;
}

/* the block's code pages (versions) and external RAM ranges (self-modification) */
static const uint32_t *jit_npv_p[JIT_PV];
static int jit_npv;
static int jit_nr;
static uint32_t jit_rf[JIT_NR], jit_rl[JIT_NR];	/* address ranges of external code */

/* add the pages of P words f..l; 0 (nothing added) when full */
static int jit_add_pages(uint32_t f, uint32_t l)
{
	const uint32_t *p[2] = { jit_pver_of(f), jit_pver_of(l) };
	int add = 0, i, j;
	for (j = 0; j < 2; j++) {
		int have = 0;
		for (i = 0; i < jit_npv; i++) if (jit_npv_p[i] == p[j]) have = 1;
		if (j == 1 && p[1] == p[0]) have = 1;
		if (!have) add++;
	}
	if (jit_npv + add > JIT_PV) return 0;
	for (j = 0; j < 2; j++) {
		int have = 0;
		for (i = 0; i < jit_npv; i++) if (jit_npv_p[i] == p[j]) have = 1;
		if (!have) jit_npv_p[jit_npv++] = p[j];
	}
	return 1;
}

/* add P words f..l to the external code ranges; 0 when full */
static int jit_add_range(uint32_t f, uint32_t l)
{
	if (l < 0x200 || !jit_ramext) return 1;
	if (f < 0x200) f = 0x200;
	if (jit_nr && f == jit_rl[jit_nr - 1] + 1) { jit_rl[jit_nr - 1] = l; return 1; }
	if (jit_nr && f >= jit_rf[jit_nr - 1] && l <= jit_rl[jit_nr - 1]) return 1;
	if (jit_nr == JIT_NR) return 0;
	jit_rf[jit_nr] = f; jit_rl[jit_nr] = l; jit_nr++;
	return 1;
}

/* exit requests: a branch to a stub that returns count, pc, len */
static struct { uint32_t *br; uint16_t cnt, pc, len; uint8_t lz; } jit_xr[4 * JIT_MAX];
static int jit_nxr;

static void exit_if_lz(int cond, int cnt, uint32_t xpc, uint32_t xlen, int lz)
{
	if (jit_nxr >= 4 * JIT_MAX) { ep_overflow = 1; return; }
	jit_xr[jit_nxr].br = bcc(cond);
	jit_xr[jit_nxr].cnt = cnt; jit_xr[jit_nxr].pc = xpc; jit_xr[jit_nxr].len = xlen;
	jit_xr[jit_nxr].lz = lz;
	jit_nxr++;
}

static void exit_if(int cond, int cnt, uint32_t xpc, uint32_t xlen) { exit_if_lz(cond, cnt, xpc, xlen, 0); }

/* S_CNT += cnt; on to target unless the budget is spent (then exit as given) */
static void gen_back(uint32_t *target, int cnt, int xcnt, uint32_t xpc, uint32_t xlen)
{
	ldr(R2, SP, S_CNT);
	opi(OP_ADD, R2, R2, cnt);
	cmpi(R2, JIT_LOOP_BUDGET);
	exit_if(HS, xcnt, xpc, xlen);
	str(R2, SP, S_CNT);
	bto(AL, target);
}

/* the end of an inline DO loop with LC = 1 (dsp_postexecute_update_pc) */
static void jit_loopend(void)
{
	uint32_t saved_pc, saved_sr;
	dsp_stack_pop(&saved_pc, &saved_sr);
	dsp_core.registers[DSP_REG_SR] &= 0x7fff;
	dsp_core.registers[DSP_REG_SR] |= saved_sr & (1<<DSP_SR_LF);
	dsp_stack_pop(&dsp_core.registers[DSP_REG_LA], &dsp_core.registers[DSP_REG_LC]);
}

static void gen_cc(uint32_t cc)
{
	ldr(R0, R4, OFF_REG(DSP_REG_SR));
	dpi(AL, OP_AND, 0, R0, R0, 0x7f);
	mov32(IP, (uint32_t)(uintptr_t)jit_cc[cc & 15]);
	ldrb_x(R0, IP, R0);
	dpi(AL, OP_CMP, 1, 0, R0, 0);
}

static jit_block_t *jit_compile(uint16_t pc, uint16_t clip, int lf)
{
	jit_block_t *b;
	uint32_t *start, *body, *to_end[4], *lbl[JIT_MAX];
	int nend = 0;
	uint16_t ia[JIT_MAX], itg[JIT_MAX], epc[JIT_MAX], elen[JIT_MAX];
	uint8_t il[JIT_MAX], ik[JIT_MAX], icy[JIT_MAX], iback[JIT_MAX], icc[JIT_MAX];
	int8_t lend[JIT_MAX];
	uint32_t iw[JIT_MAX];
	uint32_t a = pc, cur_clip = clip, outer_clip = clip;
	int n = 0, i, ext_code, loop_open = -1, jumps_ok = !lf, has_jump = 0;

	if (jit_pool_n >= JIT_POOL || jit_ptr + 64 * 1024 > jit_mem + JIT_CODE_SIZE)
		jit_flush();
	b = &jit_pool[jit_pool_n++];
	b->start = pc;
	b->clip = clip;
	b->code = NULL;
	b->nw = 0;
	b->scal = jit_scal = (dsp_core.registers[DSP_REG_SR] >> DSP_SR_S0) & 3;
	dsp_jit_stats[3]++;
	jit_npv = 0;
	jit_nr = 0;
	jit_nxr = 0;
	jit_mmask = 0;
	for (i = 0; i < 8; i++) jit_m[i] = dsp_core.registers[DSP_REG_M0 + i];

	/* the instructions that stay in a block: straight runs, an inline DO
	   loop (body to LA, then on), and (no loop active) jumps followed to
	   their target */
	while (n < jit_max && a <= 0xfffe) {
		const uint32_t *s = jit_psrc(a);
		uint32_t w, tgt = 0;
		int k, len = 1, cyc = 2, cc = 0;
		if (!s) break;
		w = *s & BITMASK(24);
		k = jit_kind(w);
		if (k == K_TAIL) {
			int c = jit_ctl(w, a, &tgt, &len, &cyc, &cc);
			if (c == K_DO && !(loop_open < 0 && tgt >= a + 2 && (cur_clip == 0xffff || tgt < cur_clip) &&
			                   !(cur_clip >= a && cur_clip <= a + 1)))
				c = 0;
			if ((c == K_JMP || c == K_JCC) && !(jumps_ok && loop_open < 0 && cur_clip == 0xffff))
				c = 0;
			if (c) k = c;
			else len = 2;			/* a tail: count two words */
		} else if (k) {
			len = jit_len(w);
			if (len == 2 && !jit_psrc(a + 1)) break;
		}
		if (!k) break;
		if (!jit_add_pages(a, a + len - 1)) break;
		if (!jit_add_range(a, a + len - 1)) break;
		ia[n] = a; il[n] = len; ik[n] = k; iw[n] = w; itg[n] = tgt; icy[n] = cyc;
		iback[n] = 0; lend[n] = -1; icc[n] = cc;
		epc[n] = a; elen[n] = len;
		n++;
		if (k == K_TAIL) break;
		if (k == K_DO) {
			loop_open = n;
			outer_clip = cur_clip; cur_clip = tgt;
			a += 2;
			continue;
		}
		if (k == K_JMP) {
			has_jump = 1;
			epc[n - 1] = tgt; elen[n - 1] = 0;
			if (tgt == pc) { iback[n - 1] = 1; break; }
			a = tgt;
			continue;
		}
		if (k == K_JCC) {
			has_jump = 1;
			if (tgt == pc) iback[n - 1] = 1;
			a += len;
			continue;
		}
		if (cur_clip >= a && cur_clip < a + len) {
			if (loop_open >= 0 && loop_open < n && cur_clip == a + len - 1) {
				/* the inline loop's last instruction */
				lend[n - 1] = loop_open;
				loop_open = -1;
				cur_clip = outer_clip;
				a += len;
				continue;
			}
			break;
		}
		a += len;
	}
	/* a lone tail is cheaper through the interpreter (a polling loop) */
	if (n == 1 && ik[0] == K_TAIL) n = 0;
	b->n = n;
	b->last = n ? ia[n - 1] : pc;
	b->jumps = has_jump;
	b->mmask = 0;
	if (!n) { jit_npv = 0; jit_add_pages(pc, pc); }
	b->npv = jit_npv;
	for (i = 0; i < jit_npv; i++) { b->pv[i] = jit_npv_p[i]; b->ver[i] = *b->pv[i]; }
	if (!n) {
		const uint32_t *s = jit_psrc(pc);
		b->src[0] = s;
		b->word[0] = s ? *s & BITMASK(24) : 0;
		b->nw = 1;
		return b;
	}

	/* external RAM index ranges of the block's code */
	ext_code = jit_nr > 0;
	for (i = 0; i < jit_nr; i++) {
		jit_rlo[i] = jit_rf[i] & (jit_ramsize - 1);
		jit_rhi[i] = jit_rl[i] & (jit_ramsize - 1);
		if (jit_rhi[i] < jit_rlo[i] || jit_rl[i] - jit_rf[i] >= jit_ramsize) { jit_rlo[i] = 0; jit_rhi[i] = jit_ramsize - 1; }
	}
	jit_nrange = jit_nr;

	ep = start = (uint32_t *)jit_ptr;
	ep_end = (uint32_t *)(jit_mem + JIT_CODE_SIZE);
	ep_overflow = 0;
	emit(0xE92D4FF8);				/* push {r3-r11, lr} */
	dpi(AL, OP_SUB, 0, SP, SP, S_FRAME);
	mov32(R4, (uint32_t)(uintptr_t)&dsp_core);
	mov32(R6, (uint32_t)(uintptr_t)&dsp_core.ramint[DSP_SPACE_X][0]);
	mov32(R7, (uint32_t)(uintptr_t)&dsp_core.ramint[DSP_SPACE_Y][0]);
	ldr(R8, R4, offsetof(dsp_core_t, ramext));
	dpi(AL, OP_MOV, 0, R5, 0, 0);
	str(R5, SP, S_CNT);
	body = ep;

	jit_prev_agu = 0;
	for (i = 0; i < n; i++) {
		int lz = 0;
		lbl[i] = ep;
		jit_cur_agu = 0;
		/* this ALU op's flags are dead if the next instruction's ALU op
		   writes all of them (control always goes on to it, or exits) */
		if (ik[i] == K_NAT && iw[i] >= 0x100000 && i + 1 < n && ik[i + 1] == K_NAT && iw[i + 1] >= 0x100000 &&
		    lend[i] < 0 && !iback[i]) {
			uint32_t wi = alu_wmask(iw[i] & 0xff), wn = alu_wmask(iw[i + 1] & 0xff);
			if (alu_cls(iw[i] & 0xff) && wn && !(wi & ~wn)) lz = alu_cls(iw[i] & 0xff);
		}
		if (ik[i] == K_TAIL) {
			/* the tail: Hatari's handler; falls through to the end with
			   the PC and length it left in jit_out */
			mov32(R0, iw[i]);
			mov32(R1, ia[i]);
			call((const void *)jit_tail);
			dpr(AL, OP_MOV, 0, R10, 0, R0, 0, 0);
			dpr(AL, OP_ADD, 0, R5, R5, R10, 0, 0);
			jit_prev_agu = 0;
			break;
		}
		if (ik[i] == K_INT || ik[i] == K_DO) {
			mov32(R0, iw[i]);
			mov32(R1, ia[i]);
			call((const void *)jit_interp);
			ubfx(R10, R0, 0, 16);
			dpr(AL, OP_MOV, 0, R9, 0, R0, 1, 16);	/* lsr #16 */
			if (!ext_code) dpi(AL, OP_BIC, 0, R9, R9, JF_SMC);
		} else {
			if (jit_prev_agu) {
				/* the previous instruction's staged R/N move is spent:
				   reg[1] = 0, val[0] = val[1] (reg[0] stays 0, as after
				   the interpreter's next end of instruction) */
				dpi(AL, OP_MOV, 0, R0, 0, 0);
				strh_core(R0, offsetof(dsp_core_t, agu_pipeline_reg[1]));
				ldr(R0, SP, S_AGUV);
				strh_core(R0, offsetof(dsp_core_t, agu_pipeline_val[0]));
			}
			if (ik[i] == K_JMP || ik[i] == K_JCC) {
				dpi(AL, OP_MOV, 0, R10, 0, icy[i]);
			} else if (!jit_mem_inst(iw[i])) {
				jit_lazy = lz;
				gen_inst(iw[i]);
				jit_lazy = 0;
				dpi(AL, OP_MOV, 0, R10, 0, 2);
			} else {
				uint32_t *noext;
				dpi(AL, OP_MOV, 0, R9, 0, ia[i] >= 0x200 ? 4 : 0);	/* P fetch external */
				jit_lazy = lz;
				gen_inst(iw[i]);
				jit_lazy = 0;
				/* cycles: 2 + wait states (none without an external X/Y access) */
				dpi(AL, OP_MOV, 0, R10, 0, 2);
				dpi(AL, OP_TST, 1, 0, R9, 3);
				noext = bcc(EQ);
				dpi(AL, OP_AND, 0, R0, R9, 7);
				mov32(IP, (uint32_t)(uintptr_t)jit_wait);
				ldrb_x(R0, IP, R0);
				dpi(AL, OP_ADD, 0, R10, R0, 2);
				patch(noext, ep);
			}
		}
		dpr(AL, OP_ADD, 0, R5, R5, R10, 0, 0);
		if (jit_dbg) {
			mov32(R0, i);
			dpr(AL, OP_MOV, 0, R1, 0, R10, 0, 0);
			dpr(AL, OP_MOV, 0, R2, 0, R5, 0, 0);
			call((const void *)jit_dbg_rec);
		}
		if (ik[i] == K_JMP) {
			jit_prev_agu = 0;
			if (iback[i]) gen_back(body, i + 1, i + 1, epc[i], 0);
			continue;
		}
		if (ik[i] == K_JCC) {
			uint32_t *nt;
			jit_prev_agu = 0;
			gen_cc(icc[i]);
			if (iback[i]) {
				nt = bcc(EQ);
				gen_back(body, i + 1, i + 1, itg[i], 0);
				patch(nt, ep);
			} else {
				exit_if(NE, i + 1, itg[i], 0);
			}
			continue;
		}
		/* a peripheral or this block's code was touched, an AGU delay is
		   pending, or an interrupt was raised: end after this instruction
		   (only memory accesses and Hatari's handlers can) */
		if (lz && jit_dbg == 2) {
			exit_if_lz(AL, i + 1, epc[i], elen[i], lz);
		} else if (ik[i] != K_NAT || jit_mem_inst(iw[i])) {
			dpi(AL, OP_TST, 1, 0, R9, JF_EXIT);
			exit_if_lz(NE, i + 1, epc[i], elen[i], lz);
		}
		jit_prev_agu = ik[i] == K_NAT ? jit_cur_agu : 0;
		if (lend[i] >= 0) {
			/* inline DO loop end at LA: again while LC > 1, else pop */
			int ls = lend[i];
			if (jit_prev_agu) {
				exit_if(AL, i + 1, epc[i], elen[i]);
				continue;
			}
			ldr(R0, R4, OFF_REG(DSP_REG_LC));
			cmpi(R0, 1);
			{
				uint32_t *last = bcc(EQ);
				dpi(AL, OP_SUB, 0, R0, R0, 1);
				ubfx(R0, R0, 0, 16);
				/* (LC is stored only when the pass is taken) */
				ldr(R2, SP, S_CNT);
				opi(OP_ADD, R2, R2, i - ls + 1);
				cmpi(R2, JIT_LOOP_BUDGET);
				exit_if(HS, i + 1, epc[i], elen[i]);
				str(R2, SP, S_CNT);
				str(R0, R4, OFF_REG(DSP_REG_LC));
				bto(AL, lbl[ls]);
				patch(last, ep);
			}
			call((const void *)jit_loopend);
		}
	}
	/* the block is a DO loop's body, from its start (SSH) to LA: loop here
	   while LC > 1, as dsp_postexecute_update_pc would, up to a budget (LF
	   and LA were checked on entry; nothing in a block changes them, SSH or
	   LC) */
	if (clip != 0xffff && clip == ia[n - 1] + il[n - 1] - 1 && (ik[n - 1] == K_NAT || ik[n - 1] == K_INT) &&
	    !jit_prev_agu && lend[n - 1] < 0) {
		ldr(R0, R4, OFF_REG(DSP_REG_LC));
		cmpi(R0, 1);
		to_end[nend++] = bcc(EQ);
		ldr(R1, R4, OFF_REG(DSP_REG_SSH));
		cmpi(R1, pc);
		to_end[nend++] = bcc(NE);
		ldr(R2, SP, S_CNT);
		opi(OP_ADD, R2, R2, n);
		cmpi(R2, JIT_LOOP_BUDGET);
		to_end[nend++] = bcc(HS);
		str(R2, SP, S_CNT);
		dpi(AL, OP_SUB, 0, R0, R0, 1);
		ubfx(R0, R0, 0, 16);
		str(R0, R4, OFF_REG(DSP_REG_LC));
		bto(AL, body);
	}
	/* exits: r0 instructions run, r1 pc and r2 length for jit_post, r10
	   the last instruction's cycles */
	{
		uint32_t *to_epi[4 * JIT_MAX + 1], *epi;
		for (i = 0; i < nend; i++) patch(to_end[i], ep);
		ldr(R0, SP, S_CNT);
		opi(OP_ADD, R0, R0, n);
		if (ik[n - 1] == K_TAIL) {
			mov32(IP, (uint32_t)(uintptr_t)&jit_out);
			ldr(R1, IP, 4);
			ldr(R2, IP, 8);
		} else {
			mov32(R1, epc[n - 1]);
			mov32(R2, elen[n - 1]);
		}
		to_epi[jit_nxr] = bcc(AL);
		for (i = 0; i < jit_nxr; i++) {
			patch(jit_xr[i].br, ep);
			if (jit_xr[i].lz) gen_lazy_sr(jit_xr[i].lz);
			ldr(R0, SP, S_CNT);
			opi(OP_ADD, R0, R0, jit_xr[i].cnt);
			mov32(R1, jit_xr[i].pc);
			mov32(R2, jit_xr[i].len);
			to_epi[i] = bcc(AL);
		}
		epi = ep;
		for (i = 0; i <= jit_nxr; i++) patch(to_epi[i], epi);
		/* epilogue: jit_out = {n, pc, len, cycles}; instr_cycle = r10 */
		mov32(IP, (uint32_t)(uintptr_t)&jit_out);
		str(R0, IP, 0);
		str(R1, IP, 4);
		str(R2, IP, 8);
		str(R5, IP, 12);
		emit(0xE1C000B0 | R4 << 16 | R10 << 12 |
		     ((offsetof(dsp_core_t, instr_cycle) >> 4) & 0xf) << 8 | (offsetof(dsp_core_t, instr_cycle) & 0xf));	/* strh r10, [r4, #instr_cycle] */
		dpi(AL, OP_ADD, 0, SP, SP, S_FRAME);
		emit(0xE8BD8FF8);			/* pop {r3-r11, pc} */
	}
	if (ep_overflow) {
		jit_flush();
		return NULL;
	}
	__builtin___clear_cache((char *)start, (char *)ep);
	b->mmask = jit_mmask;
	for (i = 0; i < 8; i++) b->m[i] = jit_m[i];
	b->code = (void (*)(void))start;
	jit_ptr = (uint8_t *)ep;
	return b;
}

static int jit_m_ok(const jit_block_t *b)
{
	for (int i = 0; i < 8; i++)
		if ((b->mmask >> i & 1) && dsp_core.registers[DSP_REG_M0 + i] != b->m[i]) return 0;
	return 1;
}

static int jit_valid(const jit_block_t *b)
{
	for (int i = 0; i < b->npv; i++)
		if (*b->pv[i] != b->ver[i]) return 0;
	return 1;
}

/* drop every block (memory changed behind the core's back: tests) */
void dsp56k_jit_flush(void)
{
	if (jit_mem) jit_flush();
}

/* the interpreter's end of instruction, for a block's last instruction */
static void jit_post(void)
{
	dsp_core.agu_pipeline_reg[0] = 0;
	if (dsp_core.agu_pipeline_reg[1] != 0) {
		dsp_core.agu_pipeline_reg[0] = dsp_core.agu_pipeline_reg[1];
		dsp_core.agu_pipeline_val[0] = dsp_core.agu_pipeline_val[1];
		dsp_core.agu_pipeline_reg[1] = 0;
	}
	if (dsp_core.loop_rep == 0 &&
	    (!(dsp_core.registers[DSP_REG_SR] & (1<<DSP_SR_LF)) ||
	     (uint16_t)(dsp_core.pc + cur_inst_len) != dsp_core.registers[DSP_REG_LA] + 1))
		dsp_core.pc += cur_inst_len;
	else
		dsp_postexecute_update_pc();
	if (!(dsp_core.loop_rep == 0 && dsp_core.interrupt_state != DSP_INTERRUPT_DISABLED &&
	      !(dsp_core.interrupt_status & DSP_INTER_NMI_MASK) &&
	      !(dsp_core.interrupt_status & dsp_core.interrupt_enable & dsp_core.interrupt_mask)))
		dsp_postexecute_interrupts();
}

/* one block, or one interpreted instruction; returns the instructions run
   and adds their cycles to *cycles */
int dsp56k_jit_step(uint32_t *cycles)
{
	jit_block_t *b, **slot;
	uint16_t pc = dsp_core.pc;
	uint16_t la;
	int lf;

	if (dsp_core.mode == 1) {
		/* the bootstrap ROM loads internal P directly (dsp_core.c) */
		jit_saw_boot = 1;
		goto interp;
	}
	if (jit_saw_boot) {
		for (unsigned i = 0; i < sizeof(jit_iver) / sizeof(jit_iver[0]); i++) jit_iver[i]++;
		jit_saw_boot = 0;
	}
	if (jit_off || dsp_core.loop_rep || dsp_core.mode_wait ||
	    dsp_core.interrupt_state == DSP_INTERRUPT_DISABLED || dsp_core.agu_pipeline_reg[0] ||
	    (dsp_core.registers[DSP_REG_SR] & (1<<DSP_SR_T)) ||
	    /* an interrupt pending (raised by the host link since the last
	       check): the interpreter takes it after one instruction */
	    (dsp_core.interrupt_status & DSP_INTER_NMI_MASK) ||
	    (dsp_core.interrupt_status & dsp_core.interrupt_enable & dsp_core.interrupt_mask))
		goto interp;
	if (!jit_mem && jit_init()) goto interp;
	if (dsp_core.ramext != jit_ramext || DSP_RAMSIZE != jit_ramsize) jit_flush();

	lf = (dsp_core.registers[DSP_REG_SR] >> DSP_SR_LF) & 1;
	la = dsp_core.registers[DSP_REG_LA];
	slot = &jit_tab[JIT_IDX(pc)];
	b = *slot;
	if (!b || (b->start != pc && ++dsp_jit_stats[4]) || (!jit_valid(b) && ++dsp_jit_stats[5]) ||
	    (b->n && lf && la >= b->start && la < b->last && ++dsp_jit_stats[6]) ||
	    (b->clip != 0xffff && !(lf && la == b->clip) && ++dsp_jit_stats[7]) ||
	    (b->scal != ((dsp_core.registers[DSP_REG_SR] >> DSP_SR_S0) & 3) && ++dsp_jit_stats[5]) ||
	    (b->jumps && lf && ++dsp_jit_stats[7]) ||
	    (b->mmask && !jit_m_ok(b) && ++dsp_jit_stats[5])) {
		/* (a block clipped at an LA is kept only while that loop runs) */
		b = jit_compile(pc, lf && la >= pc ? la : 0xffff, lf);
		*slot = b;
		if (!b) goto interp;
	}
	if (!b->n) goto interp;

	if (jit_dbg) dsp_jit_dbg[2 * JIT_MAX + 3] = b->start;
	b->code();
	if (jit_dbg) { dsp_jit_dbg[2 * JIT_MAX] = jit_out.cycles; dsp_jit_dbg[2 * JIT_MAX + 1] = jit_out.n; dsp_jit_dbg[2 * JIT_MAX + 2] = jit_out.pc | jit_out.len << 16; }
	dsp_core.pc = jit_out.pc;
	cur_inst_len = jit_out.len;
	jit_post();
	*cycles += jit_out.cycles;
	dsp_jit_stats[0]++;
	dsp_jit_stats[1] += jit_out.n;
	return jit_out.n;

interp:
	dsp56k_execute_instruction();
	*cycles += dsp_core.instr_cycle;
	dsp_jit_stats[2]++;
	return 1;
}

#else

uint32_t dsp_jit_stats[8];
uint32_t dsp_jit_dbg[68];

void dsp56k_jit_flush(void) {}

int dsp56k_jit_step(uint32_t *cycles)
{
	dsp56k_execute_instruction();
	*cycles += dsp_core.instr_cycle;
	return 1;
}

#endif
