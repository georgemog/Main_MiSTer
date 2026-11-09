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

typedef struct jit_block_s {
	uint16_t start, last;		/* first and last instruction address */
	uint16_t n;			/* instructions; 0: the first one ends blocks */
	uint16_t clip;			/* compiled to stop at this LA (0xffff: none) */
	uint16_t nw;			/* P words */
	const uint32_t *pv[2];		/* code version counters of its pages */
	uint32_t ver[2];
	void (*code)(void);
	const uint32_t *src[2 * JIT_MAX];	/* where each P word came from */
	uint32_t word[2 * JIT_MAX];
} jit_block_t;

static struct { uint32_t n, pc, len, cycles; } jit_out;
static uint8_t *jit_mem, *jit_ptr;
static jit_block_t *jit_tab[JIT_HASH];
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
static uint32_t jit_lo, jit_hi;		/* external RAM index range of the block's code (lo > hi: none) */
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
#define JIT_LOOP_BUDGET	256	/* instructions before a looping block returns */

/* address of R[n] per ea mode (1 (Rn)+Nn, 0 (Rn)-Nn, 2 (Rn)-, 3 (Rn)+, 4 (Rn)) to slot */
static void gen_ea(uint32_t ea_mode, int slot)
{
	uint32_t mode = (ea_mode >> 3) & 7, n = ea_mode & 7;
	uint32_t *slow, *done;

	ldr(R10, R4, OFF_REG(DSP_REG_R0 + n));
	str(R10, SP, slot);
	if (mode == 4) return;
	ldr(R11, R4, OFF_REG(DSP_REG_M0 + n));
	cmpi(R11, 0xffff);
	slow = bcc(NE);
	switch (mode) {
	case 0: ldr(R1, R4, OFF_REG(DSP_REG_N0 + n)); dpr(AL, OP_SUB, 0, R0, R10, R1, 0, 0); break;
	case 1: ldr(R1, R4, OFF_REG(DSP_REG_N0 + n)); dpr(AL, OP_ADD, 0, R0, R10, R1, 0, 0); break;
	case 2: dpi(AL, OP_SUB, 0, R0, R10, 1); break;
	case 3: dpi(AL, OP_ADD, 0, R0, R10, 1); break;
	}
	ubfx(R0, R0, 0, 16);
	str(R0, R4, OFF_REG(DSP_REG_R0 + n));
	done = bcc(AL);
	patch(slow, ep);
	dpi(AL, OP_MOV, 0, R0, 0, n);
	dpr(AL, OP_MOV, 0, R1, 0, R10, 0, 0);
	switch (mode) {
	case 0: ldr(R2, R4, OFF_REG(DSP_REG_N0 + n)); dpi(AL, OP_RSB, 0, R2, R2, 0); break;
	case 1: ldr(R2, R4, OFF_REG(DSP_REG_N0 + n)); break;
	case 2: dpi(AL, OP_MVN, 0, R2, 0, 0); break;
	case 3: dpi(AL, OP_MOV, 0, R2, 0, 1); break;
	}
	dpr(AL, OP_MOV, 0, R3, 0, R11, 0, 0);
	call((const void *)jit_urn);
	patch(done, ep);
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
		if (jit_lo <= jit_hi) {
			cmpi(R11, jit_lo);
			{
				uint32_t *below = bcc(LO);
				cmpi(R11, jit_hi);
				dpi(LS, OP_ORR, 0, R9, R9, JF_SMC);
				patch(below, ep);
			}
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
	if (numreg == DSP_REG_A || numreg == DSP_REG_B) {
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
	} else {
		if (mask) dpi(AL, OP_BIC, 0, R0, R0, 0xff000000);
		str(R0, R4, OFF_REG(numreg));
	}
}

static void gen_alu(uint32_t inst)
{
	dsp_emul_t f = opcodes_alu[inst & 0xff];
	if (f != dsp_move) call((const void *)f);
}

static int jit_dst_ok(uint32_t r)  { return (r >= DSP_REG_X0 && r <= DSP_REG_Y1) || r == DSP_REG_A || r == DSP_REG_B; }
static int jit_src_ok(uint32_t r)  { return (r >= DSP_REG_X0 && r <= DSP_REG_B) || (r >= DSP_REG_R0 && r <= DSP_REG_N7); }

/* can this word be compiled? (mirrors the gen_inst cases) */
static int jit_native(uint32_t inst)
{
	if (inst < 0x100000) return 0;
	if (opcodes_alu[inst & 0xff] == dsp_undefined) return 0;
	switch ((inst >> 20) & 0xf) {
	case 2:
		if ((inst & 0xffff00) == 0x200000) return 1;
		if ((inst & 0xffe000) == 0x204000) return 0;
		if ((inst & 0xfc0000) == 0x200000)
			return jit_src_ok((inst >> 13) & 0x1f) && jit_dst_ok((inst >> 8) & 0x1f);
		return 0;
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

/* one instruction (after jit_native said yes) */
static void gen_inst(uint32_t inst)
{
	static const uint8_t xreg[4] = { DSP_REG_X0, DSP_REG_X1, DSP_REG_A, DSP_REG_B };
	static const uint8_t yreg[4] = { DSP_REG_Y0, DSP_REG_Y1, DSP_REG_A, DSP_REG_B };

	switch ((inst >> 20) & 0xf) {
	case 2:
		if ((inst & 0xffff00) == 0x200000) {
			gen_alu(inst);
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
	uint32_t value, f = 0;

	dsp_core.pc = addr;
	cur_inst = inst;
	cur_inst_len = 1;
	dsp_core.instr_cycle = 2;
	access_to_ext_memory = addr >= 0x200 ? 1 << DSP_SPACE_P : 0;
	dsp_core.agu_move_indirect_instr = 0;
	jit_periph = jit_extw = 0;
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
	if (jit_periph || dsp_core.agu_pipeline_reg[1]) f |= JF_PERIPH;
	if (jit_extw) f |= JF_SMC;
	return dsp_core.instr_cycle | f << 16;
}

/* 1: compiled; 2: through jit_interp inside a block; 0: ends blocks */
static int jit_kind(uint32_t inst)
{
	if (jit_native(inst)) return 1;
	if (inst == 0) return 2;				/* nop */
	if (inst < 0x100000) {
		uint32_t value = ((inst >> 11) & (BITMASK(6) << 3)) + ((inst >> 5) & BITMASK(3));
		return opcodes8h[value] == dsp_tcc ? 2 : 0;
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
	jit_mem = mmap(NULL, JIT_CODE_SIZE, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (jit_mem == MAP_FAILED) { jit_mem = NULL; jit_off = 1; return -1; }
	jit_pool = calloc(JIT_POOL, sizeof(jit_block_t));
	if (!jit_pool) { jit_off = 1; return -1; }
	jit_flush();
	return 0;
}

static jit_block_t *jit_compile(uint16_t pc, uint16_t clip)
{
	jit_block_t *b;
	uint32_t *exits[JIT_MAX], *start, *body, *to_end[4];
	int nend = 0;
	uint16_t ia[JIT_MAX];
	uint8_t il[JIT_MAX], ik[JIT_MAX];
	uint32_t iw[JIT_MAX];
	uint32_t a = pc;
	int n = 0, i, ext_code;

	if (jit_pool_n >= JIT_POOL || jit_ptr + 64 * 1024 > jit_mem + JIT_CODE_SIZE)
		jit_flush();
	b = &jit_pool[jit_pool_n++];
	b->start = pc;
	b->clip = clip;
	b->code = NULL;
	b->nw = 0;
	dsp_jit_stats[3]++;

	/* the run of instructions that stay in a block */
	while (n < jit_max && a <= 0xfffe) {
		const uint32_t *s = jit_psrc(a), *s2 = NULL;
		uint32_t w;
		int k, len;
		if (!s) break;
		w = *s & BITMASK(24);
		k = jit_kind(w);
		if (!k) break;
		len = jit_len(w);
		if (len == 2 && !(s2 = jit_psrc(a + 1))) break;
		b->src[b->nw] = s; b->word[b->nw++] = w;
		if (s2) { b->src[b->nw] = s2; b->word[b->nw++] = *s2 & BITMASK(24); }
		ia[n] = a; il[n] = len; ik[n] = k; iw[n] = w;
		n++;
		if (clip >= a && clip < a + len) break;
		a += len;
	}
	b->n = n;
	b->last = n ? ia[n - 1] : pc;
	/* pages: the first and last word's (a block spans at most two) */
	{
		uint32_t endw = n ? ia[n - 1] + il[n - 1] - 1 : pc;
		b->pv[0] = jit_pver_of(pc);
		b->pv[1] = jit_pver_of(endw);
		b->ver[0] = *b->pv[0];
		b->ver[1] = *b->pv[1];
	}
	if (!n) {
		const uint32_t *s = jit_psrc(pc);
		b->src[0] = s;
		b->word[0] = s ? *s & BITMASK(24) : 0;
		b->nw = 1;
		return b;
	}

	/* external RAM indices of the block's code */
	jit_lo = 1; jit_hi = 0;
	ext_code = jit_ramext && a > 0x200;
	if (ext_code) {
		uint32_t f = pc < 0x200 ? 0x200 : pc, l = ia[n - 1] + il[n - 1] - 1;
		jit_lo = f & (jit_ramsize - 1);
		jit_hi = l & (jit_ramsize - 1);
		if (jit_hi < jit_lo) { jit_lo = 0; jit_hi = jit_ramsize - 1; }
	}
	ep = start = (uint32_t *)jit_ptr;
	ep_end = (uint32_t *)(jit_mem + JIT_CODE_SIZE);
	ep_overflow = 0;
	emit(0xE92D4FF8);				/* push {r3-r11, lr} */
	dpi(AL, OP_SUB, 0, SP, SP, 32);
	mov32(R4, (uint32_t)(uintptr_t)&dsp_core);
	mov32(R6, (uint32_t)(uintptr_t)&dsp_core.ramint[DSP_SPACE_X][0]);
	mov32(R7, (uint32_t)(uintptr_t)&dsp_core.ramint[DSP_SPACE_Y][0]);
	ldr(R8, R4, offsetof(dsp_core_t, ramext));
	dpi(AL, OP_MOV, 0, R5, 0, 0);
	str(R5, SP, S_CNT);
	body = ep;

	for (i = 0; i < n; i++) {
		if (ik[i] == 2) {
			mov32(R0, iw[i]);
			mov32(R1, ia[i]);
			call((const void *)jit_interp);
			ubfx(R10, R0, 0, 16);
			dpr(AL, OP_MOV, 0, R9, 0, R0, 1, 16);	/* lsr #16 */
			if (!ext_code) dpi(AL, OP_BIC, 0, R9, R9, JF_SMC);
		} else {
			dpi(AL, OP_MOV, 0, R9, 0, ia[i] >= 0x200 ? 4 : 0);	/* P fetch external */
			gen_inst(iw[i]);
			/* cycles: 2 + wait states */
			dpi(AL, OP_AND, 0, R0, R9, 7);
			mov32(IP, (uint32_t)(uintptr_t)jit_wait);
			ldrb_x(R0, IP, R0);
			dpi(AL, OP_ADD, 0, R10, R0, 2);
		}
		dpr(AL, OP_ADD, 0, R5, R5, R10, 0, 0);
		/* a peripheral or this block's code was touched, or an AGU delay is
		   pending: end after this instruction */
		dpi(AL, OP_TST, 1, 0, R9, JF_EXIT);
		exits[i] = bcc(NE);
	}
	/* the block is a DO loop's body, from its start (SSH) to LA: loop here
	   while LC > 1, as dsp_postexecute_update_pc would, up to a budget (LF
	   and LA were checked on entry; nothing in a block changes them, SSH or
	   LC) */
	if (clip != 0xffff && clip == ia[n - 1] + il[n - 1] - 1) {
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
	/* exits: r0 instructions run, r1 last instruction, r2 its words, r10
	   its cycles */
	{
		uint32_t *to_epi[JIT_MAX + 1], *epi;
		for (i = 0; i < nend; i++) patch(to_end[i], ep);
		ldr(R0, SP, S_CNT);
		opi(OP_ADD, R0, R0, n);
		mov32(R1, ia[n - 1]);
		mov32(R2, il[n - 1]);
		to_epi[n] = bcc(AL);
		for (i = 0; i < n; i++) {
			patch(exits[i], ep);
			ldr(R0, SP, S_CNT);
			opi(OP_ADD, R0, R0, i + 1);
			mov32(R1, ia[i]);
			mov32(R2, il[i]);
			to_epi[i] = bcc(AL);
		}
		epi = ep;
		for (i = 0; i <= n; i++) patch(to_epi[i], epi);
		/* epilogue: jit_out = {n, pc, len, cycles}; instr_cycle = r10 */
		mov32(IP, (uint32_t)(uintptr_t)&jit_out);
		str(R0, IP, 0);
		str(R1, IP, 4);
		str(R2, IP, 8);
		str(R5, IP, 12);
		emit(0xE1C000B0 | R4 << 16 | R10 << 12 |
		     ((offsetof(dsp_core_t, instr_cycle) >> 4) & 0xf) << 8 | (offsetof(dsp_core_t, instr_cycle) & 0xf));	/* strh r10, [r4, #instr_cycle] */
		dpi(AL, OP_ADD, 0, SP, SP, 32);
		emit(0xE8BD8FF8);			/* pop {r3-r11, pc} */
	}
	if (ep_overflow) {
		jit_flush();
		return NULL;
	}
	__builtin___clear_cache((char *)start, (char *)ep);
	b->code = (void (*)(void))start;
	jit_ptr = (uint8_t *)ep;
	return b;
}

static int jit_valid(const jit_block_t *b)
{
	return *b->pv[0] == b->ver[0] && *b->pv[1] == b->ver[1];
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
	    (b->clip != 0xffff && !(lf && la == b->clip) && ++dsp_jit_stats[7])) {
		/* (a block clipped at an LA is kept only while that loop runs) */
		b = jit_compile(pc, lf && la >= pc ? la : 0xffff);
		*slot = b;
		if (!b) goto interp;
	}
	if (!b->n) goto interp;

	b->code();
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

void dsp56k_jit_flush(void) {}

int dsp56k_jit_step(uint32_t *cycles)
{
	dsp56k_execute_instruction();
	*cycles += dsp_core.instr_cycle;
	return 1;
}

#endif
