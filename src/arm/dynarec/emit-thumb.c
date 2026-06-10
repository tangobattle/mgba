/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "dynarec-internal.h"

#ifdef ENABLE_DYNAREC

#include <mgba/internal/arm/isa-thumb.h>

#include "third-party/sljit/sljit_src/sljitLir.h"

// Native Thumb codegen. Every sequence here must be bit-exact with the
// corresponding handler in src/arm/isa-thumb.c, including which CPSR bits
// each instruction class touches:
//   THUMB_ADDITION_S/THUMB_SUBTRACTION_S zero the whole flags byte first
//     (bits 24-31), then set NZCV            -> keep mask 0x00FFFFFF
//   shifts assign C and then NZ              -> keep mask 0x1FFFFFFF
//   THUMB_NEUTRAL_S assigns only NZ          -> keep mask 0x3FFFFFFF
// All instructions emitted here cost exactly THUMB_PREFETCH_CYCLES, which
// is accounted via the context's pending-cycle counter.

#define OFF_GPR(R) ((sljit_sw) offsetof(struct ARMCore, gprs) + (R) * (sljit_sw) sizeof(int32_t))
#define OFF_CPSR ((sljit_sw) offsetof(struct ARMCore, cpsr))
#ifdef __BIG_ENDIAN__
#define OFF_CPSR_FLAGS OFF_CPSR
#else
#define OFF_CPSR_FLAGS (OFF_CPSR + 3)
#endif

#define KEEP_NZCV 0x00FFFFFF
#define KEEP_NZC 0x1FFFFFFF
#define KEEP_NZ 0x3FFFFFFF

enum ThumbALUOperand {
	OPERAND_REG,
	OPERAND_IMM,
};

static void _loadReg(struct sljit_compiler* c, sljit_s32 dst, int reg) {
	sljit_emit_op1(c, SLJIT_MOV32, dst, 0, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg));
}

static void _storeReg(struct sljit_compiler* c, int reg, sljit_s32 src) {
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg), src, 0);
}

// Merges flag bits accumulated in R3 into cpsr: cpsr = (cpsr & keep) | R3
static void _mergeFlags(struct sljit_compiler* c, sljit_sw keep) {
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, keep);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_R3, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_CPSR, SLJIT_R4, 0);
}

// Appends Z and N (sign of R2) to R3; must be emitted IMMEDIATELY after the
// op that set SLJIT_SET_Z (sljit only allows sljit_emit_op_flags directly
// after its flag-setting op, before any other op2 resets the tracked flags)
static void _appendNZ(struct sljit_compiler* c) {
	sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_EQUAL);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 30);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R2, 0, SLJIT_IMM, (sljit_sw) 0x80000000);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
}

// rd (optional, -1 to skip) = M op N with full NZCV; operands may be cache
// registers (read-only). Each sljit_emit_op_flags immediately follows its
// flag-setting op.
static void _emitArith(struct ARMDynarecEmitContext* ctx, bool isSub, int rd, sljit_s32 m, sljit_sw mw, sljit_s32 n,
                       sljit_sw nw) {
	struct sljit_compiler* c = ctx->compiler;
	sljit_s32 op = isSub ? SLJIT_SUB32 : SLJIT_ADD32;
	// SLJIT_SET() takes the even member of a condition pair; op_flags may
	// then extract either member. C on subtraction is "no borrow", i.e.
	// unsigned greater-or-equal.
	sljit_s32 carryFlag = isSub ? SLJIT_SET(SLJIT_LESS) : SLJIT_SET_CARRY;
	sljit_s32 carryType = isSub ? SLJIT_GREATER_EQUAL : SLJIT_CARRY;

	sljit_emit_op2(c, op | carryFlag | SLJIT_SET_Z, SLJIT_R2, 0, m, mw, n, nw);
	sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R3, 0, carryType);
	sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_EQUAL);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 29);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 30);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
	sljit_emit_op2u(c, op | SLJIT_SET(SLJIT_OVERFLOW), m, mw, n, nw);
	sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_OVERFLOW);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 28);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R2, 0, SLJIT_IMM, (sljit_sw) 0x80000000);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
	if (rd >= 0) {
		ARMDynarecCacheWrite(ctx, rd, SLJIT_R2);
	}
	_mergeFlags(c, KEEP_NZCV);
	++ctx->pendingCycles;
}

// rd (optional) = result of a logical op already emitted into R2 with SET_Z;
// must be called immediately after that op (see _appendNZ)
static void _emitLogicalTail(struct ARMDynarecEmitContext* ctx, int rd) {
	struct sljit_compiler* c = ctx->compiler;
	sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R3, 0, SLJIT_EQUAL);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 30);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R2, 0, SLJIT_IMM, (sljit_sw) 0x80000000);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
	if (rd >= 0) {
		ARMDynarecCacheWrite(ctx, rd, SLJIT_R2);
	}
	_mergeFlags(c, KEEP_NZ);
	++ctx->pendingCycles;
}

static bool _emitShiftImm(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn) {
	struct sljit_compiler* c = ctx->compiler;
	int kind = (insn->opcode >> 11) & 3; // 0 LSL, 1 LSR, 2 ASR
	int immediate = (insn->opcode >> 6) & 0x1F;
	int rd = insn->opcode & 7;
	int rm = (insn->opcode >> 3) & 7;

	sljit_s32 src = ARMDynarecCacheRead(ctx, rm, SLJIT_R0);
	if (!immediate) {
		switch (kind) {
		case 0: // LSL #0: rd = rm, NZ only
			sljit_emit_op2(c, SLJIT_OR32 | SLJIT_SET_Z, SLJIT_R2, 0, src, 0, src, 0);
			_emitLogicalTail(ctx, rd);
			return true;
		case 1: // LSR #32: C = sign(rm), rd = 0
			sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, src, 0, SLJIT_IMM, 31);
			sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 29);
			// Z is constant: result is always 0
			sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 0x40000000);
			ARMDynarecCacheWriteImm(ctx, rd, 0);
			_mergeFlags(c, KEEP_NZC);
			++ctx->pendingCycles;
			return true;
		case 2: // ASR #32: rd = sign-fill, C = sign(rm)
			sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, src, 0, SLJIT_IMM, 31);
			sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 29);
			sljit_emit_op2(c, SLJIT_ASHR32 | SLJIT_SET_Z, SLJIT_R2, 0, src, 0, SLJIT_IMM, 31);
			_appendNZ(c);
			ARMDynarecCacheWrite(ctx, rd, SLJIT_R2);
			_mergeFlags(c, KEEP_NZC);
			++ctx->pendingCycles;
			return true;
		default:
			return false;
		}
	}

	// C = bit shifted out
	int carryShift = kind == 0 ? 32 - immediate : immediate - 1;
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, src, 0, SLJIT_IMM, carryShift);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 1);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 29);
	sljit_s32 shiftOp = kind == 0 ? SLJIT_SHL32 : kind == 1 ? SLJIT_LSHR32 : SLJIT_ASHR32;
	sljit_emit_op2(c, shiftOp | SLJIT_SET_Z, SLJIT_R2, 0, src, 0, SLJIT_IMM, immediate);
	_appendNZ(c);
	ARMDynarecCacheWrite(ctx, rd, SLJIT_R2);
	_mergeFlags(c, KEEP_NZC);
	++ctx->pendingCycles;
	return true;
}

static bool _emitALU(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn) {
	struct sljit_compiler* c = ctx->compiler;
	int op = (insn->opcode >> 6) & 0xF;
	int rd = insn->opcode & 7;
	int rn = (insn->opcode >> 3) & 7;

	switch (op) {
	case 0x0: // AND
	case 0x1: // EOR
	case 0xC: // ORR
	case 0xE: { // BIC
		sljit_s32 a = ARMDynarecCacheRead(ctx, rd, SLJIT_R0);
		sljit_s32 b = ARMDynarecCacheRead(ctx, rn, SLJIT_R1);
		sljit_s32 sop;
		if (op == 0xE) {
			sljit_emit_op2(c, SLJIT_XOR32, SLJIT_R1, 0, b, 0, SLJIT_IMM, -1);
			b = SLJIT_R1;
			sop = SLJIT_AND32;
		} else {
			sop = op == 0x0 ? SLJIT_AND32 : op == 0x1 ? SLJIT_XOR32 : SLJIT_OR32;
		}
		sljit_emit_op2(c, sop | SLJIT_SET_Z, SLJIT_R2, 0, a, 0, b, 0);
		_emitLogicalTail(ctx, rd);
		return true;
	}
	case 0x8: { // TST
		sljit_s32 a = ARMDynarecCacheRead(ctx, rd, SLJIT_R0);
		sljit_s32 b = ARMDynarecCacheRead(ctx, rn, SLJIT_R1);
		sljit_emit_op2(c, SLJIT_AND32 | SLJIT_SET_Z, SLJIT_R2, 0, a, 0, b, 0);
		_emitLogicalTail(ctx, -1);
		return true;
	}
	case 0xF: { // MVN
		sljit_s32 b = ARMDynarecCacheRead(ctx, rn, SLJIT_R0);
		sljit_emit_op2(c, SLJIT_XOR32 | SLJIT_SET_Z, SLJIT_R2, 0, b, 0, SLJIT_IMM, -1);
		_emitLogicalTail(ctx, rd);
		return true;
	}
	case 0x9: // NEG: rd = 0 - rn
		_emitArith(ctx, true, rd, SLJIT_IMM, 0, ARMDynarecCacheRead(ctx, rn, SLJIT_R1), 0);
		return true;
	case 0xA: // CMP
		_emitArith(ctx, true, -1, ARMDynarecCacheRead(ctx, rd, SLJIT_R0), 0, ARMDynarecCacheRead(ctx, rn, SLJIT_R1),
		           0);
		return true;
	case 0xB: // CMN
		_emitArith(ctx, false, -1, ARMDynarecCacheRead(ctx, rd, SLJIT_R0), 0, ARMDynarecCacheRead(ctx, rn, SLJIT_R1),
		           0);
		return true;
	default:
		// LSL2/LSR2/ASR2/ROR (register shifts: extra cycle, >=32 semantics),
		// ADC/SBC (carry-in), MUL (data-dependent cycles) stay callouts
		return false;
	}
}

#define OFF_CYCLES ((sljit_sw) offsetof(struct ARMCore, cycles))
#define OFF_NEXT_EVENT ((sljit_sw) offsetof(struct ARMCore, nextEvent))
#define OFF_NONSEQ16 ((sljit_sw) offsetof(struct ARMCore, memory.activeNonseqCycles16))
#define OFF_SEQ16_MEM ((sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles16))
#define OFF_PREFETCH0 ((sljit_sw) offsetof(struct ARMCore, prefetch))
#define OFF_PREFETCH1 ((sljit_sw) offsetof(struct ARMCore, prefetch) + (sljit_sw) sizeof(uint32_t))

// Taken branch to an instruction inside this block: account the cycles
// ThumbWritePC would (prefetch + 2N+S refill), apply the platform's
// taken-branch side effect, and stay in compiled code unless the event
// deadline was reached, in which case exit with the target's pipeline
// state materialized. Outstanding cycles must already be flushed.
static void _emitIntraBranchTaken(struct ARMDynarecEmitContext* ctx, size_t targetIdx) {
	struct sljit_compiler* c = ctx->compiler;
	// cycles += S1 + 2 + activeNonseqCycles16 + activeSeqCycles16
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_NONSEQ16);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_S0), OFF_SEQ16_MEM);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R1, 0);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_S1, 0);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 2);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES, SLJIT_R0, 0);
	if (ctx->cpu->dynarec->branchPrefetchReset) {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM0(), (sljit_sw) ctx->cpu->dynarec->branchPrefetchReset, SLJIT_IMM, 0);
	}
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES);
	struct sljit_jump* stay =
	    sljit_emit_cmp(c, SLJIT_SIG_LESS | SLJIT_32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_NEXT_EVENT);
	ctx->intraJumps[ctx->nIntra] = stay;
	ctx->intraTargets[ctx->nIntra] = targetIdx;
	++ctx->nIntra;
	// Deadline reached: exit with the between-instruction pipeline state of
	// the branch target
	uint32_t target = ctx->firstAddress + targetIdx * WORD_SIZE_THUMB;
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(ARM_PC), SLJIT_IMM,
	               (sljit_sw) target + WORD_SIZE_THUMB);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM,
	               (sljit_sw) ARMDynarecSourceWord(ctx, targetIdx));
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM,
	               (sljit_sw) ARMDynarecSourceWord(ctx, targetIdx + 1));
	ctx->exits[ctx->nExits] = sljit_emit_jump(c, SLJIT_JUMP);
	++ctx->nExits;
}

// Returns the in-block instruction index of a branch target, or -1
static int _intraTargetIndex(const struct ARMDynarecEmitContext* ctx, uint32_t target) {
	if (target < ctx->firstAddress || target >= ctx->firstAddress + ctx->count * WORD_SIZE_THUMB) {
		return -1;
	}
	return (target - ctx->firstAddress) >> 1;
}

// Conditional branch: test the condition natively; the not-taken path costs
// only prefetch cycles and stays inside the block, the taken path either
// jumps within the block or becomes a handler callout that exits
static void _emitConditionalBranch(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn,
                                   uint32_t prefetch0, uint32_t prefetch1) {
	struct sljit_compiler* c = ctx->compiler;
	unsigned condition = (insn->opcode >> 8) & 0xF;
	// Both paths need prior native cycles settled and memory-correct gprs
	ARMDynarecEmitFlushCycles(ctx);
	ARMDynarecCacheFlushDirty(ctx);
	sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR_FLAGS);
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 4);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, ARMDynarecConditionLut[condition]);
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
	struct sljit_jump* notTaken =
	    sljit_emit_op2cmpz(c, SLJIT_AND32 | SLJIT_JUMP_IF_ZERO, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_IMM, 1);

	int targetIdx = _intraTargetIndex(ctx, insn->address + 4 + (((int32_t)(int8_t)(insn->opcode & 0xFF)) << 1));
	if (targetIdx >= 0) {
		_emitIntraBranchTaken(ctx, targetIdx);
		sljit_set_label(notTaken, sljit_emit_label(c));
		++ctx->pendingCycles;
		return;
	}

	// Taken: run the real handler (it re-tests the condition and performs
	// ThumbWritePC with its setActiveRegion/prefetch-unit side effects)
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(ARM_PC), SLJIT_IMM,
	               (sljit_sw) insn->address + 2 * WORD_SIZE_THUMB);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, prefetch), SLJIT_IMM,
	               (sljit_sw) prefetch0);
	sljit_emit_op1(c, SLJIT_MOV32,
	               SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, prefetch) + (sljit_sw) sizeof(uint32_t),
	               SLJIT_IMM, (sljit_sw) prefetch1);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, (sljit_sw) insn->opcode);
	sljit_emit_icall(c, SLJIT_CALL, SLJIT_ARGS2V(P, 32), SLJIT_IMM, (sljit_sw) _thumbTable[insn->opcode >> 6]);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0),
	               (sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles16));
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);
	// The condition held, so the handler branched: PC is statically known
	ARMDynarecEmitLinkExit(ctx, insn->address + 4 + (((int32_t)(int8_t)(insn->opcode & 0xFF)) << 1));

	struct sljit_label* notTakenLabel = sljit_emit_label(c);
	sljit_set_label(notTaken, notTakenLabel);
	++ctx->pendingCycles;
}

// Computes the guest address into R1 (32-bit) and dispatches to the
// platform memory emitter. Caller must have verified the hook is set.
static bool _emitMemory(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn, uint32_t prefetch0,
                        uint32_t prefetch1) {
	struct sljit_compiler* c = ctx->compiler;
	uint32_t opcode = insn->opcode;
	enum ARMDynarecMemOp op;
	int rd;

	if ((opcode & 0xF800) == 0x4800) {
		// LDR3: PC-relative literal; address is a compile-time constant
		rd = (opcode >> 8) & 7;
		uint32_t address = ((insn->address + 4) & ~3) + ((opcode & 0xFF) << 2);
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, (sljit_sw) address);
		op = ARM_DYNAREC_MEM_LOAD32;
	} else if ((opcode & 0xF000) == 0x5000) {
		// Register-offset loads/stores
		static const enum ARMDynarecMemOp ops[8] = {
			ARM_DYNAREC_MEM_STORE32, ARM_DYNAREC_MEM_STORE16, ARM_DYNAREC_MEM_STORE8, ARM_DYNAREC_MEM_LOADS8,
			ARM_DYNAREC_MEM_LOAD32,  ARM_DYNAREC_MEM_LOAD16,  ARM_DYNAREC_MEM_LOAD8,  ARM_DYNAREC_MEM_LOADS16,
		};
		op = ops[(opcode >> 9) & 7];
		rd = opcode & 7;
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R1, 0, ARMDynarecCacheRead(ctx, (opcode >> 3) & 7, SLJIT_R0), 0,
		               ARMDynarecCacheRead(ctx, (opcode >> 6) & 7, SLJIT_R1), 0);
	} else if ((opcode & 0xE000) == 0x6000) {
		// Immediate-offset word/byte
		rd = opcode & 7;
		int immediate = (opcode >> 6) & 0x1F;
		if (opcode & 0x1000) {
			op = opcode & 0x0800 ? ARM_DYNAREC_MEM_LOAD8 : ARM_DYNAREC_MEM_STORE8;
		} else {
			op = opcode & 0x0800 ? ARM_DYNAREC_MEM_LOAD32 : ARM_DYNAREC_MEM_STORE32;
			immediate <<= 2;
		}
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R1, 0, ARMDynarecCacheRead(ctx, (opcode >> 3) & 7, SLJIT_R1), 0,
		               SLJIT_IMM, immediate);
	} else if ((opcode & 0xF000) == 0x8000) {
		// Immediate-offset halfword
		rd = opcode & 7;
		op = opcode & 0x0800 ? ARM_DYNAREC_MEM_LOAD16 : ARM_DYNAREC_MEM_STORE16;
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R1, 0, ARMDynarecCacheRead(ctx, (opcode >> 3) & 7, SLJIT_R1), 0,
		               SLJIT_IMM, ((opcode >> 6) & 0x1F) << 1);
	} else if ((opcode & 0xF000) == 0x9000) {
		// SP-relative word
		rd = (opcode >> 8) & 7;
		op = opcode & 0x0800 ? ARM_DYNAREC_MEM_LOAD32 : ARM_DYNAREC_MEM_STORE32;
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R1, 0, ARMDynarecCacheRead(ctx, ARM_SP, SLJIT_R1), 0, SLJIT_IMM,
		               (opcode & 0xFF) << 2);
	} else {
		return false;
	}

	ctx->cpu->dynarec->emitMemory(ctx->cpu, ctx, op, rd, false, insn->address, prefetch0, prefetch1);
	return true;
}

bool ARMDynarecEmitThumbNative(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn,
                               uint32_t prefetch0, uint32_t prefetch1) {
	struct sljit_compiler* c = ctx->compiler;
	uint32_t opcode = insn->opcode;

	switch (opcode >> 13) {
	case 0:
		if ((opcode & 0x1800) != 0x1800) {
			// Format 1: LSL/LSR/ASR rd, rm, #imm5
			return _emitShiftImm(ctx, insn);
		}
		// Format 2: ADD/SUB rd, rn, rm/#imm3
		{
			int rd = opcode & 7;
			int rn = (opcode >> 3) & 7;
			bool isSub = opcode & 0x0200;
			sljit_s32 m = ARMDynarecCacheRead(ctx, rn, SLJIT_R0);
			if (opcode & 0x0400) {
				_emitArith(ctx, isSub, rd, m, 0, SLJIT_IMM, (opcode >> 6) & 7);
			} else {
				_emitArith(ctx, isSub, rd, m, 0, ARMDynarecCacheRead(ctx, (opcode >> 6) & 7, SLJIT_R1), 0);
			}
		}
		return true;
	case 1: {
		// Format 3: MOV/CMP/ADD/SUB rd, #imm8
		int rd = (opcode >> 8) & 7;
		sljit_sw immediate = opcode & 0xFF;
		switch ((opcode >> 11) & 3) {
		case 0: // MOV: N = 0, Z = !imm, both compile-time constants
			ARMDynarecCacheWriteImm(ctx, rd, immediate);
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R3, 0, SLJIT_IMM, immediate ? 0 : 0x40000000);
			_mergeFlags(c, KEEP_NZ);
			++ctx->pendingCycles;
			return true;
		case 1: // CMP
			_emitArith(ctx, true, -1, ARMDynarecCacheRead(ctx, rd, SLJIT_R0), 0, SLJIT_IMM, immediate);
			return true;
		case 2: // ADD
			_emitArith(ctx, false, rd, ARMDynarecCacheRead(ctx, rd, SLJIT_R0), 0, SLJIT_IMM, immediate);
			return true;
		case 3: // SUB
			_emitArith(ctx, true, rd, ARMDynarecCacheRead(ctx, rd, SLJIT_R0), 0, SLJIT_IMM, immediate);
			return true;
		}
		return false;
	}
	case 2:
		if ((opcode & 0xFC00) == 0x4000) {
			// Format 4: register ALU ops
			return _emitALU(ctx, insn);
		}
		if (((opcode & 0xF800) == 0x4800 || (opcode & 0xF000) == 0x5000) && ctx->cpu->dynarec->emitMemory) {
			return _emitMemory(ctx, insn, prefetch0, prefetch1);
		}
		if ((opcode & 0xFC00) == 0x4400) {
			// Format 5: hi-register ADD/CMP/MOV (BX falls through to callout)
			int op = (opcode >> 8) & 3;
			int rd = (opcode & 7) | ((opcode >> 4) & 8);
			int rm = ((opcode >> 3) & 7) | ((opcode >> 3) & 8);
			if (op == 3) {
				return false; // BX
			}
			if (op != 1 && rd == ARM_PC) {
				return false; // ADD/MOV to PC branches; keep as callout
			}
			// PC reads observe the pipeline: instruction address + 4
			sljit_sw pcValue = (sljit_sw) insn->address + 4;
			switch (op) {
			case 0: { // ADD rd, rm (no flags)
				sljit_s32 a = ARMDynarecCacheRead(ctx, rd, SLJIT_R0);
				if (rm == ARM_PC) {
					sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, a, 0, SLJIT_IMM, pcValue);
				} else {
					sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, a, 0,
					               ARMDynarecCacheRead(ctx, rm, SLJIT_R1), 0);
				}
				ARMDynarecCacheWrite(ctx, rd, SLJIT_R0);
				++ctx->pendingCycles;
				return true;
			}
			case 1: { // CMP rd, rm
				sljit_s32 a;
				if (rd == ARM_PC) {
					sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_IMM, pcValue);
					a = SLJIT_R0;
				} else {
					a = ARMDynarecCacheRead(ctx, rd, SLJIT_R0);
				}
				if (rm == ARM_PC) {
					_emitArith(ctx, true, -1, a, 0, SLJIT_IMM, pcValue);
				} else {
					_emitArith(ctx, true, -1, a, 0, ARMDynarecCacheRead(ctx, rm, SLJIT_R1), 0);
				}
				return true;
			}
			case 2: // MOV rd, rm (no flags)
				if (rm == ARM_PC) {
					ARMDynarecCacheWriteImm(ctx, rd, pcValue);
				} else {
					sljit_s32 b = ARMDynarecCacheRead(ctx, rm, SLJIT_R0);
					ARMDynarecCacheWrite(ctx, rd, b);
				}
				++ctx->pendingCycles;
				return true;
			}
		}
		return false;
	case 3:
	case 4:
		// Immediate-offset and SP-relative loads/stores
		if (ctx->cpu->dynarec->emitMemory) {
			return _emitMemory(ctx, insn, prefetch0, prefetch1);
		}
		return false;
	case 5:
		if ((opcode & 0xF000) == 0xA000) {
			// ADD rd, PC/SP, #imm8 << 2 (no flags)
			int rd = (opcode >> 8) & 7;
			sljit_sw immediate = (opcode & 0xFF) << 2;
			if (opcode & 0x0800) {
				// ADD6: SP-relative
				sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, ARMDynarecCacheRead(ctx, ARM_SP, SLJIT_R0), 0,
				               SLJIT_IMM, immediate);
				ARMDynarecCacheWrite(ctx, rd, SLJIT_R0);
			} else {
				// ADD5: PC-relative, fully constant
				ARMDynarecCacheWriteImm(ctx, rd, (((sljit_sw) insn->address + 4) & ~3) + immediate);
			}
			++ctx->pendingCycles;
			return true;
		}
		if ((opcode & 0xF600) == 0xB400 && ctx->cpu->dynarec->emitMultiple) {
			// PUSH (0xB400/B500) / POP (0xBC00); POP with PC stays a callout
			bool isLoad = opcode & 0x0800;
			uint32_t mask = opcode & 0xFF;
			if (isLoad && (opcode & 0x0100)) {
				return false; // POPR: loads PC
			}
			if (!isLoad && (opcode & 0x0100)) {
				mask |= 1 << ARM_LR;
			}
			if (!mask) {
				return false; // empty register list has special PC behavior
			}
			ARMDynarecCacheInvalidate(ctx);
			ctx->cpu->dynarec->emitMultiple(ctx->cpu, ctx, isLoad, mask, ARM_SP, isLoad ? LSM_IA : LSM_DB, 1, false,
			                                insn->address, prefetch0, prefetch1);
			return true;
		}
		if ((opcode & 0xFF00) == 0xB000) {
			// ADD7/SUB4: SP += / -= #imm7 << 2 (no flags)
			sljit_sw immediate = (opcode & 0x7F) << 2;
			sljit_s32 sp = ARMDynarecCacheRead(ctx, ARM_SP, SLJIT_R0);
			if (opcode & 0x0080) {
				sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R0, 0, sp, 0, SLJIT_IMM, immediate);
			} else {
				sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, sp, 0, SLJIT_IMM, immediate);
			}
			ARMDynarecCacheWrite(ctx, ARM_SP, SLJIT_R0);
			++ctx->pendingCycles;
			return true;
		}
		return false;
	case 6:
		if ((opcode & 0xF000) == 0xC000 && ctx->cpu->dynarec->emitMultiple) {
			// STMIA (0xC000) / LDMIA (0xC800)
			bool isLoad = opcode & 0x0800;
			uint32_t mask = opcode & 0xFF;
			if (!mask) {
				return false;
			}
			int rn = (opcode >> 8) & 7;
			ARMDynarecCacheInvalidate(ctx);
			ctx->cpu->dynarec->emitMultiple(ctx->cpu, ctx, isLoad, mask, rn, LSM_IA, isLoad ? 2 : 1, false,
			                                insn->address, prefetch0, prefetch1);
			return true;
		}
		if ((opcode & 0xF000) == 0xD000 && (opcode & 0x0F00) < 0x0E00) {
			// B<cond>
			_emitConditionalBranch(ctx, insn, prefetch0, prefetch1);
			return true;
		}
		return false;
	case 7:
		if ((opcode & 0xF800) == 0xE000) {
			// Unconditional B: stay in the block for internal loops,
			// otherwise run the handler and link into the target block
			uint32_t target = insn->address + 4 + ((((int32_t)(opcode & 0x07FF)) << 21) >> 20);
			int targetIdx = _intraTargetIndex(ctx, target);
			ARMDynarecEmitFlushCycles(ctx);
			ARMDynarecCacheFlushDirty(ctx);
			if (targetIdx >= 0) {
				_emitIntraBranchTaken(ctx, targetIdx);
				return true;
			}
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(ARM_PC), SLJIT_IMM,
			               (sljit_sw) insn->address + 2 * WORD_SIZE_THUMB);
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, prefetch),
			               SLJIT_IMM, (sljit_sw) prefetch0);
			sljit_emit_op1(c, SLJIT_MOV32,
			               SLJIT_MEM1(SLJIT_S0),
			               (sljit_sw) offsetof(struct ARMCore, prefetch) + (sljit_sw) sizeof(uint32_t), SLJIT_IMM,
			               (sljit_sw) prefetch1);
			sljit_emit_op1(c, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, (sljit_sw) opcode);
			sljit_emit_icall(c, SLJIT_CALL, SLJIT_ARGS2V(P, 32), SLJIT_IMM, (sljit_sw) _thumbTable[opcode >> 6]);
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0),
			               (sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles16));
			sljit_emit_op2(c, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);
			ARMDynarecEmitLinkExit(ctx, target);
			return true;
		}
		if ((opcode & 0xF800) == 0xF000) {
			// BL prefix: LR = PC + sign-extended (imm11 << 12), fully constant
			int32_t offset = ((int32_t) ((opcode & 0x07FF) << 21)) >> 9;
			sljit_sw lr = (sljit_sw) (uint32_t) ((int32_t) (insn->address + 4) + offset);
			ARMDynarecCacheWriteImm(ctx, ARM_LR, lr);
			++ctx->pendingCycles;
			return true;
		}
		return false;
	default:
		return false;
	}
}

void ARMDynarecEmitFlushCycles(struct ARMDynarecEmitContext* ctx) {
	struct sljit_compiler* c = ctx->compiler;
	if (!ctx->pendingCycles) {
		return;
	}
	if (ctx->pendingCycles == 1) {
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, cycles),
		               SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, cycles), SLJIT_S1, 0);
	} else {
		sljit_emit_op2(c, SLJIT_MUL32, SLJIT_R0, 0, SLJIT_S1, 0, SLJIT_IMM, (sljit_sw) ctx->pendingCycles);
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, cycles),
		               SLJIT_MEM1(SLJIT_S0), (sljit_sw) offsetof(struct ARMCore, cycles), SLJIT_R0, 0);
	}
	ctx->pendingCycles = 0;
}

#endif
