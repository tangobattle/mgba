/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "dynarec-internal.h"

#ifdef ENABLE_DYNAREC

#include <mgba/internal/arm/decoder.h>

#include "third-party/sljit/sljit_src/sljitLir.h"

// Native ARM-mode data processing, bit-exact with src/arm/isa-arm.c:
//   arithmetic S-ops (_additionS/_subtractionS) zero the whole flags byte
//     then set NZCV                                 -> keep mask 0x00FFFFFF
//   logical S-ops (_neutralS) set N, Z and C = shifterCarryOut, keeping V
//     and bits 24-27                                -> keep mask 0x1FFFFFFF
//     (when the shifter preserves carry, C is untouched -> 0x3FFFFFFF)
// Supported: immediate operands and immediate-shifted register operands
// (LSL/LSR/ASR #imm, ROR #imm != 0). Register-shifted operands (extra
// internal cycle, shift-by-register semantics), RRX, ADC/SBC/RSC (carry-in),
// rd == PC (branch/SPSR restore), and the multiply family stay callouts.
// Executed cost is ARM_PREFETCH_CYCLES, the same as a failed condition, so
// both paths share one pending-cycle increment.

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

enum ARMALUOp {
	ALU_AND = 0x0,
	ALU_EOR = 0x1,
	ALU_SUB = 0x2,
	ALU_RSB = 0x3,
	ALU_ADD = 0x4,
	ALU_ADC = 0x5,
	ALU_SBC = 0x6,
	ALU_RSC = 0x7,
	ALU_TST = 0x8,
	ALU_TEQ = 0x9,
	ALU_CMP = 0xA,
	ALU_CMN = 0xB,
	ALU_ORR = 0xC,
	ALU_MOV = 0xD,
	ALU_BIC = 0xE,
	ALU_MVN = 0xF,
};

enum ShifterCarry {
	CARRY_PRESERVE, // shifter leaves C alone
	CARRY_CONST,    // compile-time 0/1
	CARRY_DYNAMIC,  // computed into R3 bit 29
};

// Operand 2 of the data-processing instruction
struct Operand2 {
	bool isConst;
	uint32_t constValue;
	sljit_s32 reg; // valid host operand when !isConst (always SLJIT_R1)
	enum ShifterCarry carry;
	uint32_t carryConst;
};

// Pure decode: whether the operand-2 form is natively supported
static bool _operand2Supported(uint32_t opcode) {
	if (opcode & 0x02000000) {
		return true; // rotated immediate
	}
	if (opcode & 0x00000010) {
		return false; // register-shifted register: extra cycle, >=32 semantics
	}
	if (((opcode >> 5) & 3) == 3 && !((opcode >> 7) & 0x1F)) {
		return false; // ROR #0 is RRX (carry-in dependent)
	}
	return true;
}

// Emits the shifter for a supported form. May clobber R1 and (for dynamic
// carry) R3.
static void _emitOperand2(struct ARMDynarecEmitContext* ctx, uint32_t opcode, uint32_t address, bool needCarry,
                          struct Operand2* out) {
	struct sljit_compiler* c = ctx->compiler;
	if (opcode & 0x02000000) {
		// Rotated immediate
		int rotate = (opcode & 0x00000F00) >> 7;
		uint32_t immediate = opcode & 0xFF;
		out->isConst = true;
		out->constValue = rotate ? (immediate >> rotate) | (immediate << (32 - rotate)) : immediate;
		if (!needCarry) {
			out->carry = CARRY_PRESERVE;
		} else if (rotate) {
			out->carry = CARRY_CONST;
			out->carryConst = out->constValue >> 31;
		} else {
			out->carry = CARRY_PRESERVE;
		}
		return;
	}
	int shiftType = (opcode >> 5) & 3;
	int immediate = (opcode >> 7) & 0x1F;
	int rm = opcode & 0xF;

	out->isConst = false;
	sljit_s32 src;
	if (rm == ARM_PC) {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, (sljit_sw) (address + 8));
		src = SLJIT_R1;
	} else {
		src = ARMDynarecCacheRead(ctx, rm, SLJIT_R1);
	}

	if (!immediate && shiftType == 0) {
		// LSL #0: plain register (read-only use), carry preserved
		out->reg = src;
		out->carry = CARRY_PRESERVE;
		return;
	}
	out->reg = SLJIT_R1;

	// Carry = last bit shifted out; for LSR/ASR #0 (== #32) it is bit 31
	if (needCarry) {
		int carryShift;
		switch (shiftType) {
		case 0: // LSL #n
			carryShift = 32 - immediate;
			break;
		default: // LSR/ASR/ROR
			carryShift = immediate ? immediate - 1 : 31;
			break;
		}
		sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, src, 0, SLJIT_IMM, carryShift);
		sljit_emit_op2(c, SLJIT_AND32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 1);
		sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 29);
		out->carry = CARRY_DYNAMIC;
	} else {
		out->carry = CARRY_PRESERVE;
	}

	switch (shiftType) {
	case 0:
		sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R1, 0, src, 0, SLJIT_IMM, immediate);
		break;
	case 1: // LSR; #0 means #32
		if (immediate) {
			sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R1, 0, src, 0, SLJIT_IMM, immediate);
		} else {
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, 0);
		}
		break;
	case 2: // ASR; #0 means #32
		sljit_emit_op2(c, SLJIT_ASHR32, SLJIT_R1, 0, src, 0, SLJIT_IMM, immediate ? immediate : 31);
		break;
	default: // ROR #n
		sljit_emit_op2(c, SLJIT_ROTR32, SLJIT_R1, 0, src, 0, SLJIT_IMM, immediate);
		break;
	}
}

static void _mergeFlags(struct sljit_compiler* c, sljit_sw keep) {
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, keep);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_R3, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_CPSR, SLJIT_R4, 0);
}

// NZCV from an add/sub already configured; mirrors emit-thumb.c's _emitArith.
// Operands may be cache registers (read-only); result to rd unless rd < 0.
static void _emitArith(struct ARMDynarecEmitContext* ctx, bool isSub, int rd, sljit_s32 m, sljit_sw mw, sljit_s32 n,
                       sljit_sw nw) {
	struct sljit_compiler* c = ctx->compiler;
	sljit_s32 op = isSub ? SLJIT_SUB32 : SLJIT_ADD32;
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
}

// N, Z from R2 (zero flag must be live) and C per the shifter; result to rd
// unless rd < 0. The dynamic shifter carry is already positioned in R3.
static void _emitLogicalFlags(struct ARMDynarecEmitContext* ctx, int rd, const struct Operand2* op2) {
	struct sljit_compiler* c = ctx->compiler;
	sljit_sw keep = KEEP_NZC;
	switch (op2->carry) {
	case CARRY_PRESERVE:
		keep = KEEP_NZ;
		sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R3, 0, SLJIT_EQUAL);
		sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 30);
		break;
	case CARRY_CONST:
		sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R3, 0, SLJIT_EQUAL);
		sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 30);
		if (op2->carryConst) {
			sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 0x20000000);
		}
		break;
	case CARRY_DYNAMIC:
		// R3 holds C at bit 29 already
		sljit_emit_op_flags(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_EQUAL);
		sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 30);
		sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
		break;
	}
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R2, 0, SLJIT_IMM, (sljit_sw) 0x80000000);
	sljit_emit_op2(c, SLJIT_OR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R4, 0);
	if (rd >= 0) {
		ARMDynarecCacheWrite(ctx, rd, SLJIT_R2);
	}
	_mergeFlags(c, keep);
}

bool ARMDynarecEmitARMNative(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn) {
	struct sljit_compiler* c = ctx->compiler;
	uint32_t opcode = insn->opcode;

	if ((opcode & 0x0C000000) != 0) {
		return false;
	}
	enum ARMALUOp op = (opcode >> 21) & 0xF;
	bool setFlags = opcode & (1 << 20);
	if (op >= ALU_ADC && op <= ALU_RSC) {
		return false;
	}
	if (op >= ALU_TST && op <= ALU_CMN && !setFlags) {
		// MRS/MSR/BX/BKPT space
		return false;
	}
	int rd = (opcode >> 12) & 0xF;
	int rn = (opcode >> 16) & 0xF;
	if (rd == ARM_PC) {
		return false;
	}
	if (!_operand2Supported(opcode)) {
		return false;
	}
	bool isLogical = op == ALU_AND || op == ALU_EOR || op == ALU_TST || op == ALU_TEQ || op == ALU_ORR ||
	                 op == ALU_MOV || op == ALU_BIC || op == ALU_MVN;
	bool hasDest = !(op >= ALU_TST && op <= ALU_CMN);
	bool needsRn = op != ALU_MOV && op != ALU_MVN;

	unsigned condition = opcode >> 28;
	struct sljit_jump* condFail = NULL;
	if (condition != ARM_CONDITION_AL) {
		sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR_FLAGS);
		sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 4);
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, ARMDynarecConditionLut[condition]);
		sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
		condFail = sljit_emit_op2cmpz(c, SLJIT_AND32 | SLJIT_JUMP_IF_ZERO, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_IMM, 1);
		ctx->inConditional = true;
	}

	struct Operand2 op2;
	_emitOperand2(ctx, opcode, insn->address, setFlags && isLogical, &op2);
	sljit_s32 n2 = op2.isConst ? SLJIT_IMM : op2.reg;
	sljit_sw n2w = op2.isConst ? (sljit_sw) op2.constValue : 0;

	sljit_s32 m = SLJIT_R0;
	if (needsRn) {
		if (rn == ARM_PC) {
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_IMM, (sljit_sw) (insn->address + 8));
		} else {
			m = ARMDynarecCacheRead(ctx, rn, SLJIT_R0);
		}
	}

	bool storeDone = false;
	switch (op) {
	case ALU_AND:
	case ALU_TST:
		sljit_emit_op2(c, SLJIT_AND32 | (setFlags ? SLJIT_SET_Z : 0), SLJIT_R2, 0, m, 0, n2, n2w);
		break;
	case ALU_EOR:
	case ALU_TEQ:
		sljit_emit_op2(c, SLJIT_XOR32 | (setFlags ? SLJIT_SET_Z : 0), SLJIT_R2, 0, m, 0, n2, n2w);
		break;
	case ALU_ORR:
		sljit_emit_op2(c, SLJIT_OR32 | (setFlags ? SLJIT_SET_Z : 0), SLJIT_R2, 0, m, 0, n2, n2w);
		break;
	case ALU_BIC:
		if (op2.isConst) {
			sljit_emit_op2(c, SLJIT_AND32 | (setFlags ? SLJIT_SET_Z : 0), SLJIT_R2, 0, m, 0, SLJIT_IMM,
			               (sljit_sw) ~op2.constValue);
		} else {
			sljit_emit_op2(c, SLJIT_XOR32, SLJIT_R1, 0, n2, n2w, SLJIT_IMM, -1);
			sljit_emit_op2(c, SLJIT_AND32 | (setFlags ? SLJIT_SET_Z : 0), SLJIT_R2, 0, m, 0, SLJIT_R1, 0);
		}
		break;
	case ALU_MOV:
		if (setFlags) {
			sljit_emit_op2(c, SLJIT_OR32 | SLJIT_SET_Z, SLJIT_R2, 0, n2, n2w, n2, n2w);
		} else if (op2.isConst) {
			ARMDynarecCacheWriteImm(ctx, rd, (sljit_sw) op2.constValue);
			storeDone = true;
		} else {
			ARMDynarecCacheWrite(ctx, rd, op2.reg);
			storeDone = true;
		}
		break;
	case ALU_MVN:
		sljit_emit_op2(c, SLJIT_XOR32 | (setFlags ? SLJIT_SET_Z : 0), SLJIT_R2, 0, n2, n2w, SLJIT_IMM, -1);
		break;
	case ALU_SUB:
	case ALU_CMP:
		if (setFlags) {
			_emitArith(ctx, true, hasDest ? rd : -1, m, 0, n2, n2w);
		} else {
			sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R2, 0, m, 0, n2, n2w);
		}
		break;
	case ALU_RSB:
		// d = shifter - n: swapped operands
		if (setFlags) {
			_emitArith(ctx, true, rd, n2, n2w, m, 0);
		} else {
			sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R2, 0, n2, n2w, m, 0);
		}
		break;
	case ALU_ADD:
	case ALU_CMN:
		if (setFlags) {
			_emitArith(ctx, false, hasDest ? rd : -1, m, 0, n2, n2w);
		} else {
			sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R2, 0, m, 0, n2, n2w);
		}
		break;
	default:
		ctx->inConditional = false;
		return false;
	}

	if (isLogical && setFlags) {
		_emitLogicalFlags(ctx, hasDest ? rd : -1, &op2);
	} else if (!setFlags && hasDest && !storeDone) {
		ARMDynarecCacheWrite(ctx, rd, SLJIT_R2);
	}

	ctx->inConditional = false;
	if (condFail) {
		sljit_set_label(condFail, sljit_emit_label(c));
	}
	// Executed and failed-condition cost are both ARM_PREFETCH_CYCLES
	++ctx->pendingCycles;
	return true;
}

#endif
