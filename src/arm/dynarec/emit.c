/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "dynarec-internal.h"

#ifdef ENABLE_DYNAREC

#include <mgba/internal/arm/decoder.h>
#include <mgba/internal/arm/isa-arm.h>
#include <mgba/internal/arm/isa-thumb.h>

#include "third-party/sljit/sljit_src/sljitLir.h"

#define OFF_GPR(R) ((sljit_sw) offsetof(struct ARMCore, gprs) + (R) * (sljit_sw) sizeof(int32_t))
#define OFF_PC OFF_GPR(ARM_PC)
#ifdef __BIG_ENDIAN__
#define OFF_CPSR_FLAGS ((sljit_sw) offsetof(struct ARMCore, cpsr))
#else
#define OFF_CPSR_FLAGS ((sljit_sw) offsetof(struct ARMCore, cpsr) + 3)
#endif
#define OFF_CYCLES ((sljit_sw) offsetof(struct ARMCore, cycles))
#define OFF_NEXT_EVENT ((sljit_sw) offsetof(struct ARMCore, nextEvent))
#define OFF_PREFETCH0 ((sljit_sw) offsetof(struct ARMCore, prefetch))
#define OFF_PREFETCH1 ((sljit_sw) offsetof(struct ARMCore, prefetch) + (sljit_sw) sizeof(uint32_t))
#define OFF_SEQ_CYCLES32 ((sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles32))
#define OFF_SEQ_CYCLES16 ((sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles16))
#define OFF_NONSEQ_CYCLES32 ((sljit_sw) offsetof(struct ARMCore, memory.activeNonseqCycles32))

// Indexed by condition; bit (NZCV) set when the condition passes
const uint16_t ARMDynarecConditionLut[16] = {
	0xF0F0, // EQ [-Z--]
	0x0F0F, // NE [-z--]
	0xCCCC, // CS [--C-]
	0x3333, // CC [--c-]
	0xFF00, // MI [N---]
	0x00FF, // PL [n---]
	0xAAAA, // VS [---V]
	0x5555, // VC [---v]
	0x0C0C, // HI [-zC-]
	0xF3F3, // LS [-Z--] || [--c-]
	0xAA55, // GE [N--V] || [n--v]
	0x55AA, // LT [N--v] || [n--V]
	0x0A05, // GT [Nz-V] || [nz-v]
	0xF5FA, // LE [-Z--] || [Nz-v] || [nz-V]
	0xFFFF, // AL [----]
	0x0000 // NV
};

// Worst case per instruction: PC check + deadline check + valid check +
// always-exit jump or link-exit pair, plus slack for the epilogue
#define MAX_EXITS (ARM_DYNAREC_MAX_BLOCK_LENGTH * 6 + 8)

void ARMDynarecEmitLinkExit(struct ARMDynarecEmitContext* ctx, uint32_t targetAddress) {
	struct sljit_compiler* c = ctx->compiler;
	struct ARMDynarec* dynarec = ctx->cpu->dynarec;
	uint32_t canonical = dynarec->mapAddress(ctx->cpu, targetAddress);
	bool linkable = canonical != ARM_DYNAREC_UNCACHEABLE && !dynarec->isWritable(ctx->cpu, canonical);
	// Events first: due deadlines return to the dispatcher's processEvents
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES);
	ctx->exits[ctx->nExits] =
	    sljit_emit_cmp(c, SLJIT_SIG_GREATER_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_NEXT_EVENT);
	++ctx->nExits;
	if (!linkable) {
		ctx->exits[ctx->nExits] = sljit_emit_jump(c, SLJIT_JUMP);
		++ctx->nExits;
		return;
	}
	struct ARMDynarecLinkCell* cell = &ctx->block->cells[ctx->block->nCells];
	++ctx->block->nCells;
	cell->targetKey = canonical | (ctx->block->mode == MODE_THUMB ? 1 : 0);
	cell->target = NULL;
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R1, 0, SLJIT_MEM0(), (sljit_sw) &cell->target);
	ctx->exits[ctx->nExits] = sljit_emit_cmp(c, SLJIT_EQUAL, SLJIT_R1, 0, SLJIT_IMM, 0);
	++ctx->nExits;
	// Deferred-retire bookkeeping follows the running block across the link
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_MEM0(), (sljit_sw) &dynarec->currentBlock, SLJIT_R1, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_R1),
	               (sljit_sw) offsetof(struct ARMDynarecBlock, entryInner));
	sljit_emit_ijump(c, SLJIT_JUMP, SLJIT_R1, 0);
}

// Native ARM-mode branch staying inside the block. Replicates the B
// handler: ARM_PREFETCH_CYCLES + ARMWritePC's 2+N+S cycles and the
// taken-branch prefetch-unit reset; exits with the target's pipeline state
// if the event deadline was reached. Outstanding cycles must be flushed.
static void _emitARMIntraBranch(struct ARMDynarecEmitContext* ctx, size_t targetIdx) {
	struct sljit_compiler* c = ctx->compiler;
	// cycles += (1 + seq32) + 2 + nonseq32 + seq32
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_SEQ_CYCLES32);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R0, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_S0), OFF_NONSEQ_CYCLES32);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R1, 0);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 3);
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
	uint32_t target = ctx->firstAddress + targetIdx * WORD_SIZE_ARM;
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PC, SLJIT_IMM, (sljit_sw) target + WORD_SIZE_ARM);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM,
	               (sljit_sw) ARMDynarecSourceWord(ctx, targetIdx));
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM,
	               (sljit_sw) ARMDynarecSourceWord(ctx, targetIdx + 1));
	ctx->exits[ctx->nExits] = sljit_emit_jump(c, SLJIT_JUMP);
	++ctx->nExits;
}

// Adds the failed-condition cost ARM_PREFETCH_CYCLES, cached in S1 for ARM
// blocks (reloaded after every callout, so WAITCNT changes are honored)
static void _emitPrefetchCycles(struct sljit_compiler* compiler) {
	sljit_emit_op2(compiler, SLJIT_ADD32, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES,
	               SLJIT_S1, 0);
}

static sljit_s32 _cacheSlotReg(int slot) {
	return SLJIT_S(3 + slot);
}

static int _cacheSlot(const struct ARMDynarecEmitContext* ctx, int reg) {
	int i;
	for (i = 0; i < ARM_DYNAREC_CACHE_REGS; ++i) {
		if (ctx->cacheGuest[i] == reg) {
			return i;
		}
	}
	return -1;
}

int32_t ARMDynarecCacheRead(struct ARMDynarecEmitContext* ctx, int reg, int32_t scratch) {
	int slot = _cacheSlot(ctx, reg);
	if (slot < 0) {
		sljit_emit_op1(ctx->compiler, SLJIT_MOV32, scratch, 0, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg));
		return scratch;
	}
	if (ctx->cacheState[slot] == ARM_DYNAREC_CACHE_UNLOADED) {
		if (ctx->inConditional) {
			// Cannot establish cache state on a conditional path
			sljit_emit_op1(ctx->compiler, SLJIT_MOV32, scratch, 0, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg));
			return scratch;
		}
		sljit_emit_op1(ctx->compiler, SLJIT_MOV32, _cacheSlotReg(slot), 0, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg));
		ctx->cacheState[slot] = ARM_DYNAREC_CACHE_CLEAN;
	}
	return _cacheSlotReg(slot);
}

void ARMDynarecCacheWrite(struct ARMDynarecEmitContext* ctx, int reg, int32_t src) {
	int slot = _cacheSlot(ctx, reg);
	if (slot < 0 || (ctx->inConditional && ctx->cacheState[slot] == ARM_DYNAREC_CACHE_UNLOADED)) {
		sljit_emit_op1(ctx->compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg), src, 0);
		return;
	}
	// Writing a loaded entry on a conditional path is sound: the skipped
	// path leaves the register equal to memory, and flushing rewrites the
	// same value
	sljit_emit_op1(ctx->compiler, SLJIT_MOV32, _cacheSlotReg(slot), 0, src, 0);
	ctx->cacheState[slot] = ARM_DYNAREC_CACHE_DIRTY;
}

void ARMDynarecCacheWriteImm(struct ARMDynarecEmitContext* ctx, int reg, intptr_t imm) {
	int slot = _cacheSlot(ctx, reg);
	if (slot < 0 || (ctx->inConditional && ctx->cacheState[slot] == ARM_DYNAREC_CACHE_UNLOADED)) {
		sljit_emit_op1(ctx->compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg), SLJIT_IMM, imm);
		return;
	}
	sljit_emit_op1(ctx->compiler, SLJIT_MOV32, _cacheSlotReg(slot), 0, SLJIT_IMM, imm);
	ctx->cacheState[slot] = ARM_DYNAREC_CACHE_DIRTY;
}

void ARMDynarecCacheWriteThrough(struct ARMDynarecEmitContext* ctx, int reg, int32_t src) {
	sljit_emit_op1(ctx->compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg), src, 0);
	int slot = _cacheSlot(ctx, reg);
	if (slot >= 0 && ctx->cacheState[slot] != ARM_DYNAREC_CACHE_UNLOADED) {
		// Keep the cached copy current; memory is authoritative again
		sljit_emit_op1(ctx->compiler, SLJIT_MOV32, _cacheSlotReg(slot), 0, src, 0);
		ctx->cacheState[slot] = ARM_DYNAREC_CACHE_CLEAN;
	}
}

void ARMDynarecCacheFlushDirty(struct ARMDynarecEmitContext* ctx) {
	int i;
	for (i = 0; i < ARM_DYNAREC_CACHE_REGS; ++i) {
		if (ctx->cacheState[i] == ARM_DYNAREC_CACHE_DIRTY) {
			sljit_emit_op1(ctx->compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(ctx->cacheGuest[i]),
			               _cacheSlotReg(i), 0);
			ctx->cacheState[i] = ARM_DYNAREC_CACHE_CLEAN;
		}
	}
}

void ARMDynarecCacheInvalidate(struct ARMDynarecEmitContext* ctx) {
	ARMDynarecCacheFlushDirty(ctx);
	int i;
	for (i = 0; i < ARM_DYNAREC_CACHE_REGS; ++i) {
		ctx->cacheState[i] = ARM_DYNAREC_CACHE_UNLOADED;
	}
}

// Rough per-register reference counts to choose which guest registers get
// host registers for this block
static void _selectCacheRegs(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insns, size_t count,
                             enum ExecutionMode mode) {
	uint8_t refs[15] = { 0 };
	size_t i;
	for (i = 0; i < count; ++i) {
		uint32_t op = insns[i].opcode;
		if (mode == MODE_THUMB) {
			switch (op >> 13) {
			case 0: // shifts, add/sub
				++refs[op & 7];
				++refs[(op >> 3) & 7];
				break;
			case 1: // imm8 ops
				++refs[(op >> 8) & 7];
				break;
			case 2:
				if ((op & 0xFC00) == 0x4000) {
					++refs[op & 7];
					++refs[(op >> 3) & 7];
				} else if ((op & 0xF800) == 0x4800) {
					++refs[(op >> 8) & 7];
				} else if ((op & 0xF000) == 0x5000) {
					++refs[op & 7];
					++refs[(op >> 3) & 7];
					++refs[(op >> 6) & 7];
				}
				break;
			case 3:
			case 4:
				if ((op & 0xF000) == 0x9000) {
					++refs[(op >> 8) & 7];
					++refs[ARM_SP];
				} else {
					++refs[op & 7];
					++refs[(op >> 3) & 7];
				}
				break;
			default:
				break;
			}
		} else if ((op & 0x0C000000) == 0) {
			// ARM data processing
			int rd = (op >> 12) & 0xF;
			int rn = (op >> 16) & 0xF;
			if (rd != ARM_PC) {
				++refs[rd];
			}
			if (rn != ARM_PC) {
				++refs[rn];
			}
			if (!(op & 0x02000000) && (op & 0xF) != ARM_PC) {
				++refs[op & 0xF];
			}
		} else if ((op & 0x0C000000) == 0x04000000) {
			int rd = (op >> 12) & 0xF;
			int rn = (op >> 16) & 0xF;
			if (rd != ARM_PC) {
				++refs[rd];
			}
			if (rn != ARM_PC) {
				++refs[rn];
			}
		}
	}
	int slot;
	for (slot = 0; slot < ARM_DYNAREC_CACHE_REGS; ++slot) {
		int best = -1;
		int bestRefs = 1; // require at least two references
		int r;
		for (r = 0; r < 15; ++r) {
			if (refs[r] > bestRefs) {
				best = r;
				bestRefs = refs[r];
			}
		}
		ctx->cacheGuest[slot] = best;
		if (best >= 0) {
			refs[best] = 0;
		}
	}
}

bool ARMDynarecEmitBlock(struct ARMCore* cpu, struct ARMDynarecBlock* block, const struct ARMDynarecInsn* insns,
                         size_t count, const uint32_t lookahead[2]) {
	enum ExecutionMode mode = block->mode;
	uint32_t width = mode == MODE_THUMB ? WORD_SIZE_THUMB : WORD_SIZE_ARM;

	struct sljit_compiler* compiler = sljit_create_compiler(NULL);
	if (!compiler) {
		return false;
	}
	// S0 = cpu, S1 = the mode's prefetch cycles, S2 = memory-op scratch that
	// survives helper calls, S3-S5 = guest register cache; locals hold
	// helper cycle counters and spills ([0] counter, [4] offset, [8] pointer)
	sljit_emit_enter(compiler, 0, SLJIT_ARGS1V(P), 5, 3 + ARM_DYNAREC_CACHE_REGS, 24);
	struct sljit_label* innerLabel = sljit_emit_label(compiler);

	struct sljit_jump* exits[MAX_EXITS];
	struct sljit_label* insnLabels[ARM_DYNAREC_MAX_BLOCK_LENGTH] = { NULL };
	struct sljit_jump* intraJumps[ARM_DYNAREC_MAX_BLOCK_LENGTH];
	size_t intraTargets[ARM_DYNAREC_MAX_BLOCK_LENGTH];
	struct ARMDynarecEmitContext ctx = {
		.compiler = compiler,
		.cpu = cpu,
		.block = block,
		.pendingCycles = 0,
		.exits = exits,
		.nExits = 0,
		.insns = insns,
		.count = count,
		.firstAddress = insns[0].address,
		.lookahead = lookahead,
		.insnLabels = insnLabels,
		.intraJumps = intraJumps,
		.intraTargets = intraTargets,
		.nIntra = 0,
		.cacheGuest = { -1, -1, -1 },
		.cacheState = { 0 },
		.inConditional = false,
	};
	_selectCacheRegs(&ctx, insns, count, mode);
	// S1 caches the mode's prefetch cycles (THUMB_PREFETCH_CYCLES or
	// ARM_PREFETCH_CYCLES) for native instructions; WAITCNT writes can
	// change it, so it is reloaded after every callout
	sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0),
	               mode == MODE_THUMB ? OFF_SEQ_CYCLES16 : OFF_SEQ_CYCLES32);
	sljit_emit_op2(compiler, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);

	size_t i;
	for (i = 0; i < count; ++i) {
		const struct ARMDynarecInsn* insn = &insns[i];
		if (insn->isBranchTarget) {
			// Intra-block branches land here; outstanding native cycles
			// must be settled and the register cache reset so all incoming
			// edges agree on state
			ARMDynarecEmitFlushCycles(&ctx);
			ARMDynarecCacheInvalidate(&ctx);
			insnLabels[i] = sljit_emit_label(compiler);
		}
		uint32_t prefetch0 = i + 1 < count ? insns[i + 1].opcode : lookahead[i + 1 - count];
		uint32_t prefetch1 = i + 2 < count ? insns[i + 2].opcode : lookahead[i + 2 - count];
		uint32_t nextPC = insn->address + 2 * width;

		if (mode == MODE_THUMB && ARMDynarecEmitThumbNative(&ctx, insn, prefetch0, prefetch1)) {
			// Native instructions cannot branch, fault, touch memory, or
			// observe PC/prefetch, so pipeline state stays lazy
			continue;
		}
		if (mode == MODE_ARM && (insn->opcode >> 28) != ARM_CONDITION_NV && cpu->dynarec->emitMemory) {
			// ARM single loads/stores, immediate offset without writeback
			// (the dominant compiler-generated forms); rd == PC stays a
			// callout. The base may be PC: it reads as a constant.
			uint32_t opcode = insn->opcode;
			enum ARMDynarecMemOp op;
			bool decoded = false;
			sljit_sw offset = 0;
			if ((opcode & 0x0E000000) == 0x04000000 && (opcode & 0x01200000) == 0x01000000) {
				// LDR/STR/LDRB/STRB [rn, #±imm12]
				offset = opcode & 0xFFF;
				if (opcode & (1 << 22)) {
					op = opcode & (1 << 20) ? ARM_DYNAREC_MEM_LOAD8 : ARM_DYNAREC_MEM_STORE8;
				} else {
					op = opcode & (1 << 20) ? ARM_DYNAREC_MEM_LOAD32 : ARM_DYNAREC_MEM_STORE32;
				}
				decoded = true;
			} else if ((opcode & 0x0E400090) == 0x00400090 && (opcode & 0x01200000) == 0x01000000 &&
			           (opcode & 0x60)) {
				// LDRH/LDRSB/LDRSH/STRH [rn, #±imm8]
				offset = ((opcode >> 4) & 0xF0) | (opcode & 0xF);
				int sh = (opcode >> 5) & 3;
				if (opcode & (1 << 20)) {
					op = sh == 1 ? ARM_DYNAREC_MEM_LOAD16 : sh == 2 ? ARM_DYNAREC_MEM_LOADS8
					                                                : ARM_DYNAREC_MEM_LOADS16;
					decoded = true;
				} else if (sh == 1) {
					op = ARM_DYNAREC_MEM_STORE16;
					decoded = true;
				}
			}
			int rd = (opcode >> 12) & 0xF;
			int rn = (opcode >> 16) & 0xF;
			if (decoded && rd != ARM_PC) {
				if (!(opcode & (1 << 23))) {
					offset = -offset;
				}
				ARMDynarecEmitFlushCycles(&ctx);
				ARMDynarecCacheFlushDirty(&ctx);
				unsigned condition = opcode >> 28;
				struct sljit_jump* condFail = NULL;
				if (condition != ARM_CONDITION_AL) {
					sljit_emit_op1(compiler, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR_FLAGS);
					sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 4);
					sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, ARMDynarecConditionLut[condition]);
					sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
					condFail = sljit_emit_op2cmpz(compiler, SLJIT_AND32 | SLJIT_JUMP_IF_ZERO, SLJIT_R1, 0, SLJIT_R1,
					                              0, SLJIT_IMM, 1);
				}
				if (condFail) {
					ctx.inConditional = true;
				}
				if (rn == ARM_PC) {
					sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM,
					               (sljit_sw) (uint32_t) ((int64_t) insn->address + 8 + offset));
				} else {
					sljit_s32 base = ARMDynarecCacheRead(&ctx, rn, SLJIT_R1);
					if (offset) {
						sljit_emit_op2(compiler, SLJIT_ADD32, SLJIT_R1, 0, base, 0, SLJIT_IMM, offset);
					} else if (base != SLJIT_R1) {
						sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, base, 0);
					}
				}
				cpu->dynarec->emitMemory(cpu, &ctx, op, rd, true, insn->address, prefetch0, prefetch1);
				ctx.inConditional = false;
				if (condFail) {
					struct sljit_jump* skipFail = sljit_emit_jump(compiler, SLJIT_JUMP);
					sljit_set_label(condFail, sljit_emit_label(compiler));
					_emitPrefetchCycles(compiler);
					sljit_set_label(skipFail, sljit_emit_label(compiler));
				}
				continue;
			}
		}
		if (mode == MODE_ARM && (insn->opcode & 0x0E400000) == 0x08000000 && (insn->opcode >> 28) != ARM_CONDITION_NV &&
		    (insn->opcode & 0xFFFF) && !(insn->opcode & 0x8000) && ((insn->opcode >> 16) & 0xF) != ARM_PC &&
		    cpu->dynarec->emitMultiple) {
			// ARM LDM/STM without the S bit, PC, or a PC base
			bool isLoad = insn->opcode & (1 << 20);
			bool wbBit = insn->opcode & (1 << 21);
			int directionBits = ((insn->opcode >> 23) & 1 ? 0 : LSM_D) | ((insn->opcode >> 24) & 1 ? LSM_B : 0);
			int writeback = wbBit ? (isLoad ? 2 : 1) : 0;
			unsigned condition = insn->opcode >> 28;
			ARMDynarecEmitFlushCycles(&ctx);
			// The transfer loops and helpers access gprs in memory
			ARMDynarecCacheInvalidate(&ctx);
			struct sljit_jump* condFail = NULL;
			if (condition != ARM_CONDITION_AL) {
				sljit_emit_op1(compiler, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR_FLAGS);
				sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 4);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, ARMDynarecConditionLut[condition]);
				sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
				condFail = sljit_emit_op2cmpz(compiler, SLJIT_AND32 | SLJIT_JUMP_IF_ZERO, SLJIT_R1, 0, SLJIT_R1, 0,
				                              SLJIT_IMM, 1);
			}
			cpu->dynarec->emitMultiple(cpu, &ctx, isLoad, insn->opcode & 0xFFFF, (insn->opcode >> 16) & 0xF,
			                           directionBits, writeback, true, insn->address, prefetch0, prefetch1);
			if (condFail) {
				struct sljit_jump* skipFail = sljit_emit_jump(compiler, SLJIT_JUMP);
				sljit_set_label(condFail, sljit_emit_label(compiler));
				_emitPrefetchCycles(compiler);
				sljit_set_label(skipFail, sljit_emit_label(compiler));
			}
			continue;
		}
		if (mode == MODE_ARM && (insn->opcode >> 28) != ARM_CONDITION_NV && ARMDynarecEmitARMNative(&ctx, insn)) {
			// Native ARM data processing; cycle costs accumulate via S1
			continue;
		}
		if (mode == MODE_ARM && (insn->opcode & 0x0F000000) == 0x0A000000 && (insn->opcode >> 28) != ARM_CONDITION_NV) {
			// ARM B looping within this block: branch natively
			int64_t target = insn->address + 8 + ((((int32_t)(insn->opcode & 0x00FFFFFF)) << 8) >> 6);
			ARMDynarecEmitFlushCycles(&ctx);
			ARMDynarecCacheFlushDirty(&ctx);
			unsigned condition = insn->opcode >> 28;
			struct sljit_jump* condFail = NULL;
			if (condition != ARM_CONDITION_AL) {
				sljit_emit_op1(compiler, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR_FLAGS);
				sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 4);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, ARMDynarecConditionLut[condition]);
				sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
				condFail = sljit_emit_op2cmpz(compiler, SLJIT_AND32 | SLJIT_JUMP_IF_ZERO, SLJIT_R1, 0, SLJIT_R1,
				                              0, SLJIT_IMM, 1);
			}
			if (target >= ctx.firstAddress && (uint64_t) target < ctx.firstAddress + count * WORD_SIZE_ARM) {
				_emitARMIntraBranch(&ctx, ((uint32_t) target - ctx.firstAddress) / WORD_SIZE_ARM);
			} else {
				// Out-of-block: run the handler (ARMWritePC side effects)
				// and link straight into the target block
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PC, SLJIT_IMM,
				               (sljit_sw) insn->address + 2 * WORD_SIZE_ARM);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM,
				               (sljit_sw) prefetch0);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM,
				               (sljit_sw) prefetch1);
				void* handler =
				    (void*) _armTable[((insn->opcode >> 16) & 0xFF0) | ((insn->opcode >> 4) & 0x00F)];
				sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, (sljit_sw) insn->opcode);
				sljit_emit_icall(compiler, SLJIT_CALL, SLJIT_ARGS2V(P, 32), SLJIT_IMM, (sljit_sw) handler);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0), OFF_SEQ_CYCLES32);
				sljit_emit_op2(compiler, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);
				ARMDynarecEmitLinkExit(&ctx, (uint32_t) target);
			}
			if (condFail) {
				sljit_set_label(condFail, sljit_emit_label(compiler));
				_emitPrefetchCycles(compiler);
			}
			continue;
		}

		// Interpreter callout: materialize the architectural state the
		// handler (and open-bus reads) observe — PC one fetch ahead of the
		// executing instruction, pipeline filled from the source bytes this
		// block was compiled against — and settle outstanding cycles and
		// cached registers (handlers read and write gprs arbitrarily)
		ARMDynarecEmitFlushCycles(&ctx);
		ARMDynarecCacheInvalidate(&ctx);
		sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PC, SLJIT_IMM, (sljit_sw) nextPC);
		sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM, (sljit_sw) prefetch0);
		sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM, (sljit_sw) prefetch1);

		struct sljit_jump* condFail = NULL;
		if (mode == MODE_ARM) {
			unsigned condition = insn->opcode >> 28;
			if (condition == ARM_CONDITION_NV) {
				_emitPrefetchCycles(compiler);
				continue;
			}
			if (condition != ARM_CONDITION_AL) {
				// conditionMet = conditionLut[cond] & (1 << (cpsr.flags >> 4))
				sljit_emit_op1(compiler, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CPSR_FLAGS);
				sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 4);
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, ARMDynarecConditionLut[condition]);
				sljit_emit_op2(compiler, SLJIT_LSHR32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
				condFail = sljit_emit_op2cmpz(compiler, SLJIT_AND32 | SLJIT_JUMP_IF_ZERO, SLJIT_R1, 0, SLJIT_R1, 0,
				                              SLJIT_IMM, 1);
			}
		}

		void* handler;
		if (mode == MODE_THUMB) {
			handler = (void*) _thumbTable[insn->opcode >> 6];
		} else {
			handler = (void*) _armTable[((insn->opcode >> 16) & 0xFF0) | ((insn->opcode >> 4) & 0x00F)];
		}
		sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
		sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, (sljit_sw) insn->opcode);
		sljit_emit_icall(compiler, SLJIT_CALL, SLJIT_ARGS2V(P, 32), SLJIT_IMM, (sljit_sw) handler);
		sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0),
		               mode == MODE_THUMB ? OFF_SEQ_CYCLES16 : OFF_SEQ_CYCLES32);
		sljit_emit_op2(compiler, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);

		if (insn->alwaysExit) {
			// The handler may have rewritten any state (Tango traps can
			// move PC or load a whole save state); return to the dispatcher
			exits[ctx.nExits] = sljit_emit_jump(compiler, SLJIT_JUMP);
			++ctx.nExits;
		} else {
			exits[ctx.nExits] = sljit_emit_cmp(compiler, SLJIT_NOT_EQUAL | SLJIT_32, SLJIT_MEM1(SLJIT_S0), OFF_PC,
			                               SLJIT_IMM, (sljit_sw) nextPC);
			++ctx.nExits;
			if (insn->isMemory) {
				// Loads/stores can reschedule events (IO writes, HALTCNT,
				// Tango's end_run_loop) — honor a pulled-in deadline
				sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES);
				exits[ctx.nExits] = sljit_emit_cmp(compiler, SLJIT_SIG_GREATER_EQUAL | SLJIT_32, SLJIT_R0, 0,
				                               SLJIT_MEM1(SLJIT_S0), OFF_NEXT_EVENT);
				++ctx.nExits;
			}
			if (insn->isStore) {
				// A store may have invalidated this very block
				sljit_emit_op1(compiler, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM0(), (sljit_sw) &block->valid);
				exits[ctx.nExits] = sljit_emit_cmp(compiler, SLJIT_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_IMM, 0);
				++ctx.nExits;
			}
		}

		if (condFail) {
			struct sljit_jump* skipFail = sljit_emit_jump(compiler, SLJIT_JUMP);
			struct sljit_label* failLabel = sljit_emit_label(compiler);
			sljit_set_label(condFail, failLabel);
			_emitPrefetchCycles(compiler);
			struct sljit_label* nextLabel = sljit_emit_label(compiler);
			sljit_set_label(skipFail, nextLabel);
		}
	}

	// Fallthrough block end: settle cycles, cached registers, and the
	// pipeline state for the boundary after the last instruction. Exits
	// taken from callout post-checks skip this — handlers leave memory
	// authoritative — and are dirty-free by construction.
	ARMDynarecEmitFlushCycles(&ctx);
	ARMDynarecCacheFlushDirty(&ctx);
	uint32_t endPC = insns[count - 1].address + 2 * width;
	sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PC, SLJIT_IMM, (sljit_sw) endPC);
	sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM, (sljit_sw) lookahead[0]);
	sljit_emit_op1(compiler, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM, (sljit_sw) lookahead[1]);

	struct sljit_label* exitLabel = sljit_emit_label(compiler);
	for (i = 0; i < ctx.nExits; ++i) {
		sljit_set_label(exits[i], exitLabel);
	}
	for (i = 0; i < ctx.nIntra; ++i) {
		sljit_set_label(intraJumps[i], insnLabels[intraTargets[i]]);
	}
	sljit_emit_return_void(compiler);

	if (sljit_get_compiler_error(compiler) != SLJIT_SUCCESS) {
		sljit_free_compiler(compiler);
		return false;
	}
	void* code = sljit_generate_code(compiler, 0, NULL);
	if (!code) {
		sljit_free_compiler(compiler);
		return false;
	}
	block->code = code;
	block->entry = (void (*)(struct ARMCore*)) code;
	block->entryInner = (void*) sljit_get_label_addr(innerLabel);
	sljit_free_compiler(compiler);
	return true;
}

#endif
