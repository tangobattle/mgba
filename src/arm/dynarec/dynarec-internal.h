/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef ARM_DYNAREC_INTERNAL_H
#define ARM_DYNAREC_INTERNAL_H

#include <mgba/internal/arm/dynarec/dynarec.h>

#ifdef ENABLE_DYNAREC

struct sljit_compiler;

struct ARMDynarecInsn {
	uint32_t opcode;
	uint32_t address;
	// Emit a deadline check after the callout (may reschedule events)
	bool isMemory;
	// Emit a block-validity check after the callout (may invalidate itself)
	bool isStore;
	// Unconditionally exit the block after the callout (SWI/BKPT/MSR/ILL)
	bool alwaysExit;
	// Some branch inside this block jumps here; the emitter places a
	// cycle-settled label at this instruction
	bool isBranchTarget;
};

// Register conventions inside compiled blocks:
//   SLJIT_S0 = struct ARMCore* (live for the whole block)
//   SLJIT_S1 = THUMB_PREFETCH_CYCLES (Thumb blocks; reloaded after callouts)
//   SLJIT_R0-R4 = scratch
#define ARM_DYNAREC_CACHE_REGS 3

enum ARMDynarecCacheState {
	ARM_DYNAREC_CACHE_UNLOADED = 0,
	ARM_DYNAREC_CACHE_CLEAN,
	ARM_DYNAREC_CACHE_DIRTY,
};

struct ARMDynarecEmitContext {
	struct sljit_compiler* compiler;
	struct ARMCore* cpu;
	struct ARMDynarecBlock* block;
	// Guest registers assigned to host saved registers S3..S5 for this
	// block (-1 = slot unused), with their compile-time coherence state
	int8_t cacheGuest[ARM_DYNAREC_CACHE_REGS];
	uint8_t cacheState[ARM_DYNAREC_CACHE_REGS];
	// Inside a conditionally-executed region, cache state transitions are
	// forbidden (the skipped path would leave compile-time state wrong)
	bool inConditional;
	// Natively-executed instructions whose prefetch cycles (S1 each) have
	// not been added to cpu->cycles yet
	unsigned pendingCycles;
	// Jumps to the shared block epilogue, patched at the end of emission
	struct sljit_jump** exits;
	size_t nExits;
	// Block contents for intra-block branch resolution
	const struct ARMDynarecInsn* insns;
	size_t count;
	uint32_t firstAddress;
	const uint32_t* lookahead;
	// Labels at branch-target instructions (NULL elsewhere), and pending
	// forward intra-block jumps patched once their target label exists
	struct sljit_label** insnLabels;
	struct sljit_jump** intraJumps;
	size_t* intraTargets;
	size_t nIntra;
};

// Word at instruction index (insn opcodes overflowing into the lookahead)
static inline uint32_t ARMDynarecSourceWord(const struct ARMDynarecEmitContext* ctx, size_t index) {
	if (index < ctx->count) {
		return ctx->insns[index].opcode;
	}
	return ctx->lookahead[index - ctx->count];
}

// Same table as the interpreter's (src/arm/arm.c); indexed by condition,
// bit (NZCV) set when the condition passes. Defined in emit.c.
extern const uint16_t ARMDynarecConditionLut[16];

// Emits cpu->cycles += pendingCycles * S1 if any are outstanding
void ARMDynarecEmitFlushCycles(struct ARMDynarecEmitContext* ctx);

// Emits a taken-branch block exit that links directly into the target block
// when possible: deadline due or target uncompiled/retired falls back to the
// dispatcher. Must be emitted with cycles and cached registers settled, and
// only where the runtime PC is statically known to be targetAddress (the
// branch handler has run and refilled the pipeline).
void ARMDynarecEmitLinkExit(struct ARMDynarecEmitContext* ctx, uint32_t targetAddress);

// Guest register cache (see ARMDynarecEmitContext). Readers get either the
// caching S register or the value loaded into the provided scratch; writers
// choose write-back (ALU results; deferred memory store) or write-through
// (memory-op destinations; memory stays authoritative so block exits need
// no flushing). Never used for r15.
int32_t ARMDynarecCacheRead(struct ARMDynarecEmitContext* ctx, int reg, int32_t scratch);
void ARMDynarecCacheWrite(struct ARMDynarecEmitContext* ctx, int reg, int32_t src);
void ARMDynarecCacheWriteImm(struct ARMDynarecEmitContext* ctx, int reg, intptr_t imm);
void ARMDynarecCacheWriteThrough(struct ARMDynarecEmitContext* ctx, int reg, int32_t src);
void ARMDynarecCacheFlushDirty(struct ARMDynarecEmitContext* ctx);
void ARMDynarecCacheInvalidate(struct ARMDynarecEmitContext* ctx);

// Emits native code for a Thumb instruction if it is in the natively
// supported subset; returns false to fall back to an interpreter callout.
// Native instructions cannot branch, fault, access memory, or reschedule
// events, so they need no PC/prefetch materialization or post-checks.
bool ARMDynarecEmitThumbNative(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn,
                               uint32_t prefetch0, uint32_t prefetch1);

// Emits native code for an ARM data-processing instruction if supported;
// returns false to fall back to an interpreter callout. Must be called
// before any per-instruction code is emitted.
bool ARMDynarecEmitARMNative(struct ARMDynarecEmitContext* ctx, const struct ARMDynarecInsn* insn);

// Returns a registered, valid block or NULL if this address cannot be compiled
struct ARMDynarecBlock* ARMDynarecCompile(struct ARMCore* cpu, uint32_t address, uint32_t canonical,
                                          enum ExecutionMode mode);

// Generates host code for the gathered instructions; fills block->entry/code
bool ARMDynarecEmitBlock(struct ARMCore* cpu, struct ARMDynarecBlock* block, const struct ARMDynarecInsn* insns,
                         size_t count, const uint32_t lookahead[2]);

// Inserts the block into the cache and page index (dynarec.c)
void ARMDynarecRegisterBlock(struct ARMCore* cpu, struct ARMDynarecBlock* block);

#endif

#endif
