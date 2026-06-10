/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gba/memory.h>

#ifdef ENABLE_DYNAREC

#include <mgba/internal/gba/gba.h>
#include <mgba-util/math.h>

#include "arm/dynarec/dynarec-internal.h"
#include "third-party/sljit/sljit_src/sljitLir.h"

// Native code for guest memory accesses, replicating GBALoadX/GBAStoreX
// (src/gba/memory.c) including their exact cycle math:
//   loads:  wait = regionWait; wait += 2; if (addr < ROM0) stall; rotate
//   stores: wait = regionWait; wait += 1; if (addr < ROM0) stall
// plus the Thumb handler's THUMB_PREFETCH_CYCLES base and LOAD/STORE_POST_BODY
// (activeNonseqCycles16 - activeSeqCycles16).
//
// Inline fast paths: EWRAM and IWRAM (with the prefetch-unit stall call) and
// in-bounds ROM0/ROM1 loads (regions 0x8-0xB; 0xC-0xD have EEPROM mappings).
// Everything else calls the full GBALoadX/GBAStoreX helpers. Fast-path
// stores fall back to the helper when the page-has-code bitmap hits, so the
// helper's invalidation hook runs.
//
// Register/stack contract (see dynarec-internal.h): S0 = cpu, S1 = Thumb
// prefetch cycles, S2 = the guest address (survives calls). Locals:
// [SP+0] helper cycle counter, [SP+4] spilled offset, [SP+8] spilled pointer.

#define OFF_GPR(R) ((sljit_sw) offsetof(struct ARMCore, gprs) + (R) * (sljit_sw) sizeof(int32_t))
#define OFF_PC OFF_GPR(ARM_PC)
#define OFF_CYCLES ((sljit_sw) offsetof(struct ARMCore, cycles))
#define OFF_NEXT_EVENT ((sljit_sw) offsetof(struct ARMCore, nextEvent))
#define OFF_PREFETCH0 ((sljit_sw) offsetof(struct ARMCore, prefetch))
#define OFF_PREFETCH1 ((sljit_sw) offsetof(struct ARMCore, prefetch) + (sljit_sw) sizeof(uint32_t))
#define OFF_SEQ16 ((sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles16))
#define OFF_NONSEQ16 ((sljit_sw) offsetof(struct ARMCore, memory.activeNonseqCycles16))
#define OFF_MASTER ((sljit_sw) offsetof(struct ARMCore, master))

#define LOCAL_CC 0
#define LOCAL_OFFSET 4
#define LOCAL_PTR 8
#define OFF_SEQ32 ((sljit_sw) offsetof(struct ARMCore, memory.activeSeqCycles32))
#define OFF_NONSEQ32 ((sljit_sw) offsetof(struct ARMCore, memory.activeNonseqCycles32))

struct MemOpInfo {
	bool load;
	int width;
	sljit_s32 accessOp;
	void* helper;
	sljit_s32 helperArgs;
	sljit_sw nsTable;
	int waitBase; // loads add 2, stores add 1
};

static void _getOpInfo(enum ARMDynarecMemOp op, struct MemOpInfo* info) {
	switch (op) {
	case ARM_DYNAREC_MEM_LOAD32:
		*info = (struct MemOpInfo) { true, 4, SLJIT_MOV32, (void*) GBALoad32, SLJIT_ARGS3(32, P, 32, P),
			                         (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq32), 2 };
		break;
	case ARM_DYNAREC_MEM_LOAD16:
	case ARM_DYNAREC_MEM_LOADS16:
		*info = (struct MemOpInfo) { true, 2, SLJIT_MOV_U16, (void*) GBALoad16, SLJIT_ARGS3(32, P, 32, P),
			                         (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq16), 2 };
		break;
	case ARM_DYNAREC_MEM_LOAD8:
	case ARM_DYNAREC_MEM_LOADS8:
		*info = (struct MemOpInfo) { true, 1, SLJIT_MOV_U8, (void*) GBALoad8, SLJIT_ARGS3(32, P, 32, P),
			                         (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq16), 2 };
		break;
	case ARM_DYNAREC_MEM_STORE32:
		*info = (struct MemOpInfo) { false, 4, SLJIT_MOV32, (void*) GBAStore32, SLJIT_ARGS4V(P, 32, 32, P),
			                         (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq32), 1 };
		break;
	case ARM_DYNAREC_MEM_STORE16:
		*info = (struct MemOpInfo) { false, 2, SLJIT_MOV_U16, (void*) GBAStore16, SLJIT_ARGS4V(P, 32, 32, P),
			                         (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq16), 1 };
		break;
	case ARM_DYNAREC_MEM_STORE8:
		*info = (struct MemOpInfo) { false, 1, SLJIT_MOV_U8, (void*) GBAStore8, SLJIT_ARGS4V(P, 32, 32, P),
			                         (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq16), 1 };
		break;
	}
}

// JIT-compiled GBAMemoryStall body (src/gba/memory.c), shared by all blocks
// of a core and called with sljit's lightweight register-argument
// convention: R0 = guest PC (the value gprs[15] holds), R1 = wait, R2 = cpu;
// returns the stalled wait in R0; clobbers R0-R4. The caller has already
// handled the early-out, so the stall logic itself runs unconditionally.
// previousLoads cancels out of the lastPrefetchedPc update:
//   loads + previousLoads - 1 == 7 - remaining, where remaining counts down
//   from maxLoads - 1 == 7 - previousLoads.
static void* _compileStallStub(void) {
	struct sljit_compiler* c = sljit_create_compiler(NULL);
	if (!c) {
		return NULL;
	}
	sljit_emit_enter(c, SLJIT_ENTER_REG_ARG, SLJIT_ARGS3(32, 32_R, 32_R, P_R), 5, 0, 16);
	// dist = lastPrefetchedPc - pc; spill pc
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R2), OFF_MASTER);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R3),
	               (sljit_sw) offsetof(struct GBA, memory.lastPrefetchedPc));
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R0, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_SP), 0, SLJIT_R0, 0);
	// remaining = maxLoads - 1 = 7 - (dist < 16 ? dist >> 1 : 0)
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R4, 0, SLJIT_IMM, 7);
	struct sljit_jump* noPrev = sljit_emit_cmp(c, SLJIT_GREATER_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_IMM, 16);
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 1);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_R3, 0);
	sljit_set_label(noPrev, sljit_emit_label(c));
	// stall = s + 1; while (stall < wait && remaining) { stall += s; --remaining; }
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2), OFF_SEQ16);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R3, 0, SLJIT_R0, 0, SLJIT_IMM, 1);
	struct sljit_label* loop = sljit_emit_label(c);
	struct sljit_jump* outStall = sljit_emit_cmp(c, SLJIT_SIG_GREATER_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_R1, 0);
	struct sljit_jump* outLoads = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R4, 0, SLJIT_IMM, 0);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R0, 0);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 1);
	struct sljit_jump* again = sljit_emit_jump(c, SLJIT_JUMP);
	sljit_set_label(again, loop);
	struct sljit_label* out = sljit_emit_label(c);
	sljit_set_label(outStall, out);
	sljit_set_label(outLoads, out);
	// lastPrefetchedPc = pc + 2 * (7 - remaining)
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R0, 0, SLJIT_IMM, 7, SLJIT_R4, 0);
	sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 1);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_SP), 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R4, 0, SLJIT_MEM1(SLJIT_R2), OFF_MASTER);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_R4), (sljit_sw) offsetof(struct GBA, memory.lastPrefetchedPc),
	               SLJIT_R0, 0);
	// if (stall > wait) wait = stall
	struct sljit_jump* noClamp = sljit_emit_cmp(c, SLJIT_SIG_LESS_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_R1, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_R3, 0);
	sljit_set_label(noClamp, sljit_emit_label(c));
	// wait -= (nonseq16 - s); wait -= stall
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2), OFF_NONSEQ16);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2), OFF_SEQ16);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R0, 0);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R1, 0, SLJIT_R1, 0, SLJIT_R3, 0);
	sljit_emit_return(c, SLJIT_MOV32, SLJIT_R1, 0);
	if (sljit_get_compiler_error(c) != SLJIT_SUCCESS) {
		sljit_free_compiler(c);
		return NULL;
	}
	void* code = sljit_generate_code(c, 0, NULL);
	sljit_free_compiler(c);
	return code;
}

static void _freeStub(void* code) {
	sljit_free_code(code, NULL);
}

void GBADynarecInitStubs(struct ARMCore* cpu) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	if (!dynarec || dynarec->platformCode) {
		return;
	}
	dynarec->platformCode = _compileStallStub();
	dynarec->platformCodeFree = _freeStub;
}

// Applies GBAMemoryStall to the wait in R1 with the early-out inlined:
// executing from RAM or with the cart prefetch disabled, the stall is the
// identity (no side effects), so skip it entirely. Expects gba in R2;
// leaves the wait in R0.
static void _emitStallCall(struct sljit_compiler* c, struct ARMDynarec* dynarec, sljit_sw pcValue) {
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2),
	               (sljit_sw) offsetof(struct GBA, memory.activeRegion));
	struct sljit_jump* skipRegion = sljit_emit_cmp(c, SLJIT_SIG_LESS | SLJIT_32, SLJIT_R0, 0,
	                                               SLJIT_IMM, GBA_REGION_ROM0);
	sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2),
	               (sljit_sw) offsetof(struct GBA, memory.prefetch));
	struct sljit_jump* skipPrefetch = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_IMM, 0);
	if (dynarec->platformCode) {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_IMM, pcValue);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_S0, 0);
		sljit_emit_icall(c, SLJIT_CALL_REG_ARG, SLJIT_ARGS3(32, 32, 32, P), SLJIT_IMM,
		                 (sljit_sw) dynarec->platformCode);
	} else {
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
		sljit_emit_icall(c, SLJIT_CALL, SLJIT_ARGS2(32, P, 32), SLJIT_IMM, (sljit_sw) GBAMemoryStall);
	}
	struct sljit_jump* done = sljit_emit_jump(c, SLJIT_JUMP);
	struct sljit_label* skipLabel = sljit_emit_label(c);
	sljit_set_label(skipRegion, skipLabel);
	sljit_set_label(skipPrefetch, skipLabel);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_R1, 0);
	sljit_set_label(done, sljit_emit_label(c));
}

// cycles += R0 + extra + per-mode prefetch/post cost: Thumb instructions add
// THUMB_PREFETCH_CYCLES (cached in S1) + (NS16 - S16); ARM instructions add
// ARM_PREFETCH_CYCLES (1 + S32) + (NS32 - S32) = NS32 + 1
static void _emitCycleTail(struct sljit_compiler* c, bool isArm, sljit_sw extra) {
	if (isArm) {
		// S1 holds ARM_PREFETCH_CYCLES from instruction entry; the access
		// itself may rewrite WAITCNT, so NS32/S32 are reloaded here but the
		// prefetch term must use the entry value
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_S0), OFF_NONSEQ32);
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R1, 0);
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_S0), OFF_SEQ32);
		sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R1, 0);
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_S1, 0);
		if (extra) {
			sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, extra);
		}
	} else {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_S0), OFF_NONSEQ16);
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R1, 0);
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_S0), OFF_SEQ16);
		sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_R1, 0);
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_S1, 0);
		if (extra) {
			sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, extra);
		}
	}
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES, SLJIT_R0, 0);
}

// Which IO registers GBAIORead (src/gba/io.c) serves straight from the io
// array: 0 = full helper required, 1 = direct read after clearing
// haltPending, 2 = direct read (GBAIOIsReadConstant). Keep in sync with
// GBAIORead's switch; anything not provably transparent stays 0.
static const uint8_t _ioDirectRead[0x200] = {
	[0x000 >> 1] = 1, // DISPCNT
	[0x002 >> 1] = 1, // STEREOCNT
	[0x004 >> 1] = 1, // DISPSTAT
	[0x006 >> 1] = 1, // VCOUNT
	[0x008 >> 1] = 2, // BG0CNT
	[0x00A >> 1] = 2, // BG1CNT
	[0x00C >> 1] = 2, // BG2CNT
	[0x00E >> 1] = 2, // BG3CNT
	[0x048 >> 1] = 2, // WININ
	[0x04A >> 1] = 2, // WINOUT
	[0x050 >> 1] = 2, // BLDCNT
	[0x052 >> 1] = 2, // BLDALPHA
	[0x082 >> 1] = 2, // SOUNDCNT_HI
	[0x088 >> 1] = 1, // SOUNDBIAS
	[0x0BA >> 1] = 1, // DMA0CNT_HI
	[0x0C6 >> 1] = 1, // DMA1CNT_HI
	[0x0D2 >> 1] = 1, // DMA2CNT_HI
	[0x0DE >> 1] = 1, // DMA3CNT_HI
	[0x102 >> 1] = 2, // TM0CNT_HI
	[0x106 >> 1] = 2, // TM1CNT_HI
	[0x10A >> 1] = 2, // TM2CNT_HI
	[0x10E >> 1] = 2, // TM3CNT_HI
	[0x132 >> 1] = 2, // KEYCNT
	[0x200 >> 1] = 2, // IE
	[0x202 >> 1] = 1, // IF
	[0x204 >> 1] = 1, // WAITCNT
	[0x208 >> 1] = 1, // IME
};

// Loaded value is in R2 (fast paths: raw aligned load; helper: rotated by
// the C implementation). Applies the rotation/extension the handler would
// and stores to rd. The address is in S2.
static void _emitValueTail(struct ARMDynarecEmitContext* ctx, enum ARMDynarecMemOp op, int rd, bool rotated) {
	struct sljit_compiler* c = ctx->compiler;
	switch (op) {
	case ARM_DYNAREC_MEM_LOAD32:
		if (!rotated) {
			sljit_emit_op2(c, SLJIT_AND32, SLJIT_R0, 0, SLJIT_S2, 0, SLJIT_IMM, 3);
			sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 3);
			sljit_emit_op2(c, SLJIT_ROTR32, SLJIT_R2, 0, SLJIT_R2, 0, SLJIT_R0, 0);
		}
		break;
	case ARM_DYNAREC_MEM_LOAD16:
		if (!rotated) {
			sljit_emit_op2(c, SLJIT_AND32, SLJIT_R0, 0, SLJIT_S2, 0, SLJIT_IMM, 1);
			sljit_emit_op2(c, SLJIT_SHL32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 3);
			sljit_emit_op2(c, SLJIT_ROTR32, SLJIT_R2, 0, SLJIT_R2, 0, SLJIT_R0, 0);
		}
		break;
	case ARM_DYNAREC_MEM_LOADS16: {
		// Aligned: SXT16. Unaligned: the GBA yields SXT8 of the upper byte
		// (the helper has already rotated it into the low byte)
		sljit_emit_op2u(c, SLJIT_AND32 | SLJIT_SET_Z, SLJIT_S2, 0, SLJIT_IMM, 1);
		struct sljit_jump* aligned = sljit_emit_jump(c, SLJIT_ZERO);
		if (!rotated) {
			sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R2, 0, SLJIT_R2, 0, SLJIT_IMM, 8);
		}
		sljit_emit_op1(c, SLJIT_MOV32_S8, SLJIT_R2, 0, SLJIT_R2, 0);
		struct sljit_jump* done = sljit_emit_jump(c, SLJIT_JUMP);
		sljit_set_label(aligned, sljit_emit_label(c));
		sljit_emit_op1(c, SLJIT_MOV32_S16, SLJIT_R2, 0, SLJIT_R2, 0);
		sljit_set_label(done, sljit_emit_label(c));
		break;
	}
	case ARM_DYNAREC_MEM_LOADS8:
		sljit_emit_op1(c, SLJIT_MOV32_S8, SLJIT_R2, 0, SLJIT_R2, 0);
		break;
	default:
		break;
	}
	// Write-through: memory stays authoritative so block exits stay
	// dirty-free, while a cached copy is refreshed for later reads
	ARMDynarecCacheWriteThrough(ctx, rd, SLJIT_R2);
}

// Emits the store-target page-has-code test; jumps to helperLabel (which
// runs the invalidation hook) when compiled code may overlap the write.
// canonicalBase is the region base >> ARM_DYNAREC_PAGE_BITS; the aligned
// in-region offset is in R4.
static struct sljit_jump* _emitBitmapTest(struct sljit_compiler* c, const struct ARMDynarec* dynarec,
                                          sljit_sw canonicalBase) {
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_R4, 0, SLJIT_IMM, ARM_DYNAREC_PAGE_BITS);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, canonicalBase);
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R2, 0, SLJIT_R0, 0, SLJIT_IMM, 3);
	sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R2, 0, SLJIT_R2, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_IMM, (sljit_sw) dynarec->pageBitmap);
	sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R3, 0, SLJIT_MEM2(SLJIT_R3, SLJIT_R2), 0);
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R0, 0, SLJIT_R0, 0, SLJIT_IMM, 7);
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R0, 0);
	return sljit_emit_op2cmpz(c, SLJIT_AND32 | SLJIT_JUMP_IF_NON_ZERO, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM, 1);
}

void GBADynarecEmitMemory(struct ARMCore* cpu, struct ARMDynarecEmitContext* ctx, enum ARMDynarecMemOp op, int rd,
                          bool isArm, uint32_t instrAddress, uint32_t prefetch0, uint32_t prefetch1) {
	struct sljit_compiler* c = ctx->compiler;
	struct MemOpInfo info;
	_getOpInfo(op, &info);
	sljit_sw alignDown = -(sljit_sw) info.width; // ~(width - 1)

	// Any path may call into C code that observes or advances cycles, and
	// the prefetch-unit stall reads PC. Dirty cached registers must reach
	// memory before the post-access exit checks can leave the block. The
	// helper/RAM/ROM/IO paths below are alternatives chosen at run time, so
	// cache state transitions inside them are forbidden: an S-register load
	// emitted on one path would be skipped on the others, leaving later
	// compile-time state wrong (conditional-region rule).
	ARMDynarecEmitFlushCycles(ctx);
	ARMDynarecCacheFlushDirty(ctx);
	bool wasConditional = ctx->inConditional;
	ctx->inConditional = true;
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PC, SLJIT_IMM,
	               (sljit_sw) instrAddress + (isArm ? 8 : 4));
	sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_S2, 0, SLJIT_R1, 0);

	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, SLJIT_S2, 0, SLJIT_IMM, 24);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R3, 0, SLJIT_IMM, 2);
	struct sljit_jump* toRAM = sljit_emit_cmp(c, SLJIT_LESS_EQUAL | SLJIT_32, SLJIT_R4, 0, SLJIT_IMM, 1);
	struct sljit_jump* toROM = NULL;
	struct sljit_jump* toIO = NULL;
	if (info.load) {
		sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R3, 0, SLJIT_IMM, 8);
		toROM = sljit_emit_cmp(c, SLJIT_LESS_EQUAL | SLJIT_32, SLJIT_R4, 0, SLJIT_IMM, 3);
		if (info.width == 2) {
			toIO = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_IMM, GBA_REGION_IO);
		}
	}

	// ---- Helper path: full GBALoadX/GBAStoreX (BIOS, IO, VRAM, SRAM, OOB
	// ROM, EEPROM regions, and stores that may overlap compiled code) ----
	struct sljit_label* helperLabel = sljit_emit_label(c);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM, (sljit_sw) prefetch0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM, (sljit_sw) prefetch1);
	// The cycle tail adds S1 (THUMB_PREFETCH_CYCLES) itself; the helper only
	// accumulates its own wait into the counter
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_SP), LOCAL_CC, SLJIT_IMM, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_S2, 0);
	if (info.load) {
		sljit_get_local_base(c, SLJIT_R2, 0, LOCAL_CC);
	} else {
		sljit_s32 v = ARMDynarecCacheRead(ctx, rd, SLJIT_R2);
		if (v != SLJIT_R2) {
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R2, 0, v, 0);
		}
		sljit_get_local_base(c, SLJIT_R3, 0, LOCAL_CC);
	}
	sljit_emit_icall(c, SLJIT_CALL, info.helperArgs, SLJIT_IMM, (sljit_sw) info.helper);
	if (info.load) {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R2, 0, SLJIT_R0, 0);
	}
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_CC);
	_emitCycleTail(c, isArm, 0);
	if (info.load) {
		_emitValueTail(ctx, op, rd, true);
	}
	// Helpers can touch IO: honor rescheduled deadlines; stores may have
	// invalidated this very block
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES);
	ctx->exits[ctx->nExits] =
	    sljit_emit_cmp(c, SLJIT_SIG_GREATER_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_NEXT_EVENT);
	++ctx->nExits;
	if (!info.load) {
		sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM0(), (sljit_sw) &ctx->block->valid);
		ctx->exits[ctx->nExits] = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_IMM, 0);
		++ctx->nExits;
	}
	// IO stores can rewrite WAITCNT; refresh the cached prefetch cycles
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0), isArm ? OFF_SEQ32 : OFF_SEQ16);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);
	struct sljit_jump* doneHelper = sljit_emit_jump(c, SLJIT_JUMP);

	// ---- RAM paths ----
	sljit_set_label(toRAM, sljit_emit_label(c));
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
	struct sljit_jump* toIWRAM = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_IMM, GBA_REGION_IWRAM);

	// EWRAM
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_S2, 0, SLJIT_IMM, (GBA_SIZE_EWRAM - 1) & alignDown);
	struct sljit_jump* ewramHit = NULL;
	if (!info.load) {
		ewramHit = _emitBitmapTest(c, cpu->dynarec, GBA_BASE_EWRAM >> ARM_DYNAREC_PAGE_BITS);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
	}
	sljit_emit_op1(c, SLJIT_MOV_S8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2), info.nsTable + GBA_REGION_EWRAM);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, memory.wram));
	struct sljit_jump* toRAMCommon = sljit_emit_jump(c, SLJIT_JUMP);

	// IWRAM
	sljit_set_label(toIWRAM, sljit_emit_label(c));
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_S2, 0, SLJIT_IMM, (GBA_SIZE_IWRAM - 1) & alignDown);
	struct sljit_jump* iwramHit = NULL;
	if (!info.load) {
		iwramHit = _emitBitmapTest(c, cpu->dynarec, GBA_BASE_IWRAM >> ARM_DYNAREC_PAGE_BITS);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
	}
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_IMM, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, memory.iwram));

	// Common RAM tail: R4 = aligned offset, R3 = backing pointer, R0 = base
	// region waitstates. The prefetch-unit stall must run exactly as the
	// interpreter's would (it credits prefetched halfwords when executing
	// from ROM), so call it and let it early-out otherwise.
	sljit_set_label(toRAMCommon, sljit_emit_label(c));
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_SP), LOCAL_OFFSET, SLJIT_R4, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_MEM1(SLJIT_SP), LOCAL_PTR, SLJIT_R3, 0);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R1, 0, SLJIT_R0, 0, SLJIT_IMM, info.waitBase);
	_emitStallCall(c, cpu->dynarec, (sljit_sw) instrAddress + (isArm ? 8 : 4));
	_emitCycleTail(c, isArm, 0);
	sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R4, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_OFFSET);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_PTR);
	if (info.load) {
		sljit_emit_op1(c, info.accessOp, SLJIT_R2, 0, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 0);
		_emitValueTail(ctx, op, rd, false);
	} else {
		sljit_s32 v = ARMDynarecCacheRead(ctx, rd, SLJIT_R2);
		if (v != SLJIT_R2) {
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R2, 0, v, 0);
		}
		sljit_emit_op1(c, info.accessOp, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 0, SLJIT_R2, 0);
	}
	struct sljit_jump* doneRAM = info.load ? sljit_emit_jump(c, SLJIT_JUMP) : NULL;

	// ---- ROM path (loads only; regions 0x8-0xB, in-bounds) ----
	if (info.load) {
		sljit_set_label(toROM, sljit_emit_label(c));
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
		sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_S2, 0, SLJIT_IMM, (GBA_SIZE_ROM0 - 1) & alignDown);
		sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R4, 0, SLJIT_R4, 0);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, memory.romSize));
		struct sljit_jump* oob = sljit_emit_cmp(c, SLJIT_GREATER_EQUAL, SLJIT_R4, 0, SLJIT_R0, 0);
		sljit_set_label(oob, helperLabel);
		sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R0, 0, SLJIT_S2, 0, SLJIT_IMM, 24);
		sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R0, 0, SLJIT_R0, 0);
		sljit_emit_op2(c, SLJIT_ADD, SLJIT_R1, 0, SLJIT_R2, 0, SLJIT_IMM, info.nsTable);
		sljit_emit_op1(c, SLJIT_MOV_S8, SLJIT_R0, 0, SLJIT_MEM2(SLJIT_R1, SLJIT_R0), 0);
		_emitCycleTail(c, isArm, info.waitBase);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, memory.rom));
		sljit_emit_op1(c, info.accessOp, SLJIT_R2, 0, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 0);
		_emitValueTail(ctx, op, rd, false);
	} else {
		sljit_set_label(ewramHit, helperLabel);
		sljit_set_label(iwramHit, helperLabel);
	}

	struct sljit_jump* doneROM = NULL;
	if (toIO) {
		doneROM = sljit_emit_jump(c, SLJIT_JUMP);
		// ---- IO path (16-bit loads): registers GBAIORead serves straight
		// from the io array are read inline; others go to the helper ----
		sljit_set_label(toIO, sljit_emit_label(c));
		sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_S2, 0, SLJIT_IMM, 0x00FFFFFE);
		struct sljit_jump* ioHigh = sljit_emit_cmp(c, SLJIT_GREATER_EQUAL | SLJIT_32, SLJIT_R4, 0, SLJIT_IMM, 0x400);
		sljit_set_label(ioHigh, helperLabel);
		sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 1);
		sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R4, 0, SLJIT_R4, 0);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_IMM, (sljit_sw) _ioDirectRead);
		sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R3, 0, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 0);
		struct sljit_jump* ioSpecial = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_IMM, 0);
		sljit_set_label(ioSpecial, helperLabel);
		// Class 1 clears haltPending (any non-constant IO read does)
		struct sljit_jump* ioConst = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_IMM, 2);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
		sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, haltPending), SLJIT_IMM,
		               0);
		sljit_set_label(ioConst, sljit_emit_label(c));
		// wait = 0 + 2, then the sub-ROM0 stall
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_IMM, info.waitBase);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_SP), LOCAL_OFFSET, SLJIT_R4, 0);
		_emitStallCall(c, cpu->dynarec, (sljit_sw) instrAddress + (isArm ? 8 : 4));
		_emitCycleTail(c, isArm, 0);
		sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R4, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_OFFSET);
		sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
		sljit_emit_op2(c, SLJIT_ADD, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_IMM,
		               (sljit_sw) offsetof(struct GBA, memory.io));
		sljit_emit_op1(c, SLJIT_MOV_U16, SLJIT_R2, 0, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 1);
		_emitValueTail(ctx, op, rd, false);
	}

	struct sljit_label* doneLabel = sljit_emit_label(c);
	sljit_set_label(doneHelper, doneLabel);
	if (doneRAM) {
		sljit_set_label(doneRAM, doneLabel);
	}
	if (doneROM) {
		sljit_set_label(doneROM, doneLabel);
	}
	ctx->inConditional = wasConditional;
}

// sljit calls take at most four arguments; pack the LSM direction into the
// mask's upper bits
static uint32_t _dynarecLoadMultiple(struct ARMCore* cpu, uint32_t address, int maskDir, int* cycleCounter) {
	return GBALoadMultiple(cpu, address, maskDir & 0xFFFF, maskDir >> 16, cycleCounter);
}

static uint32_t _dynarecStoreMultiple(struct ARMCore* cpu, uint32_t address, int maskDir, int* cycleCounter) {
	return GBAStoreMultiple(cpu, address, maskDir & 0xFFFF, maskDir >> 16, cycleCounter);
}

void GBADynarecEmitMultiple(struct ARMCore* cpu, struct ARMDynarecEmitContext* ctx, bool isLoad, uint32_t mask,
                            int baseReg, int direction, int writeback, bool isArm, uint32_t instrAddress,
                            uint32_t prefetch0, uint32_t prefetch1) {
	struct sljit_compiler* c = ctx->compiler;
	int width = isArm ? WORD_SIZE_ARM : WORD_SIZE_THUMB;
	int count = popcount32(mask);
	sljit_sw span = (sljit_sw) count * 4;
	// Start-of-transfer offset from the base register and the writeback
	// value, mirroring GBALoad/StoreMultiple's address adjustment
	sljit_sw adjConst;
	switch (direction) {
	case LSM_IA:
		adjConst = 0;
		break;
	case LSM_IB:
		adjConst = 4;
		break;
	case LSM_DA:
		adjConst = -span + 4;
		break;
	default: // LSM_DB
		adjConst = -span;
		break;
	}
	sljit_sw wbConst = (direction & LSM_D) ? -span : span;

	ARMDynarecEmitFlushCycles(ctx);
	// Transfer loops and helpers access gprs directly in memory
	ARMDynarecCacheInvalidate(ctx);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PC, SLJIT_IMM, (sljit_sw) instrAddress + 2 * width);
	// S2 = original base; R2 = adjusted (lowest) transfer address
	sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_S2, 0, SLJIT_MEM1(SLJIT_S0), OFF_GPR(baseReg));
	if (adjConst) {
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R2, 0, SLJIT_S2, 0, SLJIT_IMM, adjConst);
		sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R2, 0, SLJIT_R2, 0);
	} else {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R2, 0, SLJIT_S2, 0);
	}
	sljit_emit_op2(c, SLJIT_LSHR32, SLJIT_R3, 0, SLJIT_R2, 0, SLJIT_IMM, 24);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R3, 0, SLJIT_IMM, 2);
	struct sljit_jump* toRAM = sljit_emit_cmp(c, SLJIT_LESS_EQUAL | SLJIT_32, SLJIT_R4, 0, SLJIT_IMM, 1);

	// ---- Helper path ----
	struct sljit_label* helperLabel = sljit_emit_label(c);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH0, SLJIT_IMM, (sljit_sw) prefetch0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_PREFETCH1, SLJIT_IMM, (sljit_sw) prefetch1);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_SP), LOCAL_CC, SLJIT_IMM, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_S2, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R2, 0, SLJIT_IMM, (sljit_sw) (mask | ((uint32_t) direction << 16)));
	sljit_get_local_base(c, SLJIT_R3, 0, LOCAL_CC);
	sljit_emit_icall(c, SLJIT_CALL, SLJIT_ARGS4(32, P, 32, 32, P),
	                 SLJIT_IMM, (sljit_sw) (isLoad ? _dynarecLoadMultiple : _dynarecStoreMultiple));
	if (writeback == 1 || (writeback == 2 && !(mask & (1 << baseReg)))) {
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(baseReg), SLJIT_R0, 0);
	}
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_CC);
	_emitCycleTail(c, isArm, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_CYCLES);
	ctx->exits[ctx->nExits] =
	    sljit_emit_cmp(c, SLJIT_SIG_GREATER_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_NEXT_EVENT);
	++ctx->nExits;
	if (!isLoad) {
		sljit_emit_op1(c, SLJIT_MOV_U8, SLJIT_R0, 0, SLJIT_MEM0(), (sljit_sw) &ctx->block->valid);
		ctx->exits[ctx->nExits] = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R0, 0, SLJIT_IMM, 0);
		++ctx->nExits;
	}
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_S1, 0, SLJIT_MEM1(SLJIT_S0), isArm ? OFF_SEQ32 : OFF_SEQ16);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_S1, 0, SLJIT_S1, 0, SLJIT_IMM, 1);
	struct sljit_jump* doneHelper = sljit_emit_jump(c, SLJIT_JUMP);

	// ---- RAM fast paths ----
	sljit_set_label(toRAM, sljit_emit_label(c));
	struct sljit_jump* toIWRAM = sljit_emit_cmp(c, SLJIT_EQUAL | SLJIT_32, SLJIT_R3, 0, SLJIT_IMM, GBA_REGION_IWRAM);

	// Two span checks plus up to four bitmap tests divert to the helper
	struct sljit_jump* hits[8];
	size_t nHits = 0;

	// EWRAM
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R2, 0, SLJIT_IMM, GBA_SIZE_EWRAM - 4);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R4, 0, SLJIT_IMM, span);
	hits[nHits] = sljit_emit_cmp(c, SLJIT_GREATER | SLJIT_32, SLJIT_R0, 0, SLJIT_IMM, GBA_SIZE_EWRAM);
	++nHits;
	if (!isLoad) {
		hits[nHits] = _emitBitmapTest(c, cpu->dynarec, GBA_BASE_EWRAM >> ARM_DYNAREC_PAGE_BITS);
		++nHits;
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, span - 4);
		hits[nHits] = _emitBitmapTest(c, cpu->dynarec, GBA_BASE_EWRAM >> ARM_DYNAREC_PAGE_BITS);
		++nHits;
		sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, span - 4);
	}
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
	// wait = (seq32 - nonseq32) + count * (seq32 + 1) [+1 for loads]
	sljit_emit_op1(c, SLJIT_MOV_S8, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_R2),
	               (sljit_sw) offsetof(struct GBA, memory.waitstatesSeq32) + GBA_REGION_EWRAM);
	sljit_emit_op1(c, SLJIT_MOV_S8, SLJIT_R1, 0, SLJIT_MEM1(SLJIT_R2),
	               (sljit_sw) offsetof(struct GBA, memory.waitstatesNonseq32) + GBA_REGION_EWRAM);
	sljit_emit_op2(c, SLJIT_MUL32, SLJIT_R3, 0, SLJIT_R0, 0, SLJIT_IMM, count + 1);
	sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R3, 0, SLJIT_R3, 0, SLJIT_R1, 0);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R3, 0, SLJIT_IMM, count + (isLoad ? 1 : 0));
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, memory.wram));
	struct sljit_jump* toRAMCommon = sljit_emit_jump(c, SLJIT_JUMP);

	// IWRAM
	sljit_set_label(toIWRAM, sljit_emit_label(c));
	sljit_emit_op2(c, SLJIT_AND32, SLJIT_R4, 0, SLJIT_R2, 0, SLJIT_IMM, GBA_SIZE_IWRAM - 4);
	sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_R4, 0, SLJIT_IMM, span);
	hits[nHits] = sljit_emit_cmp(c, SLJIT_GREATER | SLJIT_32, SLJIT_R0, 0, SLJIT_IMM, GBA_SIZE_IWRAM);
	++nHits;
	if (!isLoad) {
		hits[nHits] = _emitBitmapTest(c, cpu->dynarec, GBA_BASE_IWRAM >> ARM_DYNAREC_PAGE_BITS);
		++nHits;
		// A <=36-byte span touches at most two 1KiB pages; testing both ends
		// covers it
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, span - 4);
		hits[nHits] = _emitBitmapTest(c, cpu->dynarec, GBA_BASE_IWRAM >> ARM_DYNAREC_PAGE_BITS);
		++nHits;
		sljit_emit_op2(c, SLJIT_SUB32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, span - 4);
	}
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_IMM, count + (isLoad ? 1 : 0));
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_R2), (sljit_sw) offsetof(struct GBA, memory.iwram));

	// Common RAM tail: R4 = start offset, R3 = backing, R0 = stall argument
	sljit_set_label(toRAMCommon, sljit_emit_label(c));
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_SP), LOCAL_OFFSET, SLJIT_R4, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_MEM1(SLJIT_SP), LOCAL_PTR, SLJIT_R3, 0);
	sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R1, 0, SLJIT_R0, 0);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R2, 0, SLJIT_MEM1(SLJIT_S0), OFF_MASTER);
	_emitStallCall(c, cpu->dynarec, (sljit_sw) instrAddress + 2 * width);
	_emitCycleTail(c, isArm, 0);
	sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R4, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_OFFSET);
	sljit_emit_op1(c, SLJIT_MOV, SLJIT_R3, 0, SLJIT_MEM1(SLJIT_SP), LOCAL_PTR);
	int reg;
	int word = 0;
	for (reg = 0; reg < 16; ++reg) {
		if (!(mask & (1 << reg))) {
			continue;
		}
		if (word) {
			sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R4, 0, SLJIT_R4, 0, SLJIT_IMM, 4);
			sljit_emit_op1(c, SLJIT_MOV_U32, SLJIT_R4, 0, SLJIT_R4, 0);
		}
		if (isLoad) {
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 0);
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg), SLJIT_R0, 0);
		} else {
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_R0, 0, SLJIT_MEM1(SLJIT_S0), OFF_GPR(reg));
			sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM2(SLJIT_R3, SLJIT_R4), 0, SLJIT_R0, 0);
		}
		++word;
	}
	if (writeback == 1 || (writeback == 2 && !(mask & (1 << baseReg)))) {
		sljit_emit_op2(c, SLJIT_ADD32, SLJIT_R0, 0, SLJIT_S2, 0, SLJIT_IMM, wbConst);
		sljit_emit_op1(c, SLJIT_MOV32, SLJIT_MEM1(SLJIT_S0), OFF_GPR(baseReg), SLJIT_R0, 0);
	}

	struct sljit_label* doneL = sljit_emit_label(c);
	sljit_set_label(doneHelper, doneL);
	size_t i;
	for (i = 0; i < nHits; ++i) {
		sljit_set_label(hits[i], helperLabel);
	}
}

#endif
