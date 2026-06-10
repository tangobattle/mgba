/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef ARM_DYNAREC_H
#define ARM_DYNAREC_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/internal/arm/arm.h>

#ifdef ENABLE_DYNAREC

#include <mgba-util/table.h>

// Returned by mapAddress for addresses that code may not be compiled from
// (IO, SRAM, open bus). Stores to such addresses never invalidate code.
#define ARM_DYNAREC_UNCACHEABLE 0xFFFFFFFF

// Compiled code spans are tracked at this granularity for store invalidation
#define ARM_DYNAREC_PAGE_BITS 10
// Canonical addresses fit in the 28-bit physical bus
#define ARM_DYNAREC_ADDRESS_BITS 28

#define ARM_DYNAREC_MAX_BLOCK_LENGTH 64
#define ARM_DYNAREC_FAST_LOOKUP_BITS 11

// One per static taken-branch exit: holds the resolved target block (NULL
// when the target is not compiled), kept registered in the dynarec's link
// registry for its owner's lifetime so retiring a target can sever it
struct ARMDynarecLinkCell {
	struct ARMDynarecBlock* target;
	uint32_t targetKey;
	struct ARMDynarecLinkCell* next;
	struct ARMDynarecBlock* owner;
};

struct ARMDynarecBlock {
	uint32_t key;
	// Raw (uncanonicalized) dispatch key currently caching this block, and
	// the fast-lookup slot it occupies (-1 if none)
	uint32_t rawKey;
	int32_t fastSlot;
	uint32_t canonical;
	uint32_t sourceLength;
	uint32_t generation;
	enum ExecutionMode mode;
	bool valid;
	bool writable;
	void (*entry)(struct ARMCore*);
	// Past the prologue; cross-block links jump here (frames are identical)
	void* entryInner;
	void* code;
	void* sourceCopy;
	const void* sourcePtr;
	struct ARMDynarecLinkCell* cells;
	size_t nCells;
};

// Memory operations the platform may emit natively (see emitThumbMemory)
enum ARMDynarecMemOp {
	ARM_DYNAREC_MEM_LOAD32,
	ARM_DYNAREC_MEM_LOAD16,
	ARM_DYNAREC_MEM_LOAD8,
	ARM_DYNAREC_MEM_LOADS16,
	ARM_DYNAREC_MEM_LOADS8,
	ARM_DYNAREC_MEM_STORE32,
	ARM_DYNAREC_MEM_STORE16,
	ARM_DYNAREC_MEM_STORE8,
};

struct ARMDynarecEmitContext;

struct ARMDynarec {
	struct Table blocks;
	struct Table pages;
	// targetKey -> intrusive list of link cells wanting that block
	struct Table links;
	struct ARMDynarecBlock* fastLookup[1 << ARM_DYNAREC_FAST_LOOKUP_BITS];
	uint8_t* pageBitmap;
	uint32_t ramGeneration;
	unsigned maxBlockLength;
	struct ARMDynarecBlock* currentBlock;
	struct ARMDynarecBlock* pendingRetire;

	// Maps a guest address to a mirror-canonical address used as the block
	// cache key and the page bitmap index, or ARM_DYNAREC_UNCACHEABLE.
	// Set by the platform after ARMDynarecInit.
	uint32_t (*mapAddress)(struct ARMCore*, uint32_t address);
	// Whether guest stores can reach this canonical address; non-writable
	// blocks skip generation revalidation and rely on explicit invalidation
	bool (*isWritable)(struct ARMCore*, uint32_t canonical);
	// Host pointer cleared on natively-emitted taken branches, mirroring
	// what the platform's setActiveRegion does on every jump (the GBA
	// cartridge prefetch unit state). May be NULL.
	uint32_t* branchPrefetchReset;
	// Emits native code for a guest memory access whose address has already
	// been computed into SLJIT_R1 (cycles flushed by the implementation).
	// The platform supplies this to inline its fast memory regions; when
	// NULL, memory instructions fall back to interpreter callouts.
	void (*emitMemory)(struct ARMCore*, struct ARMDynarecEmitContext*, enum ARMDynarecMemOp, int rd, bool isArm,
	                   uint32_t instrAddress, uint32_t prefetch0, uint32_t prefetch1);
	// Emits a native LDM/STM-class transfer (callers pre-filter PC lists,
	// empty masks, and S-bit forms to interpreter callouts). writeback:
	// 0 = none, 1 = always, 2 = skip when the base is in the mask
	void (*emitMultiple)(struct ARMCore*, struct ARMDynarecEmitContext*, bool isLoad, uint32_t mask, int baseReg,
	                     int direction, int writeback, bool isArm, uint32_t instrAddress, uint32_t prefetch0,
	                     uint32_t prefetch1);

	uint32_t blocksCompiled;
	uint32_t blocksInvalidated;
	uint32_t blocksRevalidated;

	// Platform-owned generated code (shared helper stubs), freed at deinit
	void* platformCode;
	void (*platformCodeFree)(void* code);
};

void ARMDynarecInit(struct ARMCore* cpu);
void ARMDynarecDeinit(struct ARMCore* cpu);

void ARMDynarecRunLoop(struct ARMCore* cpu);
// One dispatch step (one block or one interpreted instruction), no event
// processing; intended for debugging and differential testing
void ARMDynarecStep(struct ARMCore* cpu);

void ARMDynarecFlush(struct ARMCore* cpu);
// Takes canonical addresses (as produced by mapAddress)
void ARMDynarecInvalidateRange(struct ARMCore* cpu, uint32_t canonical, uint32_t length);
void ARMDynarecOnLoadState(struct ARMCore* cpu);

static inline bool ARMDynarecPageHasCode(const struct ARMDynarec* dynarec, uint32_t canonical) {
	return dynarec->pageBitmap[canonical >> (ARM_DYNAREC_PAGE_BITS + 3)] & (1 << ((canonical >> ARM_DYNAREC_PAGE_BITS) & 7));
}

// Fast store-path hook; canonical must already be mirror-canonicalized
static inline void ARMDynarecNotifyStore(struct ARMCore* cpu, uint32_t canonical, uint32_t length) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	if (UNLIKELY(dynarec && ARMDynarecPageHasCode(dynarec, canonical))) {
		ARMDynarecInvalidateRange(cpu, canonical, length);
	}
}

#else

static inline void ARMDynarecDeinit(struct ARMCore* cpu) { UNUSED(cpu); }
static inline void ARMDynarecFlush(struct ARMCore* cpu) { UNUSED(cpu); }
static inline void ARMDynarecInvalidateRange(struct ARMCore* cpu, uint32_t canonical, uint32_t length) {
	UNUSED(cpu);
	UNUSED(canonical);
	UNUSED(length);
}
static inline void ARMDynarecOnLoadState(struct ARMCore* cpu) { UNUSED(cpu); }
static inline void ARMDynarecNotifyStore(struct ARMCore* cpu, uint32_t canonical, uint32_t length) {
	UNUSED(cpu);
	UNUSED(canonical);
	UNUSED(length);
}

#endif

CXX_GUARD_END

#endif
