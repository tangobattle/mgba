/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "dynarec-internal.h"

#ifdef ENABLE_DYNAREC

#include <mgba/internal/arm/macros.h>

#include "third-party/sljit/sljit_src/sljitLir.h"

#define DYNAREC_PAGE_COUNT (1 << (ARM_DYNAREC_ADDRESS_BITS - ARM_DYNAREC_PAGE_BITS))

struct ARMDynarecPage {
	struct ARMDynarecBlock** blocks;
	size_t nBlocks;
	size_t capacity;
};

static size_t _fastIndex(uint32_t key) {
	return (key ^ (key >> ARM_DYNAREC_FAST_LOOKUP_BITS)) & ((1 << ARM_DYNAREC_FAST_LOOKUP_BITS) - 1);
}

static void _freePage(void* value) {
	struct ARMDynarecPage* page = value;
	free(page->blocks);
	free(page);
}

static void _freeBlock(struct ARMDynarecBlock* block) {
	if (block->code) {
		sljit_free_code(block->code, NULL);
	}
	free(block->sourceCopy);
	free(block->cells);
	free(block);
}

static void _setPageBit(struct ARMDynarec* dynarec, uint32_t pageIndex, bool set) {
	uint8_t* byte = &dynarec->pageBitmap[pageIndex >> 3];
	if (set) {
		*byte |= 1 << (pageIndex & 7);
	} else {
		*byte &= ~(1 << (pageIndex & 7));
	}
}

static void _unregisterBlockPages(struct ARMDynarec* dynarec, struct ARMDynarecBlock* block) {
	uint32_t first = block->canonical >> ARM_DYNAREC_PAGE_BITS;
	uint32_t last = (block->canonical + block->sourceLength - 1) >> ARM_DYNAREC_PAGE_BITS;
	uint32_t pageIndex;
	for (pageIndex = first; pageIndex <= last; ++pageIndex) {
		struct ARMDynarecPage* page = TableLookup(&dynarec->pages, pageIndex);
		if (!page) {
			continue;
		}
		size_t i;
		for (i = 0; i < page->nBlocks; ++i) {
			if (page->blocks[i] == block) {
				page->blocks[i] = page->blocks[page->nBlocks - 1];
				--page->nBlocks;
				break;
			}
		}
		if (!page->nBlocks) {
			TableRemove(&dynarec->pages, pageIndex);
			_setPageBit(dynarec, pageIndex, false);
		}
	}
}

static void _registerBlockPages(struct ARMDynarec* dynarec, struct ARMDynarecBlock* block) {
	uint32_t first = block->canonical >> ARM_DYNAREC_PAGE_BITS;
	uint32_t last = (block->canonical + block->sourceLength - 1) >> ARM_DYNAREC_PAGE_BITS;
	uint32_t pageIndex;
	for (pageIndex = first; pageIndex <= last; ++pageIndex) {
		struct ARMDynarecPage* page = TableLookup(&dynarec->pages, pageIndex);
		if (!page) {
			page = calloc(1, sizeof(*page));
			TableInsert(&dynarec->pages, pageIndex, page);
			_setPageBit(dynarec, pageIndex, true);
		}
		if (page->nBlocks == page->capacity) {
			page->capacity = page->capacity ? page->capacity * 2 : 4;
			page->blocks = realloc(page->blocks, page->capacity * sizeof(*page->blocks));
		}
		page->blocks[page->nBlocks] = block;
		++page->nBlocks;
	}
}

// Severs link cells in both directions: cells of other blocks resolving to
// this block, and this block's own cells from the registry
static void _unregisterLinks(struct ARMDynarec* dynarec, struct ARMDynarecBlock* block) {
	struct ARMDynarecLinkCell* cell = TableLookup(&dynarec->links, block->key);
	for (; cell; cell = cell->next) {
		if (cell->target == block) {
			cell->target = NULL;
		}
	}
	size_t i;
	for (i = 0; i < block->nCells; ++i) {
		struct ARMDynarecLinkCell* own = &block->cells[i];
		struct ARMDynarecLinkCell* head = TableLookup(&dynarec->links, own->targetKey);
		if (head == own) {
			if (own->next) {
				TableInsert(&dynarec->links, own->targetKey, own->next);
			} else {
				TableRemove(&dynarec->links, own->targetKey);
			}
		} else {
			struct ARMDynarecLinkCell* prev = head;
			while (prev && prev->next != own) {
				prev = prev->next;
			}
			if (prev) {
				prev->next = own->next;
			}
		}
	}
}

// Unlinks the block from all indexes; frees it unless it is mid-execution
static void _retireBlock(struct ARMCore* cpu, struct ARMDynarecBlock* block) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	block->valid = false;
	_unregisterLinks(dynarec, block);
	_unregisterBlockPages(dynarec, block);
	if (block->fastSlot >= 0 && dynarec->fastLookup[block->fastSlot] == block) {
		dynarec->fastLookup[block->fastSlot] = NULL;
	}
	block->fastSlot = -1;
	TableRemove(&dynarec->blocks, block->key);
	if (block == dynarec->currentBlock) {
		// Replacing an older deferred retire is safe: control has already
		// link-jumped out of it, so its code is no longer on the stack
		if (dynarec->pendingRetire) {
			_freeBlock(dynarec->pendingRetire);
		}
		dynarec->pendingRetire = block;
	} else {
		_freeBlock(block);
	}
	++dynarec->blocksInvalidated;
}

// Caches the block for raw-address dispatch; a block occupies at most one
// slot so retirement can clear it in O(1) without dangling aliases
static void _fastInsert(struct ARMDynarec* dynarec, struct ARMDynarecBlock* block, uint32_t rawKey) {
	if (block->fastSlot >= 0 && dynarec->fastLookup[block->fastSlot] == block) {
		dynarec->fastLookup[block->fastSlot] = NULL;
	}
	size_t slot = _fastIndex(rawKey);
	block->rawKey = rawKey;
	block->fastSlot = slot;
	dynarec->fastLookup[slot] = block;
}

void ARMDynarecRegisterBlock(struct ARMCore* cpu, struct ARMDynarecBlock* block) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	TableInsert(&dynarec->blocks, block->key, block);
	_registerBlockPages(dynarec, block);
	// Register this block's outgoing link cells and resolve them against
	// already-compiled targets
	size_t i;
	for (i = 0; i < block->nCells; ++i) {
		struct ARMDynarecLinkCell* cell = &block->cells[i];
		cell->owner = block;
		cell->next = TableLookup(&dynarec->links, cell->targetKey);
		TableInsert(&dynarec->links, cell->targetKey, cell);
		cell->target = TableLookup(&dynarec->blocks, cell->targetKey);
	}
	// Resolve other blocks' cells waiting for this one
	struct ARMDynarecLinkCell* cell = TableLookup(&dynarec->links, block->key);
	for (; cell; cell = cell->next) {
		cell->target = block;
	}
	++dynarec->blocksCompiled;
}

void ARMDynarecInit(struct ARMCore* cpu) {
	if (cpu->dynarec) {
		return;
	}
	struct ARMDynarec* dynarec = calloc(1, sizeof(*dynarec));
	TableInit(&dynarec->blocks, 0x400, NULL);
	TableInit(&dynarec->pages, 0x100, _freePage);
	TableInit(&dynarec->links, 0x100, NULL);
	dynarec->pageBitmap = calloc(1, DYNAREC_PAGE_COUNT / 8);
	dynarec->maxBlockLength = 32;
	dynarec->ramGeneration = 1;
	cpu->dynarec = dynarec;
}

static void _flushHandler(uint32_t key, void* value, void* user) {
	UNUSED(key);
	struct ARMDynarec* dynarec = user;
	struct ARMDynarecBlock* block = value;
	block->valid = false;
	if (block == dynarec->currentBlock) {
		dynarec->pendingRetire = block;
	} else {
		_freeBlock(block);
	}
}

void ARMDynarecFlush(struct ARMCore* cpu) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	if (!dynarec) {
		return;
	}
	TableEnumerate(&dynarec->blocks, _flushHandler, dynarec);
	TableClear(&dynarec->blocks);
	TableClear(&dynarec->pages);
	TableClear(&dynarec->links);
	memset(dynarec->fastLookup, 0, sizeof(dynarec->fastLookup));
	memset(dynarec->pageBitmap, 0, DYNAREC_PAGE_COUNT / 8);
}

void ARMDynarecDeinit(struct ARMCore* cpu) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	if (!dynarec) {
		return;
	}
	ARMDynarecFlush(cpu);
	if (dynarec->pendingRetire) {
		_freeBlock(dynarec->pendingRetire);
		dynarec->pendingRetire = NULL;
	}
	if (dynarec->platformCode && dynarec->platformCodeFree) {
		dynarec->platformCodeFree(dynarec->platformCode);
	}
	TableDeinit(&dynarec->blocks);
	TableDeinit(&dynarec->pages);
	TableDeinit(&dynarec->links);
	free(dynarec->pageBitmap);
	free(dynarec);
	cpu->dynarec = NULL;
}

void ARMDynarecInvalidateRange(struct ARMCore* cpu, uint32_t canonical, uint32_t length) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	if (!dynarec) {
		return;
	}
	uint32_t first = canonical >> ARM_DYNAREC_PAGE_BITS;
	uint32_t last = (canonical + length - 1) >> ARM_DYNAREC_PAGE_BITS;
	uint32_t pageIndex;
	for (pageIndex = first; pageIndex <= last; ++pageIndex) {
		struct ARMDynarecPage* page = TableLookup(&dynarec->pages, pageIndex);
		if (!page) {
			_setPageBit(dynarec, pageIndex, false);
			continue;
		}
		// Only retire blocks whose source bytes actually overlap the write;
		// data commonly shares pages with code and must not thrash it.
		// Retiring swap-removes from this page's array (and frees the page
		// when emptied), so re-look the page up and hold the index
		size_t i = 0;
		while (i < page->nBlocks) {
			struct ARMDynarecBlock* block = page->blocks[i];
			if (block->canonical < canonical + length && canonical < block->canonical + block->sourceLength) {
				_retireBlock(cpu, block);
				page = TableLookup(&dynarec->pages, pageIndex);
				if (!page) {
					break;
				}
			} else {
				++i;
			}
		}
	}
}

void ARMDynarecOnLoadState(struct ARMCore* cpu) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	if (!dynarec) {
		return;
	}
	++dynarec->ramGeneration;
}

// Executes one dispatch step: one compiled block, or one interpreted
// instruction when the address is uncacheable or the pipeline is incoherent.
// Does not process events; mirrors one iteration of ARMRunLoop's body.
void ARMDynarecStep(struct ARMCore* cpu) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	{
		enum ExecutionMode mode = cpu->executionMode;
		uint32_t width = mode == MODE_THUMB ? WORD_SIZE_THUMB : WORD_SIZE_ARM;
		// gprs[PC] points one fetch ahead; the next instruction to execute
		// is the one currently in prefetch[0]
		uint32_t address = cpu->gprs[ARM_PC] - width;
		uint32_t rawKey = address | (mode == MODE_THUMB ? 1 : 0);
		// Pipeline coherence gate: only enter compiled code if the prefetch
		// pipeline matches a natural refetch from PC via the current
		// activeRegion, mirroring ThumbStep/ARMStep's fetch exactly.
		// activeRegion itself is maintained by the interpreter handlers
		// (WritePC) that the compiled callouts also run; calling
		// setActiveRegion here would clobber the cartridge prefetch-unit
		// state (lastPrefetchedPc) and skew cycle accounting. ARMRunFake
		// (Tango traps, cheat hooks) and self-modifying stores leave a
		// pipeline that diverges from memory; the interpreter handles
		// those steps.
		uint32_t expected0;
		uint32_t expected1;
		if (mode == MODE_THUMB) {
			LOAD_16(expected0, address & cpu->memory.activeMask, cpu->memory.activeRegion);
			LOAD_16(expected1, (address + WORD_SIZE_THUMB) & cpu->memory.activeMask, cpu->memory.activeRegion);
		} else {
			LOAD_32(expected0, address & cpu->memory.activeMask, cpu->memory.activeRegion);
			LOAD_32(expected1, (address + WORD_SIZE_ARM) & cpu->memory.activeMask, cpu->memory.activeRegion);
		}
		if (cpu->prefetch[0] != expected0 || cpu->prefetch[1] != expected1) {
			ARMRunOne(cpu);
			return;
		}
		struct ARMDynarecBlock* block = dynarec->fastLookup[_fastIndex(rawKey)];
		if (block && block->rawKey != rawKey) {
			block = NULL;
		}
		if (block && block->writable && block->generation != dynarec->ramGeneration) {
			if (memcmp(block->sourcePtr, block->sourceCopy, block->sourceLength) == 0) {
				block->generation = dynarec->ramGeneration;
				++dynarec->blocksRevalidated;
			} else {
				_retireBlock(cpu, block);
				block = NULL;
			}
		}
		if (!block) {
			// Slow path: canonicalize for the block table (folds mirrors)
			uint32_t canonical = dynarec->mapAddress(cpu, address);
			if (canonical == ARM_DYNAREC_UNCACHEABLE) {
				ARMRunOne(cpu);
				return;
			}
			uint32_t key = canonical | (mode == MODE_THUMB ? 1 : 0);
			block = TableLookup(&dynarec->blocks, key);
			if (block && block->writable && block->generation != dynarec->ramGeneration) {
				if (memcmp(block->sourcePtr, block->sourceCopy, block->sourceLength) == 0) {
					block->generation = dynarec->ramGeneration;
					++dynarec->blocksRevalidated;
				} else {
					_retireBlock(cpu, block);
					block = NULL;
				}
			}
			if (!block) {
				block = ARMDynarecCompile(cpu, address, canonical, mode);
				if (!block) {
					ARMRunOne(cpu);
					return;
				}
			}
			_fastInsert(dynarec, block, rawKey);
		}
		dynarec->currentBlock = block;
		block->entry(cpu);
		dynarec->currentBlock = NULL;
		if (dynarec->pendingRetire) {
			_freeBlock(dynarec->pendingRetire);
			dynarec->pendingRetire = NULL;
		}
	}
}

void ARMDynarecRunLoop(struct ARMCore* cpu) {
	while (cpu->cycles < cpu->nextEvent) {
		ARMDynarecStep(cpu);
	}
	cpu->irqh.processEvents(cpu);
}

#endif
