/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Lockstep differential tester: runs the same ROM on an interpreter core and
// a dynarec core restricted to single-instruction blocks (which makes its
// event timing identical to the interpreter), comparing full CPU state after
// every run-loop slice and memory checksums periodically. Any divergence is
// a dynarec bug. Optionally exercises the Tango pattern of savestate
// save/load roundtrips on the dynarec core mid-run.

#include <mgba/core/config.h>
#include <mgba/core/core.h>
#include <mgba/core/serialize.h>
#include <mgba/gba/core.h>
#include <mgba/internal/arm/arm.h>
#include <mgba/internal/arm/dynarec/dynarec.h>
#include <mgba/internal/gba/gba.h>
#include <mgba-util/crc32.h>
#include <mgba-util/vfs.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ENABLE_DYNAREC
#error dynarec-diff requires ENABLE_DYNAREC
#endif

static struct mCore* _createCore(const char* rom, bool dynarec, unsigned maxBlockLength) {
	struct mCore* core = GBACoreCreate();
	if (!core->init(core)) {
		return NULL;
	}
	mCoreInitConfig(core, "dynarec-diff");
	mCoreConfigSetDefaultValue(&core->config, "idleOptimization", "ignore");
	if (dynarec) {
		mCoreConfigSetDefaultValue(&core->config, "cpu.dynarec", "1");
		if (maxBlockLength) {
			char buf[16];
			snprintf(buf, sizeof(buf), "%u", maxBlockLength);
			mCoreConfigSetDefaultValue(&core->config, "cpu.dynarec.maxBlockLength", buf);
		}
	}
	if (!mCoreLoadFile(core, rom)) {
		fprintf(stderr, "failed to load ROM: %s\n", rom);
		return NULL;
	}
	core->loadConfig(core, &core->config);
	core->reset(core);
	return core;
}

static bool _compareCPU(struct ARMCore* a, struct ARMCore* b, uint64_t slice) {
	bool ok = true;
	int i;
	for (i = 0; i < 16; ++i) {
		if (a->gprs[i] != b->gprs[i]) {
			fprintf(stderr, "slice %" PRIu64 ": r%d mismatch: %08X != %08X\n", slice, i, a->gprs[i], b->gprs[i]);
			ok = false;
		}
	}
	if (a->cpsr.packed != b->cpsr.packed) {
		fprintf(stderr, "slice %" PRIu64 ": cpsr mismatch: %08X != %08X\n", slice, a->cpsr.packed, b->cpsr.packed);
		ok = false;
	}
	if (a->spsr.packed != b->spsr.packed) {
		fprintf(stderr, "slice %" PRIu64 ": spsr mismatch: %08X != %08X\n", slice, a->spsr.packed, b->spsr.packed);
		ok = false;
	}
	if (a->cycles != b->cycles || a->nextEvent != b->nextEvent) {
		fprintf(stderr, "slice %" PRIu64 ": cycles mismatch: %d/%d != %d/%d\n", slice, a->cycles, a->nextEvent,
		        b->cycles, b->nextEvent);
		ok = false;
	}
	if (a->prefetch[0] != b->prefetch[0] || a->prefetch[1] != b->prefetch[1]) {
		fprintf(stderr, "slice %" PRIu64 ": prefetch mismatch: %08X,%08X != %08X,%08X\n", slice, a->prefetch[0],
		        a->prefetch[1], b->prefetch[0], b->prefetch[1]);
		ok = false;
	}
	if (a->executionMode != b->executionMode || a->halted != b->halted) {
		fprintf(stderr, "slice %" PRIu64 ": mode/halt mismatch\n", slice);
		ok = false;
	}
	return ok;
}

static bool _compareMemory(struct mCore* coreA, struct mCore* coreB, uint64_t slice) {
	struct GBA* a = coreA->board;
	struct GBA* b = coreB->board;
	struct {
		const char* name;
		const void* memA;
		const void* memB;
		size_t size;
	} regions[] = {
		{ "EWRAM", a->memory.wram, b->memory.wram, GBA_SIZE_EWRAM },
		{ "IWRAM", a->memory.iwram, b->memory.iwram, GBA_SIZE_IWRAM },
		{ "VRAM", a->video.vram, b->video.vram, GBA_SIZE_VRAM },
		{ "OAM", a->video.oam.raw, b->video.oam.raw, GBA_SIZE_OAM },
		{ "Palette", a->video.palette, b->video.palette, GBA_SIZE_PALETTE_RAM },
	};
	bool ok = true;
	size_t i;
	for (i = 0; i < sizeof(regions) / sizeof(regions[0]); ++i) {
		uint32_t crcA = doCrc32(regions[i].memA, regions[i].size);
		uint32_t crcB = doCrc32(regions[i].memB, regions[i].size);
		if (crcA != crcB) {
			fprintf(stderr, "slice %" PRIu64 ": %s checksum mismatch: %08X != %08X\n", slice, regions[i].name, crcA,
			        crcB);
			ok = false;
		}
	}
	return ok;
}

int main(int argc, char** argv) {
	uint64_t maxSlices = 2000000;
	uint64_t stateInterval = 0;
	uint64_t traceStart = UINT64_MAX;
	bool fuzzInput = false;
	bool jitVsJit = false;
	int opt;
	while ((opt = getopt(argc, argv, "n:s:t:ij")) != -1) {
		switch (opt) {
		case 'n':
			maxSlices = strtoull(optarg, NULL, 0);
			break;
		case 's':
			stateInterval = strtoull(optarg, NULL, 0);
			break;
		case 't':
			traceStart = strtoull(optarg, NULL, 0);
			break;
		case 'i':
			fuzzInput = true;
			break;
		case 'j':
			// Two independent dynarec cores at the default block length:
			// verifies loose-mode self-consistency (intra-block branches,
			// per-instance baked pointers) rather than interpreter parity
			jitVsJit = true;
			break;
		default:
			fprintf(stderr, "usage: %s [-n slices] [-s stateInterval] [-t traceSlice] [-i] rom\n", argv[0]);
			return 2;
		}
	}
	if (optind >= argc) {
		fprintf(stderr, "usage: %s [-n slices] [-s stateInterval] [-t traceSlice] [-i] rom\n", argv[0]);
		return 2;
	}
	const char* rom = argv[optind];

	struct mCore* coreA = _createCore(rom, jitVsJit, 0);
	struct mCore* coreB = _createCore(rom, true, jitVsJit ? 0 : 1);
	if (!coreA || !coreB) {
		return 2;
	}
	struct ARMCore* cpuA = coreA->cpu;
	struct ARMCore* cpuB = coreB->cpu;
	if (!cpuB->dynarec) {
		fprintf(stderr, "dynarec failed to enable\n");
		return 2;
	}

	void* state = NULL;
	if (stateInterval) {
		state = malloc(coreB->stateSize(coreB));
	}

	uint32_t lfsr = 0xACE1u;
	uint64_t slice;
	bool ok = true;
	for (slice = 0; slice < maxSlices && ok; ++slice) {
		if (fuzzInput && (slice & 0x3FF) == 0) {
			lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
			uint32_t keys = lfsr & 0x3FF;
			coreA->setKeys(coreA, keys);
			coreB->setKeys(coreB, keys);
		}
		if (slice >= traceStart) {
			// Instruction-level lockstep with per-step trace
			int step = 0;
			while (ok && (cpuA->cycles < cpuA->nextEvent || cpuB->cycles < cpuB->nextEvent)) {
				bool stepA = cpuA->cycles < cpuA->nextEvent;
				bool stepB = cpuB->cycles < cpuB->nextEvent;
				if (stepA != stepB) {
					fprintf(stderr, "slice %" PRIu64 " step %d: slice length mismatch (A=%d B=%d)\n", slice, step,
					        stepA, stepB);
					ok = false;
					break;
				}
				ARMRunOne(cpuA);
				ARMDynarecStep(cpuB);
				struct GBA* gbaA = coreA->board;
				struct GBA* gbaB = coreB->board;
				fprintf(stderr,
				        "T %" PRIu64 ":%d A pc=%08X cyc=%d/%d lpp=%08X bp=%08X t=%08X r1=%08X sm=%04X vw=%08X dc=%04X | B pc=%08X cyc=%d/%d "
				        "lpp=%08X bp=%08X t=%08X r1=%08X sm=%04X vw=%08X dc=%04X\n",
				        slice, step, cpuA->gprs[ARM_PC], cpuA->cycles, cpuA->nextEvent, gbaA->memory.lastPrefetchedPc,
				        gbaA->memory.biosPrefetch, mTimingCurrentTime(coreA->timing), cpuA->gprs[1],
				        gbaA->video.stallMask, gbaA->video.event.when, gbaA->memory.io[0],
				        cpuB->gprs[ARM_PC], cpuB->cycles,
				        cpuB->nextEvent, gbaB->memory.lastPrefetchedPc, gbaB->memory.biosPrefetch,
				        mTimingCurrentTime(coreB->timing), cpuB->gprs[1],
				        gbaB->video.stallMask, gbaB->video.event.when, gbaB->memory.io[0]);
				ok = _compareCPU(cpuA, cpuB, slice) && ok;
				++step;
			}
			cpuA->irqh.processEvents(cpuA);
			cpuB->irqh.processEvents(cpuB);
		} else {
			if (jitVsJit) {
				ARMDynarecRunLoop(cpuA);
			} else {
				ARMRunLoop(cpuA);
			}
			ARMDynarecRunLoop(cpuB);
		}
		ok = _compareCPU(cpuA, cpuB, slice) && ok;
		if (ok && (slice % 50000) == 0) {
			ok = _compareMemory(coreA, coreB, slice);
		}
		if (ok && stateInterval && (slice % stateInterval) == 0) {
			// Tango pattern: savestate roundtrip. Applied to both cores:
			// derived state that mGBA recomputes on load (e.g. the VRAM
			// stall mask) is not byte-identical to live state, so a
			// one-sided roundtrip would diverge even interpreter-vs-
			// interpreter. Tango resimulates from loaded states on both
			// ends, so symmetric roundtrips match its usage.
			if (!coreA->saveState(coreA, state) || !coreA->loadState(coreA, state) ||
			    !coreB->saveState(coreB, state) || !coreB->loadState(coreB, state)) {
				fprintf(stderr, "slice %" PRIu64 ": savestate roundtrip failed\n", slice);
				ok = false;
			}
		}
	}
	free(state);

	if (!ok) {
		uint32_t pc = cpuA->gprs[ARM_PC];
		fprintf(stderr, "diverged near pc %08X after %" PRIu64 " slices\n", pc, slice);
		return 1;
	}
	ok = _compareMemory(coreA, coreB, slice);

	struct ARMDynarec* dynarec = cpuB->dynarec;
	printf("OK: %" PRIu64 " slices, %u blocks compiled, %u invalidated, %u revalidated\n", slice,
	       dynarec->blocksCompiled, dynarec->blocksInvalidated, dynarec->blocksRevalidated);
	return ok ? 0 : 1;
}
