/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "dynarec-internal.h"

#ifdef ENABLE_DYNAREC

#include <mgba/internal/arm/decoder.h>
#include <mgba/internal/arm/macros.h>

// Whether compilation may continue past this instruction. Ending blocks is
// purely a throughput decision: every callout is followed by a PC check that
// exits the block if the handler branched, so a "missed" end is still correct.
// Conditional instructions never end a block: their not-taken path falls
// through, and the taken path exits via the post-callout PC check.
static bool _endsBlock(const struct ARMInstructionInfo* info, uint32_t opcode, enum ExecutionMode mode) {
	if (mode == MODE_ARM && (opcode >> 28) != ARM_CONDITION_AL) {
		return false;
	}
	if (mode == MODE_THUMB && (opcode & 0xF000) == 0xD000 && (opcode & 0x0F00) < 0x0E00) {
		// Conditional branch
		return false;
	}
	if (mode == MODE_THUMB && (opcode & 0xF800) == 0xF000) {
		// BL prefix only sets LR; the suffix does the branching
		return false;
	}
	if (info->branchType != ARM_BRANCH_NONE) {
		return true;
	}
	if (info->traps) {
		return true;
	}
	switch (info->mnemonic) {
	case ARM_MN_B:
	case ARM_MN_BL:
	case ARM_MN_BX:
	case ARM_MN_SWI:
	case ARM_MN_BKPT:
	case ARM_MN_ILL:
	case ARM_MN_MSR:
		return true;
	default:
		break;
	}
	// Any operand that writes r15
	if ((info->operandFormat & ARM_OPERAND_AFFECTED_1) && (info->operandFormat & ARM_OPERAND_REGISTER_1) &&
	    info->op1.reg == ARM_PC) {
		return true;
	}
	// LDM/STM with r15 in the register list
	if (info->mnemonic == ARM_MN_LDM && (info->opcode & (1 << 15)) && info->execMode == MODE_ARM) {
		return true;
	}
	if (info->mnemonic == ARM_MN_LDM && info->execMode == MODE_THUMB && (info->opcode & 0x0100) &&
	    (info->opcode & 0xFE00) == 0xBC00) {
		// Thumb POP with PC
		return true;
	}
	return false;
}

static void _classify(const struct ARMInstructionInfo* info, struct ARMDynarecInsn* insn) {
	insn->isMemory = false;
	insn->isStore = false;
	insn->alwaysExit = false;
	switch (info->mnemonic) {
	case ARM_MN_LDM:
	case ARM_MN_LDR:
		insn->isMemory = true;
		break;
	case ARM_MN_STM:
	case ARM_MN_STR:
	case ARM_MN_SWP:
		insn->isMemory = true;
		insn->isStore = true;
		break;
	case ARM_MN_SWI:
	case ARM_MN_BKPT:
	case ARM_MN_ILL:
	case ARM_MN_MSR:
		insn->alwaysExit = true;
		break;
	default:
		if (info->traps) {
			insn->alwaysExit = true;
		}
		break;
	}
}

struct ARMDynarecBlock* ARMDynarecCompile(struct ARMCore* cpu, uint32_t address, uint32_t canonical,
                                          enum ExecutionMode mode) {
	struct ARMDynarec* dynarec = cpu->dynarec;
	uint32_t width = mode == MODE_THUMB ? WORD_SIZE_THUMB : WORD_SIZE_ARM;
	const uint32_t* region = cpu->memory.activeRegion;
	uint32_t mask = cpu->memory.activeMask;
	uint32_t offset = address & mask;

	// The block needs its instructions plus two lookahead words for the
	// prefetch pipeline, all within the active region (no mask wrap)
	uint32_t remaining = mask + 1 - offset;
	if (remaining < 3 * width) {
		return NULL;
	}
	unsigned maxInsns = remaining / width - 2;
	if (maxInsns > dynarec->maxBlockLength) {
		maxInsns = dynarec->maxBlockLength;
	}

	struct ARMDynarecInsn insns[ARM_DYNAREC_MAX_BLOCK_LENGTH];
	struct ARMInstructionInfo info;
	unsigned count = 0;
	while (count < maxInsns) {
		struct ARMDynarecInsn* insn = &insns[count];
		insn->address = address + count * width;
		if (mode == MODE_THUMB) {
			LOAD_16(insn->opcode, (offset + count * width) & mask, region);
			ARMDecodeThumb(insn->opcode, &info);
		} else {
			LOAD_32(insn->opcode, (offset + count * width) & mask, region);
			ARMDecodeARM(insn->opcode, &info);
		}
		insn->isBranchTarget = false;
		_classify(&info, insn);
		++count;
		if (_endsBlock(&info, insn->opcode, mode)) {
			break;
		}
	}

	// Mark intra-block branch targets so the emitter can place
	// cycle-settled labels for native loop back-edges
	unsigned i;
	for (i = 0; i < count; ++i) {
		uint32_t opcode = insns[i].opcode;
		int64_t target = -1;
		if (mode == MODE_THUMB) {
			if ((opcode & 0xF000) == 0xD000 && (opcode & 0x0F00) < 0x0E00) {
				target = insns[i].address + 4 + (((int32_t)(int8_t)(opcode & 0xFF)) << 1);
			} else if ((opcode & 0xF800) == 0xE000) {
				target = insns[i].address + 4 + ((((int32_t)(opcode & 0x07FF)) << 21) >> 20);
			}
		} else if ((opcode & 0x0F000000) == 0x0A000000 && (opcode >> 28) != 0xF) {
			// ARM B (not BL, not NV)
			target = insns[i].address + 8 + ((((int32_t)(opcode & 0x00FFFFFF)) << 8) >> 6);
		}
		if (target >= 0 && (uint64_t) target >= address && (uint64_t) target < address + count * width) {
			insns[((uint32_t) target - address) / width].isBranchTarget = true;
		}
	}

	uint32_t lookahead[2];
	if (mode == MODE_THUMB) {
		LOAD_16(lookahead[0], (offset + count * width) & mask, region);
		LOAD_16(lookahead[1], (offset + (count + 1) * width) & mask, region);
	} else {
		LOAD_32(lookahead[0], (offset + count * width) & mask, region);
		LOAD_32(lookahead[1], (offset + (count + 1) * width) & mask, region);
	}

	struct ARMDynarecBlock* block = calloc(1, sizeof(*block));
	block->key = canonical | (mode == MODE_THUMB ? 1 : 0);
	block->fastSlot = -1;
	block->canonical = canonical;
	block->mode = mode;
	block->sourceLength = (count + 2) * width;
	block->generation = dynarec->ramGeneration;
	block->valid = true;
	block->writable = dynarec->isWritable(cpu, canonical);
	block->sourcePtr = (const uint8_t*) region + offset;
	block->cells = calloc(count, sizeof(*block->cells));
	block->nCells = 0;
	if (block->writable) {
		block->sourceCopy = malloc(block->sourceLength);
		memcpy(block->sourceCopy, block->sourcePtr, block->sourceLength);
	}

	if (!ARMDynarecEmitBlock(cpu, block, insns, count, lookahead)) {
		free(block->sourceCopy);
		free(block->cells);
		free(block);
		return NULL;
	}

	ARMDynarecRegisterBlock(cpu, block);
	return block;
}

#endif
