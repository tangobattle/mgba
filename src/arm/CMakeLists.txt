include(ExportDirectory)
set(SOURCE_FILES
	arm.c
	decoder-arm.c
	decoder.c
	decoder-thumb.c
	isa-arm.c
	isa-thumb.c)

set(DEBUGGER_FILES
	debugger/cli-debugger.c
	debugger/debugger.c
	debugger/memory-debugger.c)

set(DYNAREC_FILES
	dynarec/dynarec.c
	dynarec/frontend.c
	dynarec/emit.c
	dynarec/emit-arm.c
	dynarec/emit-thumb.c)

source_group("ARM core" FILES ${SOURCE_FILES})
source_group("ARM debugger" FILES ${DEBUGGER_FILES})

export_directory(ARM SOURCE_FILES)
export_directory(ARM_DEBUGGER DEBUGGER_FILES)
export_directory(ARM_DYNAREC DYNAREC_FILES)