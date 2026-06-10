/* Copyright (c) 2013-2016 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef GBA_CORE_H
#define GBA_CORE_H

#include <mgba-util/common.h>

CXX_GUARD_START

struct mCore;
struct mCore* GBACoreCreate(void);
// Enables or disables the CPU dynarec directly, for embedders that manage
// cores without the mCoreConfig flow (equivalent to the "cpu.dynarec"
// config option). No-op when the dynarec is compiled out or a debugger is
// in use; safe to call at any point outside the run loop.
void GBACoreEnableDynarec(struct mCore* core, bool enable);
#ifndef MINIMAL_CORE
struct mCore* GBAVideoLogPlayerCreate(void);
#endif

CXX_GUARD_END

#endif
