/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gba/sio/wireless.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>

// The wireless adapter driver: see the architecture note in wireless.h.
//
// The synchronization core (shared clock, event queues, parking) is
// deliberately a close copy of lockstep.c so the cooperative
// single-thread fixes there — the caught-up-secondary parking branch and
// the reschedule-underflow clamp — carry over by construction.
//
// The adapter protocol implemented here follows Nintendo's librfu as
// ground truth (the pret decompilations), cross-checked against the
// gba-link-connection driver and its wireless_adapter notes:
// - Command/param words from the GBA are answered with 0x80000000;
//   librfu treats anything else as clock drift and retries.
// - The ack header 0x9966_RR_(cmd|0x80) is preloaded so it lands in the
//   GBA's response-request transfer; response words follow one per idle
//   word. Errors ack as 0x996601EE plus one code word.
// - 0x25/0x27/0x35/0x37 ack through the normal master-side flow; only
//   the later event frame (0x9966_LL_[27|28|29 variants]) is
//   adapter-clocked, ending with an ack transfer in which the GBA sends
//   0x9966_00_(ev|0x80) and the adapter must answer 0x80000000.
// - Data moves only on host RF frames: a client's send is a scheduled
//   upload the next host transmission collects.

#define DRIVER_ID 0x61554652 // "RFUa"
#define DRIVER_STATE_VERSION 1
#define WIRELESS_INTERVAL 4096
#define UNLOCKED_INTERVAL 4096
// One RF exchange every quarter frame: fine enough that a game polling
// after sendDataAndWait sees sub-frame latency, coarse enough that the
// barrier is a small fraction of the cable lockstep's sync traffic. The
// wait timeout is specified in 16.7ms frames, so it converts at 4 ticks
// per frame.
#define RF_TICK_INTERVAL 70224
#define RF_TICKS_PER_FRAME 4
#define TARGET(P) (1 << (P))
#define TARGET_ALL 0xF

// Adapter-clocked transfers run at the post-login 2MHz rate.
#define WL_TRANSFER_CYCLES 256

#define WL_MAGIC 0x9966
#define WL_IDLE_WORD 0x80000000
// 0x12 VersionStatus payload observed on real units.
#define WL_HW_VERSION 0x00830117

// Adapter command bytes.
enum {
	WL_CMD_RESET = 0x10,
	WL_CMD_LINK_STATUS = 0x11,
	WL_CMD_VERSION_STATUS = 0x12,
	WL_CMD_SYSTEM_STATUS = 0x13,
	WL_CMD_SLOT_STATUS = 0x14,
	WL_CMD_CONFIG_STATUS = 0x15,
	WL_CMD_GAME_CONFIG = 0x16,
	WL_CMD_SYSTEM_CONFIG = 0x17,
	WL_CMD_SC_START = 0x19,
	WL_CMD_SC_POLL = 0x1A,
	WL_CMD_SC_END = 0x1B,
	WL_CMD_SP_START = 0x1C,
	WL_CMD_SP_POLL = 0x1D,
	WL_CMD_SP_END = 0x1E,
	WL_CMD_CP_START = 0x1F,
	WL_CMD_CP_POLL = 0x20,
	WL_CMD_CP_END = 0x21,
	WL_CMD_DATA_TX = 0x24,
	WL_CMD_DATA_TX_AND_CHANGE = 0x25,
	WL_CMD_DATA_RX = 0x26,
	WL_CMD_MS_CHANGE = 0x27,
	WL_CMD_DISCONNECT = 0x30,
	WL_CMD_STOP_MODE = 0x3D,
};

// Adapter-initiated event frame command bytes.
enum {
	WL_EVENT_TIMEOUT = 0x27,
	WL_EVENT_DATA = 0x28,
	WL_EVENT_DISCONNECT = 0x29,
};

// The login sequence both sides walk: NI, NT, EN, DO, then the id word.
static const uint16_t _loginSeq[5] = { 0x494E, 0x544E, 0x4E45, 0x4F44, 0x8001 };

DECL_BITFIELD(GBASIOWirelessSerializedFlags, uint32_t);
DECL_BITS(GBASIOWirelessSerializedFlags, NumEvents, 0, 4);
DECL_BIT(GBASIOWirelessSerializedFlags, Asleep, 4);
DECL_BIT(GBASIOWirelessSerializedFlags, EventScheduled, 5);

DECL_BITFIELD(GBASIOWirelessSerializedEventFlags, uint32_t);
DECL_BITS(GBASIOWirelessSerializedEventFlags, Type, 0, 3);

DECL_BITFIELD(GBASIOWirelessSerializedAdapterFlags, uint32_t);
DECL_BITS(GBASIOWirelessSerializedAdapterFlags, Serial, 0, 3);
DECL_BITS(GBASIOWirelessSerializedAdapterFlags, Role, 3, 2);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, HostOpen, 5);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, Broadcasting, 6);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, Scanning, 7);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, ConnectPending, 8);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, ConnectFailed, 9);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, TxDataPending, 10);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, RxFresh, 11);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, WaitPending, 12);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, EventArmed, 13);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, PeerDropped, 14);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, SlaveStartArmed, 15);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, JustTransmitted, 16);
DECL_BIT(GBASIOWirelessSerializedAdapterFlags, SlaveHandshake, 17);

struct GBASIOWirelessSerializedEvent {
	int32_t timestamp;
	int32_t playerId;
	GBASIOWirelessSerializedEventFlags flags;
	int32_t reserved;
};
static_assert(sizeof(struct GBASIOWirelessSerializedEvent) == 0x10, "GBA wireless event savestate struct sized wrong");

struct GBASIOWirelessSerializedMailbox {
	uint8_t length;
	uint8_t data[WL_HOST_DATA_BYTES + 1];
	uint8_t reserved[1];
};
static_assert(sizeof(struct GBASIOWirelessSerializedMailbox) == 0x5A, "GBA wireless mailbox savestate struct sized wrong");

struct GBASIOWirelessSerializedState {
	uint32_t version;
	GBASIOWirelessSerializedFlags flags;
	uint32_t reserved[2];

	struct {
		int32_t nextEvent;
		uint32_t reservedDriver[7];
	} driver;

	struct {
		int32_t playerId;
		int32_t cycleOffset;
		uint32_t reservedPlayer[2];
		struct GBASIOWirelessSerializedEvent events[MAX_WIRELESS_EVENTS];
	} player;

	struct GBASIOWirelessSerializedAdapter {
		GBASIOWirelessSerializedAdapterFlags flags;
		uint32_t loginIndex;
		uint32_t idleTicks;

		uint32_t rxCommand;
		uint32_t rxRemaining;
		uint32_t rxCount;
		uint32_t rxParams[WL_MAX_COMMAND_WORDS];

		uint32_t txCount;
		uint32_t txIndex;
		uint32_t txWords[WL_MAX_REPLY_WORDS + 1];
		uint32_t out;

		uint32_t broadcast[WL_BROADCAST_WORDS];
		uint32_t setup;

		uint32_t clientMask;
		int8_t clientPlayers[WL_MAX_CLIENTS];
		int32_t hostPlayer;
		int32_t clientNumber;
		uint32_t connectTarget;

		uint32_t waitTicks;
		uint32_t evCount;
		uint32_t evIndex;
		uint32_t evWords[2];

		uint32_t scanCount;
		uint32_t scanResults[(MAX_GBAS - 1) * (WL_BROADCAST_WORDS + 1)];

		struct GBASIOWirelessSerializedMailbox txData;
		struct GBASIOWirelessSerializedMailbox rxData[WL_MAX_CLIENTS + 1];

		uint32_t reservedAdapter[4];
	} adapter;

	// playerId 0 only
	struct {
		int32_t cycle;
		uint32_t waiting;
		int32_t nextRfTick;
		uint32_t reservedCoordinator[5];
	} coordinator;
};
// The game-visible adapter state alone — what a boot capture carries so
// a link rebuilt mid-session resumes every adapter where it was (the
// players walk into RF range with their sessions intact). Sync
// bookkeeping (player ids, clock offsets, event queues) deliberately
// does not travel: the rebuild's attach path recreates it.
struct GBASIOWirelessSerializedAdapterState {
	uint32_t version;
	uint32_t reserved;
	struct GBASIOWirelessSerializedAdapter adapter;
};

static_assert(offsetof(struct GBASIOWirelessSerializedState, driver) == 0x10, "GBA wireless savestate driver offset wrong");
static_assert(offsetof(struct GBASIOWirelessSerializedState, player) == 0x30, "GBA wireless savestate player offset wrong");
static_assert(offsetof(struct GBASIOWirelessSerializedState, adapter) == 0xC0, "GBA wireless savestate adapter offset wrong");
static_assert(offsetof(struct GBASIOWirelessSerializedState, coordinator) == 0x484, "GBA wireless savestate coordinator offset wrong");
static_assert(sizeof(struct GBASIOWirelessSerializedState) == 0x4A4, "GBA wireless savestate struct sized wrong");

static bool GBASIOWirelessDriverInit(struct GBASIODriver* driver);
static void GBASIOWirelessDriverDeinit(struct GBASIODriver* driver);
static void GBASIOWirelessDriverReset(struct GBASIODriver* driver);
static uint32_t GBASIOWirelessDriverId(const struct GBASIODriver* driver);
static bool GBASIOWirelessDriverLoadState(struct GBASIODriver* driver, const void* state, size_t size);
static void GBASIOWirelessDriverSaveState(struct GBASIODriver* driver, void** state, size_t* size);
static void GBASIOWirelessDriverSetMode(struct GBASIODriver* driver, enum GBASIOMode mode);
static bool GBASIOWirelessDriverHandlesMode(struct GBASIODriver* driver, enum GBASIOMode mode);
static int GBASIOWirelessDriverConnectedDevices(struct GBASIODriver* driver);
static uint16_t GBASIOWirelessDriverWriteSIOCNT(struct GBASIODriver* driver, uint16_t value);
static uint16_t GBASIOWirelessDriverWriteRCNT(struct GBASIODriver* driver, uint16_t value);
static bool GBASIOWirelessDriverStart(struct GBASIODriver* driver);
static uint8_t GBASIOWirelessDriverFinishNormal8(struct GBASIODriver* driver);
static uint32_t GBASIOWirelessDriverFinishNormal32(struct GBASIODriver* driver);

static void GBASIOWirelessCoordinatorWaitOnPlayers(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessPlayer*);
static void GBASIOWirelessCoordinatorAckPlayer(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessPlayer*);
static void GBASIOWirelessCoordinatorWakePlayers(struct GBASIOWirelessCoordinator*);

static int32_t GBASIOWirelessTime(struct GBASIOWirelessPlayer*);
static void GBASIOWirelessPlayerWake(struct GBASIOWirelessPlayer*);
static void GBASIOWirelessPlayerSleep(struct GBASIOWirelessPlayer*);

static void _advanceCycle(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessPlayer*);
static void _removePlayer(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessPlayer*);
static void _reconfigPlayers(struct GBASIOWirelessCoordinator*);
static int32_t _untilNextSync(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessPlayer*);
static void _enqueueEvent(struct GBASIOWirelessCoordinator*, const struct GBASIOWirelessEvent*, uint32_t target);
static void _rfCommit(struct GBASIOWirelessCoordinator*);
static void _severAll(struct GBASIOWirelessCoordinator*, bool notify);
static void _adapterPowerOn(struct GBASIOWirelessAdapter*);
static uint32_t _adapterExchange(struct GBASIOWirelessPlayer*, uint32_t gbaWord);
static void _executeCommand(struct GBASIOWirelessPlayer*);
static void _deliverPending(struct GBASIOWirelessPlayer*);

static void _wirelessEvent(struct mTiming*, void* context, uint32_t cyclesLate);

// The adapter's 16-bit device id. Real hardware rerolls a random id at
// each 0x19/0x1F; games treat it as opaque, so a deterministic id keyed
// by player slot keeps every peer's simulation identical.
static uint16_t _deviceId(int playerId) {
	return 0x61F0 + playerId;
}

static void _verifyAwake(struct GBASIOWirelessCoordinator* coordinator) {
#ifdef NDEBUG
	UNUSED(coordinator);
#else
	int i;
	int asleep = 0;
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!coordinator->attachedPlayers[i]) {
			continue;
		}
		struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, coordinator->attachedPlayers[i]);
		asleep += player->asleep;
	}
	mASSERT_DEBUG(!asleep || asleep < coordinator->nAttached);
#endif
}

void GBASIOWirelessDriverCreate(struct GBASIOWirelessDriver* driver, struct mLockstepUser* user) {
	memset(driver, 0, sizeof(*driver));
	driver->d.init = GBASIOWirelessDriverInit;
	driver->d.deinit = GBASIOWirelessDriverDeinit;
	driver->d.reset = GBASIOWirelessDriverReset;
	driver->d.driverId = GBASIOWirelessDriverId;
	driver->d.loadState = GBASIOWirelessDriverLoadState;
	driver->d.saveState = GBASIOWirelessDriverSaveState;
	driver->d.setMode = GBASIOWirelessDriverSetMode;
	driver->d.handlesMode = GBASIOWirelessDriverHandlesMode;
	driver->d.connectedDevices = GBASIOWirelessDriverConnectedDevices;
	driver->d.writeSIOCNT = GBASIOWirelessDriverWriteSIOCNT;
	driver->d.writeRCNT = GBASIOWirelessDriverWriteRCNT;
	driver->d.start = GBASIOWirelessDriverStart;
	driver->d.finishNormal8 = GBASIOWirelessDriverFinishNormal8;
	driver->d.finishNormal32 = GBASIOWirelessDriverFinishNormal32;
	driver->event.context = driver;
	driver->event.callback = _wirelessEvent;
	driver->event.name = "GBA SIO Wireless";
	driver->event.priority = 0x80;
	driver->user = user;
}

static bool GBASIOWirelessDriverInit(struct GBASIODriver* driver) {
	GBASIOWirelessDriverReset(driver);
	return true;
}

static void GBASIOWirelessDriverDeinit(struct GBASIODriver* driver) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, wireless->wirelessId);
	if (player) {
		_removePlayer(coordinator, player);
	}
	MutexUnlock(&coordinator->mutex);
	mTimingDeschedule(&wireless->d.p->p->timing, &wireless->event);
	wireless->wirelessId = 0;
}

static void _adapterPowerOn(struct GBASIOWirelessAdapter* adapter) {
	memset(adapter, 0, sizeof(*adapter));
	adapter->serial = WL_SERIAL_LOGIN;
	adapter->hostPlayer = -1;
	adapter->clientNumber = -1;
	int i;
	for (i = 0; i < WL_MAX_CLIENTS; ++i) {
		adapter->clientPlayers[i] = -1;
	}
}

// 0x10: reset to the idle state with all session config cleared, but
// stay authenticated (unlike a power-on).
static void _adapterIdle(struct GBASIOWirelessAdapter* adapter) {
	enum GBASIOWirelessSerialState serial = adapter->serial;
	uint8_t rxCommand = adapter->rxCommand;
	uint32_t out = adapter->out;
	_adapterPowerOn(adapter);
	adapter->serial = serial;
	adapter->rxCommand = rxCommand;
	adapter->out = out;
}

static void GBASIOWirelessDriverReset(struct GBASIODriver* driver) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	struct GBASIOWirelessPlayer* player;
	if (!wireless->wirelessId) {
		unsigned id;
		player = calloc(1, sizeof(*player));
		player->driver = wireless;
		player->playerId = -1;
		_adapterPowerOn(&player->adapter);

		int i;
		for (i = 0; i < MAX_WIRELESS_EVENTS - 1; ++i) {
			player->buffer[i].next = &player->buffer[i + 1];
		}
		player->freeList = &player->buffer[0];

		MutexLock(&coordinator->mutex);
		while (true) {
			if (coordinator->nextId == UINT_MAX) {
				coordinator->nextId = 0;
			}
			++coordinator->nextId;
			id = coordinator->nextId;
			if (!TableLookup(&coordinator->players, id)) {
				TableInsert(&coordinator->players, id, player);
				wireless->wirelessId = id;
				break;
			}
		}
		bool hadOthers = TableSize(&coordinator->players) > 1;
		_reconfigPlayers(coordinator);
		player->cycleOffset = mTimingCurrentTime(&driver->p->p->timing) - coordinator->cycle;
		if (player->playerId != 0) {
			struct GBASIOWirelessEvent event = {
				.type = WL_EV_ATTACH,
				.playerId = player->playerId,
				.timestamp = GBASIOWirelessTime(player),
			};
			_enqueueEvent(coordinator, &event, TARGET_ALL & ~TARGET(player->playerId));
		}
		if (hadOthers) {
			// Joining may renumber the existing players, which would
			// dangle every playerId-based connection reference; severing
			// everything (an adapter-level disconnect games handle) is
			// the simple correct answer, and a mid-session join has no
			// live connection worth keeping anyway.
			_severAll(coordinator, true);
		}
	} else {
		MutexLock(&coordinator->mutex);
		player = TableLookup(&coordinator->players, wireless->wirelessId);
		player->cycleOffset = mTimingCurrentTime(&driver->p->p->timing) - coordinator->cycle;
		// A core reset power-cycles the adapter.
		_adapterPowerOn(&player->adapter);
	}

	if (player->playerId == 0 && coordinator->nAttached > 1) {
		coordinator->waiting = 0;
		player->asleep = false;
		GBASIOWirelessCoordinatorWakePlayers(coordinator);
	}

	if (mTimingIsScheduled(&wireless->d.p->p->timing, &wireless->event)) {
		MutexUnlock(&coordinator->mutex);
		return;
	}

	int32_t nextEvent;
	if (TableSize(&coordinator->players) == 1) {
		coordinator->cycle = mTimingCurrentTime(&wireless->d.p->p->timing);
		nextEvent = WIRELESS_INTERVAL;
	} else {
		nextEvent = _untilNextSync(wireless->coordinator, player);
	}
	MutexUnlock(&coordinator->mutex);
	mTimingSchedule(&wireless->d.p->p->timing, &wireless->event, nextEvent);
}

static uint32_t GBASIOWirelessDriverId(const struct GBASIODriver* driver) {
	UNUSED(driver);
	return DRIVER_ID;
}

static void _serializeMailbox(struct GBASIOWirelessSerializedMailbox* out, const struct GBASIOWirelessMailbox* in) {
	out->length = in->length;
	memcpy(out->data, in->data, sizeof(out->data));
}

static void _deserializeMailbox(struct GBASIOWirelessMailbox* out, const struct GBASIOWirelessSerializedMailbox* in) {
	out->length = in->length;
	if (out->length > sizeof(out->data)) {
		out->length = sizeof(out->data);
	}
	memcpy(out->data, in->data, sizeof(out->data));
}

// Restore the game-visible adapter state from its serialized form,
// clamping every index against corruption. Shared by the full driver
// blob and the standalone boot-capture blob.
static void _loadAdapter(struct GBASIOWirelessAdapter* adapter, const struct GBASIOWirelessSerializedAdapter* src) {
	unsigned i;
	int32_t check;
	
	GBASIOWirelessSerializedAdapterFlags aflags;
	LOAD_32LE(aflags, 0, &src->flags);
	adapter->serial = GBASIOWirelessSerializedAdapterFlagsGetSerial(aflags);
	adapter->role = GBASIOWirelessSerializedAdapterFlagsGetRole(aflags);
	adapter->hostOpen = GBASIOWirelessSerializedAdapterFlagsGetHostOpen(aflags);
	adapter->broadcasting = GBASIOWirelessSerializedAdapterFlagsGetBroadcasting(aflags);
	adapter->scanning = GBASIOWirelessSerializedAdapterFlagsGetScanning(aflags);
	adapter->connectPending = GBASIOWirelessSerializedAdapterFlagsGetConnectPending(aflags);
	adapter->connectFailed = GBASIOWirelessSerializedAdapterFlagsGetConnectFailed(aflags);
	adapter->txDataPending = GBASIOWirelessSerializedAdapterFlagsGetTxDataPending(aflags);
	adapter->rxFresh = GBASIOWirelessSerializedAdapterFlagsGetRxFresh(aflags);
	adapter->waitPending = GBASIOWirelessSerializedAdapterFlagsGetWaitPending(aflags);
	adapter->eventArmed = GBASIOWirelessSerializedAdapterFlagsGetEventArmed(aflags);
	adapter->peerDropped = GBASIOWirelessSerializedAdapterFlagsGetPeerDropped(aflags);
	adapter->slaveStartArmed = GBASIOWirelessSerializedAdapterFlagsGetSlaveStartArmed(aflags);
	adapter->justTransmitted = GBASIOWirelessSerializedAdapterFlagsGetJustTransmitted(aflags);
	adapter->slaveHandshake = GBASIOWirelessSerializedAdapterFlagsGetSlaveHandshake(aflags);

	uint32_t scratch;
	LOAD_32LE(scratch, 0, &src->loginIndex);
	adapter->loginIndex = scratch < 5 ? scratch : 0;
	LOAD_32LE(scratch, 0, &src->idleTicks);
	adapter->idleTicks = scratch;
	LOAD_32LE(scratch, 0, &src->rxCommand);
	adapter->rxCommand = scratch;
	LOAD_32LE(scratch, 0, &src->rxRemaining);
	adapter->rxRemaining = scratch;
	LOAD_32LE(scratch, 0, &src->rxCount);
	adapter->rxCount = scratch;
	if (adapter->rxRemaining > WL_MAX_COMMAND_WORDS) {
		adapter->rxRemaining = WL_MAX_COMMAND_WORDS;
	}
	if (adapter->rxCount > WL_MAX_COMMAND_WORDS) {
		adapter->rxCount = WL_MAX_COMMAND_WORDS;
	}
	for (i = 0; i < WL_MAX_COMMAND_WORDS; ++i) {
		LOAD_32LE(adapter->rxParams[i], 0, &src->rxParams[i]);
	}
	LOAD_32LE(scratch, 0, &src->txCount);
	adapter->txCount = scratch;
	LOAD_32LE(scratch, 0, &src->txIndex);
	adapter->txIndex = scratch;
	if (adapter->txCount > WL_MAX_REPLY_WORDS + 1) {
		adapter->txCount = WL_MAX_REPLY_WORDS + 1;
	}
	if (adapter->txIndex > adapter->txCount) {
		adapter->txIndex = adapter->txCount;
	}
	for (i = 0; i < WL_MAX_REPLY_WORDS + 1; ++i) {
		LOAD_32LE(adapter->txWords[i], 0, &src->txWords[i]);
	}
	LOAD_32LE(adapter->out, 0, &src->out);
	for (i = 0; i < WL_BROADCAST_WORDS; ++i) {
		LOAD_32LE(adapter->broadcast[i], 0, &src->broadcast[i]);
	}
	LOAD_32LE(adapter->setup, 0, &src->setup);
	LOAD_32LE(scratch, 0, &src->clientMask);
	adapter->clientMask = scratch;
	for (i = 0; i < WL_MAX_CLIENTS; ++i) {
		adapter->clientPlayers[i] = src->clientPlayers[i];
	}
	LOAD_32LE(check, 0, &src->hostPlayer);
	adapter->hostPlayer = check;
	LOAD_32LE(check, 0, &src->clientNumber);
	adapter->clientNumber = check;
	LOAD_32LE(scratch, 0, &src->connectTarget);
	adapter->connectTarget = scratch;
	LOAD_32LE(scratch, 0, &src->waitTicks);
	adapter->waitTicks = scratch;
	LOAD_32LE(scratch, 0, &src->evCount);
	adapter->evCount = scratch <= 2 ? scratch : 2;
	LOAD_32LE(scratch, 0, &src->evIndex);
	adapter->evIndex = scratch <= adapter->evCount ? scratch : adapter->evCount;
	for (i = 0; i < 2; ++i) {
		LOAD_32LE(adapter->evWords[i], 0, &src->evWords[i]);
	}
	LOAD_32LE(scratch, 0, &src->scanCount);
	adapter->scanCount = scratch;
	if (adapter->scanCount > MAX_GBAS - 1) {
		adapter->scanCount = MAX_GBAS - 1;
	}
	for (i = 0; i < (MAX_GBAS - 1) * (WL_BROADCAST_WORDS + 1); ++i) {
		LOAD_32LE(adapter->scanResults[i], 0, &src->scanResults[i]);
	}
	_deserializeMailbox(&adapter->txData, &src->txData);
	for (i = 0; i < WL_MAX_CLIENTS + 1; ++i) {
		_deserializeMailbox(&adapter->rxData[i], &src->rxData[i]);
	}
}

// Serialize the game-visible adapter state. Shared by the full driver
// blob and the standalone boot-capture blob.
static void _saveAdapter(struct GBASIOWirelessSerializedAdapter* dst, const struct GBASIOWirelessAdapter* adapter) {
	
	GBASIOWirelessSerializedAdapterFlags aflags = 0;
	aflags = GBASIOWirelessSerializedAdapterFlagsSetSerial(aflags, adapter->serial);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetRole(aflags, adapter->role);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetHostOpen(aflags, adapter->hostOpen);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetBroadcasting(aflags, adapter->broadcasting);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetScanning(aflags, adapter->scanning);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetConnectPending(aflags, adapter->connectPending);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetConnectFailed(aflags, adapter->connectFailed);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetTxDataPending(aflags, adapter->txDataPending);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetRxFresh(aflags, adapter->rxFresh);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetWaitPending(aflags, adapter->waitPending);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetEventArmed(aflags, adapter->eventArmed);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetPeerDropped(aflags, adapter->peerDropped);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetSlaveStartArmed(aflags, adapter->slaveStartArmed);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetJustTransmitted(aflags, adapter->justTransmitted);
	aflags = GBASIOWirelessSerializedAdapterFlagsSetSlaveHandshake(aflags, adapter->slaveHandshake);
	STORE_32LE(aflags, 0, &dst->flags);

	STORE_32LE(adapter->loginIndex, 0, &dst->loginIndex);
	STORE_32LE(adapter->idleTicks, 0, &dst->idleTicks);
	STORE_32LE(adapter->rxCommand, 0, &dst->rxCommand);
	STORE_32LE(adapter->rxRemaining, 0, &dst->rxRemaining);
	STORE_32LE(adapter->rxCount, 0, &dst->rxCount);
	size_t i;
	for (i = 0; i < WL_MAX_COMMAND_WORDS; ++i) {
		STORE_32LE(adapter->rxParams[i], 0, &dst->rxParams[i]);
	}
	STORE_32LE(adapter->txCount, 0, &dst->txCount);
	STORE_32LE(adapter->txIndex, 0, &dst->txIndex);
	for (i = 0; i < WL_MAX_REPLY_WORDS + 1; ++i) {
		STORE_32LE(adapter->txWords[i], 0, &dst->txWords[i]);
	}
	STORE_32LE(adapter->out, 0, &dst->out);
	for (i = 0; i < WL_BROADCAST_WORDS; ++i) {
		STORE_32LE(adapter->broadcast[i], 0, &dst->broadcast[i]);
	}
	STORE_32LE(adapter->setup, 0, &dst->setup);
	STORE_32LE(adapter->clientMask, 0, &dst->clientMask);
	for (i = 0; i < WL_MAX_CLIENTS; ++i) {
		dst->clientPlayers[i] = adapter->clientPlayers[i];
	}
	STORE_32LE(adapter->hostPlayer, 0, &dst->hostPlayer);
	STORE_32LE(adapter->clientNumber, 0, &dst->clientNumber);
	STORE_32LE(adapter->connectTarget, 0, &dst->connectTarget);
	STORE_32LE(adapter->waitTicks, 0, &dst->waitTicks);
	STORE_32LE(adapter->evCount, 0, &dst->evCount);
	STORE_32LE(adapter->evIndex, 0, &dst->evIndex);
	for (i = 0; i < 2; ++i) {
		STORE_32LE(adapter->evWords[i], 0, &dst->evWords[i]);
	}
	STORE_32LE(adapter->scanCount, 0, &dst->scanCount);
	for (i = 0; i < (size_t) (MAX_GBAS - 1) * (WL_BROADCAST_WORDS + 1); ++i) {
		STORE_32LE(adapter->scanResults[i], 0, &dst->scanResults[i]);
	}
	_serializeMailbox(&dst->txData, &adapter->txData);
	for (i = 0; i < WL_MAX_CLIENTS + 1; ++i) {
		_serializeMailbox(&dst->rxData[i], &adapter->rxData[i]);
	}
}

static bool GBASIOWirelessDriverLoadState(struct GBASIODriver* driver, const void* data, size_t size) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	if (size != sizeof(struct GBASIOWirelessSerializedState)) {
		mLOG(GBA_SIO, WARN, "Incorrect state size: expected %" PRIz "X, got %" PRIz "X", sizeof(struct GBASIOWirelessSerializedState), size);
		return false;
	}
	const struct GBASIOWirelessSerializedState* state = data;
	bool error = false;
	uint32_t ucheck;
	int32_t check;
	LOAD_32LE(ucheck, 0, &state->version);
	if (ucheck > DRIVER_STATE_VERSION) {
		mLOG(GBA_SIO, WARN, "Invalid or too new save state: expected %u, got %u", DRIVER_STATE_VERSION, ucheck);
		return false;
	}

	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, wireless->wirelessId);
	LOAD_32LE(check, 0, &state->player.playerId);
	if (check != player->playerId) {
		mLOG(GBA_SIO, WARN, "State is for different player: expected %d, got %d", player->playerId, check);
		error = true;
		goto out;
	}

	GBASIOWirelessSerializedFlags flags = 0;
	LOAD_32LE(flags, 0, &state->flags);
	LOAD_32LE(player->cycleOffset, 0, &state->player.cycleOffset);

	// The event may still be scheduled, from the live pre-restore state
	// or from a caller restoring a driver blob without a core load in
	// between; scheduling an already-scheduled event corrupts the timing
	// list, so always deschedule first.
	mTimingDeschedule(&driver->p->p->timing, &wireless->event);
	if (GBASIOWirelessSerializedFlagsGetEventScheduled(flags)) {
		int32_t when;
		LOAD_32LE(when, 0, &state->driver.nextEvent);
		mTimingSchedule(&driver->p->p->timing, &wireless->event, when);
	}

	if (GBASIOWirelessSerializedFlagsGetAsleep(flags)) {
		if (!player->asleep && player->driver->user->sleep) {
			player->driver->user->sleep(player->driver->user);
		}
		player->asleep = true;
	} else {
		if (player->asleep && player->driver->user->wake) {
			player->driver->user->wake(player->driver->user);
		}
		player->asleep = false;
	}

	unsigned i;
	for (i = 0; i < MAX_WIRELESS_EVENTS - 1; ++i) {
		player->buffer[i].next = &player->buffer[i + 1];
	}
	player->freeList = &player->buffer[0];
	player->queue = NULL;

	struct GBASIOWirelessEvent** lastEvent = &player->queue;
	for (i = 0; i < GBASIOWirelessSerializedFlagsGetNumEvents(flags) && i < MAX_WIRELESS_EVENTS; ++i) {
		struct GBASIOWirelessEvent* event = player->freeList;
		const struct GBASIOWirelessSerializedEvent* stateEvent = &state->player.events[i];
		player->freeList = player->freeList->next;
		*lastEvent = event;
		lastEvent = &event->next;

		GBASIOWirelessSerializedEventFlags eventFlags;
		LOAD_32LE(eventFlags, 0, &stateEvent->flags);
		LOAD_32LE(event->timestamp, 0, &stateEvent->timestamp);
		LOAD_32LE(event->playerId, 0, &stateEvent->playerId);
		event->type = GBASIOWirelessSerializedEventFlagsGetType(eventFlags);
		event->next = NULL;
	}

	_loadAdapter(&player->adapter, &state->adapter);
	if (player->playerId == 0) {
		LOAD_32LE(coordinator->cycle, 0, &state->coordinator.cycle);
		LOAD_32LE(coordinator->waiting, 0, &state->coordinator.waiting);
		LOAD_32LE(coordinator->nextRfTick, 0, &state->coordinator.nextRfTick);
	}
out:
	MutexUnlock(&coordinator->mutex);
	if (!error) {
		mTimingInterrupt(&driver->p->p->timing);
	}
	return !error;
}

static void GBASIOWirelessDriverSaveState(struct GBASIODriver* driver, void** stateOut, size_t* size) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	struct GBASIOWirelessSerializedState* state = calloc(1, sizeof(*state));

	STORE_32LE(DRIVER_STATE_VERSION, 0, &state->version);

	STORE_32LE(wireless->event.when - mTimingCurrentTime(&driver->p->p->timing), 0, &state->driver.nextEvent);

	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, wireless->wirelessId);
	GBASIOWirelessSerializedFlags flags = 0;
	STORE_32LE(player->playerId, 0, &state->player.playerId);
	STORE_32LE(player->cycleOffset, 0, &state->player.cycleOffset);
	flags = GBASIOWirelessSerializedFlagsSetAsleep(flags, player->asleep);
	flags = GBASIOWirelessSerializedFlagsSetEventScheduled(flags, mTimingIsScheduled(&driver->p->p->timing, &wireless->event));

	struct GBASIOWirelessEvent* event = player->queue;
	size_t nEvents;
	for (nEvents = 0; nEvents < MAX_WIRELESS_EVENTS && event; ++nEvents, event = event->next) {
		struct GBASIOWirelessSerializedEvent* stateEvent = &state->player.events[nEvents];
		GBASIOWirelessSerializedEventFlags eventFlags = GBASIOWirelessSerializedEventFlagsSetType(0, event->type);
		STORE_32LE(event->timestamp, 0, &stateEvent->timestamp);
		STORE_32LE(event->playerId, 0, &stateEvent->playerId);
		STORE_32LE(eventFlags, 0, &stateEvent->flags);
	}
	flags = GBASIOWirelessSerializedFlagsSetNumEvents(flags, nEvents);

	_saveAdapter(&state->adapter, &player->adapter);
	if (player->playerId == 0) {
		STORE_32LE(coordinator->cycle, 0, &state->coordinator.cycle);
		STORE_32LE(coordinator->waiting, 0, &state->coordinator.waiting);
		STORE_32LE(coordinator->nextRfTick, 0, &state->coordinator.nextRfTick);
	}
	MutexUnlock(&coordinator->mutex);

	STORE_32LE(flags, 0, &state->flags);
	*stateOut = state;
	*size = sizeof(*state);
}

void GBASIOWirelessDriverSaveAdapterState(struct GBASIOWirelessDriver* wireless, void** stateOut, size_t* size) {
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	struct GBASIOWirelessSerializedAdapterState* state = calloc(1, sizeof(*state));
	STORE_32LE(DRIVER_STATE_VERSION, 0, &state->version);
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, wireless->wirelessId);
	if (player) {
		_saveAdapter(&state->adapter, &player->adapter);
	}
	MutexUnlock(&coordinator->mutex);
	*stateOut = state;
	*size = sizeof(*state);
}

bool GBASIOWirelessDriverLoadAdapterState(struct GBASIOWirelessDriver* wireless, const void* data, size_t size) {
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	if (size != sizeof(struct GBASIOWirelessSerializedAdapterState)) {
		mLOG(GBA_SIO, WARN, "Incorrect adapter state size: expected %" PRIz "X, got %" PRIz "X", sizeof(struct GBASIOWirelessSerializedAdapterState), size);
		return false;
	}
	const struct GBASIOWirelessSerializedAdapterState* state = data;
	uint32_t version;
	LOAD_32LE(version, 0, &state->version);
	if (version > DRIVER_STATE_VERSION) {
		mLOG(GBA_SIO, WARN, "Invalid or too new adapter state: expected %u, got %u", DRIVER_STATE_VERSION, version);
		return false;
	}
	bool ok = false;
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, wireless->wirelessId);
	if (player) {
		_loadAdapter(&player->adapter, &state->adapter);
		ok = true;
	}
	MutexUnlock(&coordinator->mutex);
	return ok;
}

static void GBASIOWirelessDriverSetMode(struct GBASIODriver* driver, enum GBASIOMode mode) {
	UNUSED(driver);
	UNUSED(mode);
	// The adapter does not care what the GBA's SIO mode is; it only ever
	// answers NORMAL transfers, and the RCNT reset dance is watched from
	// writeRCNT directly.
}

static bool GBASIOWirelessDriverHandlesMode(struct GBASIODriver* driver, enum GBASIOMode mode) {
	UNUSED(driver);
	switch (mode) {
	case GBA_SIO_NORMAL_8:
	case GBA_SIO_NORMAL_32:
		return true;
	default:
		return false;
	}
}

static int GBASIOWirelessDriverConnectedDevices(struct GBASIODriver* driver) {
	UNUSED(driver);
	// The adapter itself is the connected device.
	return 1;
}

static struct GBASIOWirelessPlayer* _lookupPlayer(struct GBASIOWirelessDriver* wireless) {
	return TableLookup(&wireless->coordinator->players, wireless->wirelessId);
}

static uint16_t GBASIOWirelessDriverWriteSIOCNT(struct GBASIODriver* driver, uint16_t value) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	struct GBASIO* sio = driver->p;
	uint16_t old = sio->siocnt;
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = _lookupPlayer(wireless);
	if (player) {
		struct GBASIOWirelessAdapter* adapter = &player->adapter;
		// The inter-word SO/SI handshake, per librfu's ISR cadence
		// (librfu_intr.c): after a GBA-clocked word the adapter raises
		// SI ("consumed", done in finishNormal32) and the GBA answers
		// with SO high, on which we drop SI; after an adapter-clocked
		// word the adapter drops SI and the GBA's SO high asks for the
		// next word, on which we raise SI — including one final pass
		// after the event ack transfer (the GBA's state-8 handshake),
		// which is why the answer is armed per completed transfer
		// (slaveHandshake) rather than keyed off a pending event.
		bool si = GBASIONormalGetSi(old);
		if (GBASIONormalGetIdleSo(value) && !GBASIONormalGetIdleSo(old)) {
			si = adapter->slaveHandshake;
			adapter->slaveHandshake = false;
		}
		value = GBASIONormalSetSi(value, si);
		// librfu's sio32id arms the start bit as an external-clock
		// slave, then flips the clock source to internal and expects
		// the armed transfer to begin (AgbRFU_checkID). mgba only
		// starts transfers on a start-bit edge, so kick the completion
		// ourselves on the clock-source edge.
		if (sio->mode == GBA_SIO_NORMAL_32 && GBASIONormalIsStart(value) && GBASIONormalIsStart(old)
		    && GBASIONormalGetSc(value) && !GBASIONormalGetSc(old)
		    && !mTimingIsScheduled(&sio->p->timing, &sio->completeEvent)) {
			mTimingSchedule(&sio->p->timing, &sio->completeEvent,
			                GBASIOTransferCycles(GBA_SIO_NORMAL_32, value, 1));
		}
	}
	MutexUnlock(&coordinator->mutex);
	return value;
}

static uint16_t GBASIOWirelessDriverWriteRCNT(struct GBASIODriver* driver, uint16_t value) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	// The adapter reset dance runs in GPIO mode: SD switched to output
	// and driven high (RCNT = 80A2h in the canonical sequence). Treat
	// that edge as the hardware reset, which re-arms the login handshake.
	if (driver->p->mode == GBA_SIO_GPIO && (value & 0x0022) == 0x0022) {
		MutexLock(&coordinator->mutex);
		struct GBASIOWirelessPlayer* player = _lookupPlayer(wireless);
		if (player && player->adapter.serial != WL_SERIAL_LOGIN) {
			mLOG(GBA_SIO, DEBUG, "Wireless: adapter reset");
			_adapterPowerOn(&player->adapter);
		}
		MutexUnlock(&coordinator->mutex);
	}
	return value;
}

static bool GBASIOWirelessDriverStart(struct GBASIODriver* driver) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	struct GBASIO* sio = driver->p;
	if (sio->mode != GBA_SIO_NORMAL_32) {
		// 8-bit transfers get the standard timing and a dummy reply.
		return true;
	}
	if (GBASIONormalGetSc(sio->siocnt)) {
		// GBA-clocked: the standard completion timing applies and
		// finishNormal32 supplies the adapter's word.
		return true;
	}
	// GBA is waiting as slave: the transfer only happens when the adapter
	// drives the clock, which it does to deliver an event frame. Own the
	// completion.
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = _lookupPlayer(wireless);
	if (player) {
		player->adapter.slaveStartArmed = true;
		_deliverPending(player);
	}
	MutexUnlock(&coordinator->mutex);
	return false;
}

// Schedule the completion of one adapter-clocked transfer if an event
// frame is armed and the GBA is parked as a ready slave. The GBA's slave
// ISR re-arms the start bit after each word, which funnels back through
// the start hook and paces the frame word by word. Called with the
// coordinator lock held, always on the player's own core.
static void _deliverPending(struct GBASIOWirelessPlayer* player) {
	struct GBASIOWirelessAdapter* adapter = &player->adapter;
	struct GBASIO* sio = player->driver->d.p;
	if (!adapter->slaveStartArmed || !adapter->eventArmed) {
		return;
	}
	if (sio->mode != GBA_SIO_NORMAL_32 || GBASIONormalGetSc(sio->siocnt) || !GBASIONormalIsStart(sio->siocnt)) {
		return;
	}
	adapter->slaveStartArmed = false;
	mTimingDeschedule(&sio->p->timing, &sio->completeEvent);
	mTimingSchedule(&sio->p->timing, &sio->completeEvent, WL_TRANSFER_CYCLES);
}

static uint8_t GBASIOWirelessDriverFinishNormal8(struct GBASIODriver* driver) {
	UNUSED(driver);
	// The adapter does not speak the 8-bit protocol; the line floats.
	return 0xFF;
}

static uint32_t GBASIOWirelessDriverFinishNormal32(struct GBASIODriver* driver) {
	struct GBASIOWirelessDriver* wireless = (struct GBASIOWirelessDriver*) driver;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	struct GBASIO* sio = driver->p;
	uint32_t gbaWord = sio->p->memory.io[GBA_REG(SIODATA32_LO)];
	gbaWord |= sio->p->memory.io[GBA_REG(SIODATA32_HI)] << 16;

	uint32_t reply;
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = _lookupPlayer(wireless);
	if (!player) {
		reply = 0xFFFFFFFF;
	} else if (!GBASIONormalGetSc(sio->siocnt)) {
		// Adapter-clocked: shift out the next event frame word, then the
		// ack transfer in which the GBA acknowledges the event and the
		// adapter must answer 0x80000000 (librfu checks it).
		struct GBASIOWirelessAdapter* adapter = &player->adapter;
		if (adapter->eventArmed && adapter->evIndex < adapter->evCount) {
			reply = adapter->evWords[adapter->evIndex];
			++adapter->evIndex;
		} else if (adapter->eventArmed) {
			uint8_t ev = adapter->evWords[0] & 0xFF;
			if ((gbaWord >> 16) != WL_MAGIC || (gbaWord & 0xFF) != (0x80 | ev)) {
				mLOG(GBA_SIO, GAME_ERROR, "Wireless: unexpected event ack %08X for event %02X", gbaWord, ev);
			}
			adapter->eventArmed = false;
			adapter->serial = WL_SERIAL_COMMAND;
			reply = WL_IDLE_WORD;
		} else {
			// A slave transfer completed with nothing armed; should not
			// happen since we own the completion, but be safe.
			reply = WL_IDLE_WORD;
		}
		// The GBA's slave ISR waits for SI low, then answers with SO
		// high, which we answer by raising SI (see writeSIOCNT).
		sio->siocnt = GBASIONormalClearSi(sio->siocnt);
		adapter->slaveHandshake = true;
	} else {
		bool login = player->adapter.serial == WL_SERIAL_LOGIN;
		reply = _adapterExchange(player, gbaWord);
		if (!login) {
			// Word consumed: raise SI for the GBA's post-transfer
			// handshake_wait(1). (The login exchange predates the
			// handshake; leave the line alone there.)
			sio->siocnt = GBASIONormalFillSi(sio->siocnt);
		}
	}
	MutexUnlock(&coordinator->mutex);
	mLOG(GBA_SIO, DEBUG, "Wireless: GBA -> %08X, %08X -> GBA", gbaWord, reply);
	return reply;
}

// The GBA clocked a 32-bit word to the adapter; return the word the
// adapter had preloaded for this transfer and advance the state machine.
// Called with the coordinator lock held.
static uint32_t _adapterExchange(struct GBASIOWirelessPlayer* player, uint32_t gbaWord) {
	struct GBASIOWirelessAdapter* adapter = &player->adapter;
	uint32_t reply = adapter->out;
	uint16_t lo = gbaWord;
	uint16_t hi = gbaWord >> 16;

	adapter->idleTicks = 0;

	// A login word restarts the handshake from any state but dormancy:
	// librfu re-logins without necessarily pulsing the reset line first.
	if (adapter->serial != WL_SERIAL_LOGIN && adapter->serial != WL_SERIAL_DORMANT
	    && hi != WL_MAGIC && lo == _loginSeq[0]) {
		_adapterPowerOn(adapter);
	}

	switch (adapter->serial) {
	case WL_SERIAL_LOGIN: {
		// Both sides walk the NI/NT/EN/DO sequence; the exchange is
		// full duplex, so each side's word answers the previous one. We
		// advance when the GBA's high half acknowledges (inverts) our
		// current value, and we always answer with our current value on
		// top of the inverse of the GBA's low half. The GBA resets on a
		// validation failure and so do we, keeping the machines locked.
		if (lo != _loginSeq[adapter->loginIndex]) {
			adapter->loginIndex = 0;
		}
		bool done = false;
		if (lo == _loginSeq[adapter->loginIndex]) {
			if (hi == (uint16_t) ~_loginSeq[adapter->loginIndex] && adapter->loginIndex < 4) {
				++adapter->loginIndex;
			} else if (adapter->loginIndex == 4) {
				// The GBA reached the id word and validated our echo;
				// the handshake is complete.
				done = true;
			}
		}
		adapter->out = ((uint32_t) _loginSeq[adapter->loginIndex] << 16) | (uint16_t) ~lo;
		if (done) {
			adapter->serial = WL_SERIAL_COMMAND;
			adapter->loginIndex = 0;
			adapter->out = WL_IDLE_WORD;
			mLOG(GBA_SIO, DEBUG, "Wireless: login complete");
		}
		break;
	}
	case WL_SERIAL_DORMANT:
		// 0x3D: dead to the world until the SD reset pulse.
		adapter->out = WL_IDLE_WORD;
		break;
	case WL_SERIAL_COMMAND:
		if (hi == WL_MAGIC && !(lo & 0x80)) {
			adapter->rxCommand = lo & 0xFF;
			adapter->rxRemaining = (lo >> 8) & 0xFF;
			adapter->rxCount = 0;
			if (adapter->rxRemaining > WL_MAX_COMMAND_WORDS) {
				mLOG(GBA_SIO, GAME_ERROR, "Wireless: oversized command %02X length %u", adapter->rxCommand, adapter->rxRemaining);
				adapter->rxRemaining = WL_MAX_COMMAND_WORDS;
			}
			if (adapter->rxRemaining) {
				adapter->serial = WL_SERIAL_PARAMS;
				adapter->out = WL_IDLE_WORD;
			} else {
				_executeCommand(player);
			}
		} else {
			adapter->out = WL_IDLE_WORD;
		}
		break;
	case WL_SERIAL_PARAMS:
		adapter->rxParams[adapter->rxCount] = gbaWord;
		++adapter->rxCount;
		--adapter->rxRemaining;
		if (!adapter->rxRemaining) {
			_executeCommand(player);
		} else {
			adapter->out = WL_IDLE_WORD;
		}
		break;
	case WL_SERIAL_RESPONSE:
		// The GBA clocks idle words while we shift the response out.
		if (adapter->txIndex < adapter->txCount) {
			adapter->out = adapter->txWords[adapter->txIndex];
			++adapter->txIndex;
		}
		if (adapter->txIndex >= adapter->txCount) {
			adapter->serial = WL_SERIAL_COMMAND;
		}
		break;
	case WL_SERIAL_WAITING:
		// The GBA is supposed to hand us the bus after the 0x25/0x27
		// ack, but it may still clock as master (probing); answer idle.
		adapter->out = WL_IDLE_WORD;
		break;
	}
	return reply;
}

// Queue a response: the ack header, plus nWords reply words already
// staged in txWords[1..]. The header is preloaded so it lands in the
// GBA's response-request transfer.
static void _respond(struct GBASIOWirelessAdapter* adapter, int nWords) {
	adapter->txWords[0] = ((uint32_t) WL_MAGIC << 16) | ((nWords & 0xFF) << 8) | (0x80 | adapter->rxCommand);
	adapter->txCount = nWords + 1;
	adapter->out = adapter->txWords[0];
	adapter->txIndex = 1;
	adapter->serial = adapter->txCount > 1 ? WL_SERIAL_RESPONSE : WL_SERIAL_COMMAND;
}

// Error ack: 0x996601EE plus one code word. Code 1 = valid command in
// the wrong state, 2 = unknown command.
static void _respondError(struct GBASIOWirelessAdapter* adapter, uint32_t code) {
	adapter->txWords[0] = ((uint32_t) WL_MAGIC << 16) | 0x01EE;
	adapter->txWords[1] = code;
	adapter->txCount = 2;
	adapter->out = adapter->txWords[0];
	adapter->txIndex = 1;
	adapter->serial = WL_SERIAL_RESPONSE;
}

// Ack a change command (0x25/0x27): normal zero-length ack, after which
// the adapter owns the bus until it delivers an event frame.
static void _respondAndWait(struct GBASIOWirelessAdapter* adapter) {
	_respond(adapter, 0);
	adapter->serial = WL_SERIAL_WAITING;
	adapter->waitPending = true;
	adapter->waitTicks = 0;
}

// Arm an event frame for delivery over adapter-clocked transfers.
static void _armEvent(struct GBASIOWirelessAdapter* adapter, uint8_t event, bool hasParam, uint32_t param) {
	adapter->waitPending = false;
	adapter->eventArmed = true;
	adapter->evWords[0] = ((uint32_t) WL_MAGIC << 16) | (hasParam ? 0x100 : 0) | event;
	adapter->evWords[1] = param;
	adapter->evCount = hasParam ? 2 : 1;
	adapter->evIndex = 0;
}

// The lowest free client slot, or -1.
static int _freeSlot(const struct GBASIOWirelessAdapter* host) {
	int i;
	for (i = 0; i < WL_MAX_CLIENTS; ++i) {
		if (!(host->clientMask & (1 << i))) {
			return i;
		}
	}
	return -1;
}

// The next-clientNumber byte hosts advertise: the slot a new client
// would get, or 0xFF when the room is closed or full.
static uint8_t _nextClientNumber(const struct GBASIOWirelessAdapter* host) {
	if (!host->hostOpen) {
		return 0xFF;
	}
	int slot = _freeSlot(host);
	return slot < 0 ? 0xFF : slot;
}

static void _packSendData(struct GBASIOWirelessAdapter* adapter) {
	// The first parameter packs the sender's own byte count; the adapter
	// stores the raw payload and rebuilds receive headers itself, so
	// only our own lane is extracted. The host's count rides bits [6:0];
	// client slot c's count rides the 5-bit lane at bit 8 + 5c.
	uint32_t header = adapter->rxParams[0];
	unsigned length;
	if (adapter->role == WL_ROLE_CLIENT && adapter->clientNumber >= 0) {
		length = (header >> (8 + 5 * adapter->clientNumber)) & 0x1F;
		if (length > WL_CLIENT_DATA_BYTES) {
			length = WL_CLIENT_DATA_BYTES;
		}
	} else {
		length = header & 0x7F;
		if (length > WL_HOST_DATA_BYTES) {
			length = WL_HOST_DATA_BYTES;
		}
	}
	unsigned available = (adapter->rxCount - 1) * 4;
	if (length > available) {
		length = available;
	}
	adapter->txData.length = length;
	memset(adapter->txData.data, 0, sizeof(adapter->txData.data));
	memcpy(adapter->txData.data, &adapter->rxParams[1], (length + 3) & ~3u);
	adapter->txDataPending = true;
}

static int _buildReceiveData(struct GBASIOWirelessAdapter* adapter) {
	// Response: one header word packing per-source byte counts (host in
	// bits [6:0], client slot c in the 5-bit lane at bit 8 + 5c), then
	// the payloads byte-concatenated in source order.
	uint32_t header = 0;
	uint8_t* out = (uint8_t*) &adapter->txWords[2];
	unsigned bytes = 0;
	int i;
	memset(&adapter->txWords[2], 0, sizeof(uint32_t) * (WL_MAX_REPLY_WORDS - 1));
	if (adapter->role == WL_ROLE_CLIENT) {
		unsigned length = adapter->rxData[0].length;
		if (length > WL_HOST_DATA_BYTES) {
			length = WL_HOST_DATA_BYTES;
		}
		header |= length;
		memcpy(out, adapter->rxData[0].data, length);
		bytes = length;
	} else if (adapter->role == WL_ROLE_HOST) {
		for (i = 0; i < WL_MAX_CLIENTS; ++i) {
			unsigned length = adapter->rxData[1 + i].length & 0x1F;
			header |= length << (8 + 5 * i);
			memcpy(&out[bytes], adapter->rxData[1 + i].data, length);
			bytes += length;
		}
	}
	if (!bytes) {
		return 0;
	}
	adapter->txWords[1] = header;
	return 1 + (bytes + 3) / 4;
}

// Execute a fully-received command. Called with the coordinator lock
// held; may read this player's state freely but must not touch other
// players — cross-adapter effects happen at RF ticks.
static void _executeCommand(struct GBASIOWirelessPlayer* player) {
	struct GBASIOWirelessAdapter* adapter = &player->adapter;
	int i;
	mLOG(GBA_SIO, DEBUG, "Wireless: command %02X with %u params", adapter->rxCommand, adapter->rxCount);
	switch (adapter->rxCommand) {
	case WL_CMD_RESET:
		_adapterIdle(adapter);
		_respond(adapter, 0);
		break;
	case WL_CMD_LINK_STATUS: {
		// One signal-strength byte per client slot; the emulated radio
		// is always perfect. The host sees every connected client; a
		// client sees only its own slot's byte.
		uint32_t signal = 0;
		if (adapter->role == WL_ROLE_HOST) {
			for (i = 0; i < WL_MAX_CLIENTS; ++i) {
				if (adapter->clientMask & (1 << i)) {
					signal |= 0xFFu << (8 * i);
				}
			}
		} else if (adapter->role == WL_ROLE_CLIENT && adapter->clientNumber >= 0) {
			signal = 0xFFu << (8 * adapter->clientNumber);
		}
		adapter->txWords[1] = signal;
		_respond(adapter, 1);
		break;
	}
	case WL_CMD_VERSION_STATUS:
		adapter->txWords[1] = WL_HW_VERSION;
		_respond(adapter, 1);
		break;
	case WL_CMD_SYSTEM_STATUS: {
		// bits 0-15 own device id (0 when idle), bits 16-19 one-hot
		// client slot, bits 24-31 adapter state.
		uint32_t status = 0;
		switch (adapter->role) {
		case WL_ROLE_HOST:
			status = _deviceId(player->playerId) | ((adapter->hostOpen ? 2u : 1u) << 24);
			break;
		case WL_ROLE_CLIENT:
			if (adapter->clientNumber >= 0) {
				status = _deviceId(player->playerId) | (1u << (16 + adapter->clientNumber)) | (5u << 24);
			} else {
				status = _deviceId(player->playerId) | (4u << 24);
			}
			break;
		case WL_ROLE_NONE:
			if (adapter->scanning) {
				status = 3u << 24;
			}
			break;
		}
		adapter->txWords[1] = status;
		_respond(adapter, 1);
		break;
	}
	case WL_CMD_SLOT_STATUS: {
		int n = 0;
		for (i = 0; i < WL_MAX_CLIENTS; ++i) {
			if (adapter->clientMask & (1 << i)) {
				adapter->txWords[2 + n] = ((uint32_t) i << 16) | _deviceId(adapter->clientPlayers[i]);
				++n;
			}
		}
		adapter->txWords[1] = _nextClientNumber(adapter);
		_respond(adapter, 1 + n);
		break;
	}
	case WL_CMD_CONFIG_STATUS:
		if (adapter->role == WL_ROLE_HOST) {
			for (i = 0; i < WL_BROADCAST_WORDS; ++i) {
				adapter->txWords[1 + i] = adapter->broadcast[i];
			}
			adapter->txWords[7] = adapter->setup;
			adapter->txWords[8] = 0x00000101;
			_respond(adapter, 8);
		} else {
			for (i = 0; i < WL_BROADCAST_WORDS; ++i) {
				adapter->txWords[1 + i] = 0;
			}
			adapter->txWords[7] = 0x00000101;
			_respond(adapter, 7);
		}
		break;
	case WL_CMD_GAME_CONFIG:
		for (i = 0; i < WL_BROADCAST_WORDS; ++i) {
			adapter->broadcast[i] = (unsigned) i < adapter->rxCount ? adapter->rxParams[i] : 0;
		}
		_respond(adapter, 0);
		break;
	case WL_CMD_SYSTEM_CONFIG:
		adapter->setup = adapter->rxCount ? adapter->rxParams[0] : 0;
		_respond(adapter, 0);
		break;
	case WL_CMD_SC_START:
		adapter->role = WL_ROLE_HOST;
		adapter->hostOpen = true;
		adapter->broadcasting = true;
		_respond(adapter, 0);
		break;
	case WL_CMD_SC_POLL:
	case WL_CMD_SC_END: {
		if (adapter->role != WL_ROLE_HOST || (adapter->rxCommand == WL_CMD_SC_POLL && !adapter->hostOpen)) {
			_respondError(adapter, 1);
			break;
		}
		int n = 0;
		for (i = 0; i < WL_MAX_CLIENTS; ++i) {
			if (adapter->clientMask & (1 << i)) {
				adapter->txWords[1 + n] = ((uint32_t) i << 16) | _deviceId(adapter->clientPlayers[i]);
				++n;
			}
		}
		if (adapter->rxCommand == WL_CMD_SC_END) {
			// Close the room: no new connections, but existing sessions
			// keep working and the broadcast stays visible (advertising
			// a full room).
			adapter->hostOpen = false;
		}
		_respond(adapter, n);
		break;
	}
	case WL_CMD_SP_START:
		adapter->scanning = true;
		adapter->scanCount = 0;
		_respond(adapter, 0);
		break;
	case WL_CMD_SP_POLL:
	case WL_CMD_SP_END: {
		int n = adapter->scanCount * (WL_BROADCAST_WORDS + 1);
		for (i = 0; i < n; ++i) {
			adapter->txWords[1 + i] = adapter->scanResults[i];
		}
		if (adapter->rxCommand == WL_CMD_SP_END) {
			adapter->scanning = false;
		}
		_respond(adapter, n);
		break;
	}
	case WL_CMD_CP_START:
		adapter->connectTarget = adapter->rxCount ? adapter->rxParams[0] & 0xFFFF : 0;
		adapter->connectPending = true;
		adapter->connectFailed = false;
		adapter->role = WL_ROLE_CLIENT;
		adapter->hostPlayer = -1;
		adapter->clientNumber = -1;
		_respond(adapter, 0);
		break;
	case WL_CMD_CP_POLL:
	case WL_CMD_CP_END:
		// bytes 0-1 own device id, byte 2 clientNumber, byte 3 status:
		// 0 done, 1 in process, 2 room closed/full.
		if (adapter->connectPending) {
			adapter->txWords[1] = 0x01000000;
		} else if (adapter->connectFailed) {
			adapter->txWords[1] = 0x02000000;
		} else if (adapter->clientNumber >= 0) {
			adapter->txWords[1] = _deviceId(player->playerId) | ((uint32_t) adapter->clientNumber << 16);
		} else {
			adapter->txWords[1] = 0x02000000;
		}
		_respond(adapter, 1);
		break;
	case WL_CMD_DATA_TX:
	case WL_CMD_DATA_TX_AND_CHANGE:
		if (adapter->rxCount) {
			_packSendData(adapter);
		}
		if (adapter->rxCommand == WL_CMD_DATA_TX_AND_CHANGE) {
			_respondAndWait(adapter);
		} else {
			_respond(adapter, 0);
		}
		break;
	case WL_CMD_DATA_RX: {
		int n = 0;
		if (adapter->rxFresh) {
			n = _buildReceiveData(adapter);
			adapter->rxFresh = false;
			for (i = 0; i < WL_MAX_CLIENTS + 1; ++i) {
				adapter->rxData[i].length = 0;
			}
		}
		_respond(adapter, n);
		break;
	}
	case WL_CMD_MS_CHANGE:
		_respondAndWait(adapter);
		break;
	case WL_CMD_DISCONNECT:
		// Parameter byte 0 is a bitmask of clientNumbers (hosts); a
		// client only ever disconnects itself. Local state clears now;
		// the affected peers observe it at the next RF tick.
		if (adapter->role == WL_ROLE_HOST && adapter->rxCount) {
			uint32_t mask = adapter->rxParams[0] & 0xF;
			for (i = 0; i < WL_MAX_CLIENTS; ++i) {
				if (mask & (1 << i)) {
					adapter->clientMask &= ~(1 << i);
					adapter->clientPlayers[i] = -1;
				}
			}
		} else if (adapter->role == WL_ROLE_CLIENT) {
			adapter->role = WL_ROLE_NONE;
			adapter->hostPlayer = -1;
			adapter->clientNumber = -1;
			adapter->connectPending = false;
		}
		_respond(adapter, 0);
		break;
	case WL_CMD_STOP_MODE:
		_respond(adapter, 0);
		// The ack still goes out normally; after that the adapter is
		// dormant until the SD reset pulse.
		if (adapter->serial == WL_SERIAL_COMMAND) {
			adapter->serial = WL_SERIAL_DORMANT;
		} else {
			// Response still draining; DORMANT is applied when it ends.
			// The response state machine returns to COMMAND, so stash
			// dormancy by clearing role and letting the drain finish;
			// simplest is to mark dormant now — the RESPONSE branch only
			// runs while txIndex < txCount, and a zero-length ack ends
			// immediately, so this path is unreachable in practice.
			adapter->serial = WL_SERIAL_DORMANT;
		}
		break;
	default:
		if (adapter->rxCommand >= 0x10 && adapter->rxCommand <= 0x3D) {
			mLOG(GBA_SIO, STUB, "Wireless: unimplemented command %02X", adapter->rxCommand);
			_respondError(adapter, 1);
		} else {
			mLOG(GBA_SIO, GAME_ERROR, "Wireless: unknown command %02X", adapter->rxCommand);
			_respondError(adapter, 2);
		}
		break;
	}
}

void GBASIOWirelessCoordinatorInit(struct GBASIOWirelessCoordinator* coordinator) {
	memset(coordinator, 0, sizeof(*coordinator));
	MutexInit(&coordinator->mutex);
	TableInit(&coordinator->players, 8, free);
	coordinator->nextRfTick = RF_TICK_INTERVAL;
}

void GBASIOWirelessCoordinatorDeinit(struct GBASIOWirelessCoordinator* coordinator) {
	MutexDeinit(&coordinator->mutex);
	TableDeinit(&coordinator->players);
}

void GBASIOWirelessCoordinatorAttach(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessDriver* driver) {
	if (driver->coordinator && driver->coordinator != coordinator) {
		abort();
	}
	driver->coordinator = coordinator;
}

void GBASIOWirelessCoordinatorDetach(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessDriver* driver) {
	if (driver->coordinator != coordinator) {
		abort();
	}
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, driver->wirelessId);
	if (player) {
		_removePlayer(coordinator, player);
	}
	MutexUnlock(&coordinator->mutex);
	driver->coordinator = NULL;
}

size_t GBASIOWirelessCoordinatorAttached(struct GBASIOWirelessCoordinator* coordinator) {
	size_t count;
	MutexLock(&coordinator->mutex);
	count = TableSize(&coordinator->players);
	MutexUnlock(&coordinator->mutex);
	return count;
}

int32_t _untilNextSync(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessPlayer* player) {
	int32_t cycle = coordinator->cycle - GBASIOWirelessTime(player);
	if (player->playerId == 0) {
		if (coordinator->nAttached < 2) {
			cycle += UNLOCKED_INTERVAL;
		} else {
			cycle += WIRELESS_INTERVAL;
		}
	}
	return cycle;
}

void _advanceCycle(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessPlayer* player) {
	int32_t newCycle = GBASIOWirelessTime(player);
	mASSERT_DEBUG(newCycle - coordinator->cycle >= 0);
	coordinator->nextRfTick -= newCycle - coordinator->cycle;
	coordinator->cycle = newCycle;
}

// Drop every connection on the airwaves and flag the drop so waiting
// clients get a disconnect event. Called with the lock held.
static void _severAll(struct GBASIOWirelessCoordinator* coordinator, bool notify) {
	int i;
	for (i = 0; i < MAX_GBAS; ++i) {
		if (!coordinator->attachedPlayers[i]) {
			continue;
		}
		struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, coordinator->attachedPlayers[i]);
		if (!player) {
			continue;
		}
		struct GBASIOWirelessAdapter* adapter = &player->adapter;
		bool wasClient = adapter->role == WL_ROLE_CLIENT && adapter->clientNumber >= 0;
		adapter->clientMask = 0;
		int j;
		for (j = 0; j < WL_MAX_CLIENTS; ++j) {
			adapter->clientPlayers[j] = -1;
		}
		if (adapter->role == WL_ROLE_CLIENT) {
			adapter->hostPlayer = -1;
			adapter->clientNumber = -1;
			adapter->connectPending = false;
		}
		if (notify && wasClient) {
			adapter->peerDropped = true;
		}
	}
}

void _removePlayer(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessPlayer* player) {
	struct GBASIOWirelessEvent event = {
		.type = WL_EV_DETACH,
		.playerId = player->playerId,
		.timestamp = GBASIOWirelessTime(player),
	};
	_enqueueEvent(coordinator, &event, TARGET_ALL & ~TARGET(player->playerId));

	coordinator->waiting = 0;

	TableRemove(&coordinator->players, player->driver->wirelessId);
	_reconfigPlayers(coordinator);

	// Detaching both severs this adapter's connections and renumbers the
	// survivors; both invalidate the playerId-based connection
	// references, so drop them all and let the games' disconnect
	// handling take it from there.
	_severAll(coordinator, true);

	struct GBASIOWirelessPlayer* runner = TableLookup(&coordinator->players, coordinator->attachedPlayers[0]);
	if (runner) {
		GBASIOWirelessPlayerWake(runner);
	}
	_verifyAwake(coordinator);
}

void _reconfigPlayers(struct GBASIOWirelessCoordinator* coordinator) {
	size_t players = TableSize(&coordinator->players);
	memset(coordinator->attachedPlayers, 0, sizeof(coordinator->attachedPlayers));
	if (players == 0) {
		mLOG(GBA_SIO, WARN, "Reconfiguring player IDs with no players attached somehow?");
	} else if (players == 1) {
		struct TableIterator iter;
		mASSERT_LOG(GBA_SIO, TableIteratorStart(&coordinator->players, &iter), "Trying to reconfigure 1 player with empty player list");
		unsigned p0 = TableIteratorGetKey(&coordinator->players, &iter);
		coordinator->attachedPlayers[0] = p0;

		struct GBASIOWirelessPlayer* player = TableIteratorGetValue(&coordinator->players, &iter);
		coordinator->cycle = mTimingCurrentTime(&player->driver->d.p->p->timing);
		coordinator->nextRfTick = RF_TICK_INTERVAL;

		if (player->playerId != 0) {
			player->playerId = 0;
			if (player->driver->user->playerIdChanged) {
				player->driver->user->playerIdChanged(player->driver->user, player->playerId);
			}
		}
	} else {
		struct UIntList playerPreferences[MAX_GBAS];

		int i;
		for (i = 0; i < MAX_GBAS; ++i) {
			UIntListInit(&playerPreferences[i], 4);
		}

		int seen = 0;
		struct TableIterator iter;
		mASSERT_LOG(GBA_SIO, TableIteratorStart(&coordinator->players, &iter), "Trying to reconfigure %" PRIz "u players with empty player list", players);
		do {
			unsigned pid = TableIteratorGetKey(&coordinator->players, &iter);
			struct GBASIOWirelessPlayer* player = TableIteratorGetValue(&coordinator->players, &iter);
			int requested = MAX_GBAS - 1;
			if (player->driver->user->requestedId) {
				requested = player->driver->user->requestedId(player->driver->user);
			}
			if (requested < 0) {
				continue;
			}
			if (requested >= MAX_GBAS) {
				requested = MAX_GBAS - 1;
			}

			*UIntListAppend(&playerPreferences[requested]) = pid;
			++seen;
		} while (TableIteratorNext(&coordinator->players, &iter) && seen < MAX_GBAS);

		seen = 0;
		for (i = 0; i < MAX_GBAS; ++i) {
			int j;
			for (j = 0; j <= i; ++j) {
				while (UIntListSize(&playerPreferences[j]) && seen < MAX_GBAS) {
					unsigned pid = *UIntListGetPointer(&playerPreferences[j], 0);
					UIntListShift(&playerPreferences[j], 0, 1);
					struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, pid);
					if (!player) {
						mLOG(GBA_SIO, ERROR, "Player list appears to have changed unexpectedly. PID %u missing.", pid);
						continue;
					}
					coordinator->attachedPlayers[seen] = pid;
					if (player->playerId != seen) {
						player->playerId = seen;
						if (player->driver->user->playerIdChanged) {
							player->driver->user->playerIdChanged(player->driver->user, player->playerId);
						}
					}
					++seen;
				}
			}
		}

		for (i = 0; i < MAX_GBAS; ++i) {
			UIntListDeinit(&playerPreferences[i]);
		}
	}

	int nAttached = 0;
	size_t i;
	for (i = 0; i < MAX_GBAS; ++i) {
		unsigned pid = coordinator->attachedPlayers[i];
		if (!pid) {
			continue;
		}
		struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, pid);
		if (!player) {
			coordinator->attachedPlayers[i] = 0;
		} else {
			++nAttached;
		}
	}
	coordinator->nAttached = nAttached;
}

void _enqueueEvent(struct GBASIOWirelessCoordinator* coordinator, const struct GBASIOWirelessEvent* event, uint32_t target) {
	mLOG(GBA_SIO, DEBUG, "Wireless: enqueuing event of type %X from %i for target %X at timestamp %X",
	                      event->type, event->playerId, target, event->timestamp);

	int i;
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!(target & TARGET(i))) {
			continue;
		}
		struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, coordinator->attachedPlayers[i]);
		mASSERT_LOG(GBA_SIO, player->freeList, "No free events");
		struct GBASIOWirelessEvent* newEvent = player->freeList;
		player->freeList = newEvent->next;

		memcpy(newEvent, event, sizeof(*event));
		struct GBASIOWirelessEvent** previous = &player->queue;
		struct GBASIOWirelessEvent* next = player->queue;
		while (next) {
			int32_t until = newEvent->timestamp - next->timestamp;
			if (until < 0) {
				break;
			}
			previous = &next->next;
			next = next->next;
		}
		newEvent->next = next;
		*previous = newEvent;
	}
}

// One RF exchange: commit every cross-adapter effect, in player order,
// while every attached core is parked at the tick. Called with the lock
// held. Only adapter structs are touched here; per-core consequences
// (transfer completions) are applied by each player's own pump when it
// wakes.
static void _rfCommit(struct GBASIOWirelessCoordinator* coordinator) {
	struct GBASIOWirelessPlayer* players[MAX_GBAS] = {0};
	struct GBASIOWirelessAdapter* byPid[MAX_GBAS] = {0};
	int i, j;
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (coordinator->attachedPlayers[i]) {
			players[i] = TableLookup(&coordinator->players, coordinator->attachedPlayers[i]);
			if (players[i]) {
				int pid = players[i]->playerId;
				if (pid >= 0 && pid < MAX_GBAS) {
					byPid[pid] = &players[i]->adapter;
				}
			}
		}
	}

	// Abandoned partial commands: a GBA that desynced mid-command has
	// long since given up; drop back to listening so its retried header
	// parses.
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!players[i]) {
			continue;
		}
		struct GBASIOWirelessAdapter* adapter = &players[i]->adapter;
		if (adapter->serial == WL_SERIAL_PARAMS || adapter->serial == WL_SERIAL_RESPONSE) {
			if (adapter->idleTicks < 0xFF) {
				++adapter->idleTicks;
			}
			if (adapter->idleTicks >= 8) {
				mLOG(GBA_SIO, DEBUG, "Wireless: abandoning stale partial command %02X", adapter->rxCommand);
				adapter->serial = WL_SERIAL_COMMAND;
				adapter->out = WL_IDLE_WORD;
				adapter->idleTicks = 0;
			}
		}
	}

	// Resolve connect requests against the hosts' current openings.
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!players[i] || !players[i]->adapter.connectPending) {
			continue;
		}
		struct GBASIOWirelessAdapter* client = &players[i]->adapter;
		struct GBASIOWirelessAdapter* host = NULL;
		int hostPid = -1;
		for (j = 0; j < MAX_GBAS; ++j) {
			if (j == players[i]->playerId || !byPid[j]) {
				continue;
			}
			if (byPid[j]->role == WL_ROLE_HOST && _deviceId(j) == client->connectTarget) {
				host = byPid[j];
				hostPid = j;
				break;
			}
		}
		client->connectPending = false;
		int slot = host && host->hostOpen ? _freeSlot(host) : -1;
		if (slot < 0) {
			client->connectFailed = true;
			continue;
		}
		host->clientMask |= 1 << slot;
		host->clientPlayers[slot] = players[i]->playerId;
		client->hostPlayer = hostPid;
		client->clientNumber = slot;
		client->connectFailed = false;
		mLOG(GBA_SIO, DEBUG, "Wireless: player %i connected to host %i as client %i", players[i]->playerId, hostPid, slot);
	}

	// Refresh broadcast scans.
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!players[i] || !players[i]->adapter.scanning) {
			continue;
		}
		struct GBASIOWirelessAdapter* scanner = &players[i]->adapter;
		scanner->scanCount = 0;
		for (j = 0; j < MAX_GBAS; ++j) {
			if (j == players[i]->playerId || !byPid[j] || !byPid[j]->broadcasting) {
				continue;
			}
			struct GBASIOWirelessAdapter* host = byPid[j];
			uint32_t* entry = &scanner->scanResults[scanner->scanCount * (WL_BROADCAST_WORDS + 1)];
			entry[0] = _deviceId(j) | ((uint32_t) _nextClientNumber(host) << 16);
			memcpy(&entry[1], host->broadcast, sizeof(host->broadcast));
			++scanner->scanCount;
		}
	}

	// Sever the client half of connections whose host half is gone (a
	// 0x30 kick, a bye, a host reset). The reverse — a client vanishing
	// under a host — is deliberately silent: real hosts learn of client
	// loss only by polling 0x11/0x14, which is exactly what librfu does,
	// except that we do free the slot so those polls see the truth.
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!players[i]) {
			continue;
		}
		struct GBASIOWirelessAdapter* adapter = &players[i]->adapter;
		if (adapter->role == WL_ROLE_CLIENT && adapter->clientNumber >= 0) {
			struct GBASIOWirelessAdapter* host = NULL;
			if (adapter->hostPlayer >= 0 && adapter->hostPlayer < MAX_GBAS) {
				host = byPid[adapter->hostPlayer];
			}
			bool live = host && host->role == WL_ROLE_HOST
			    && (host->clientMask & (1 << adapter->clientNumber))
			    && host->clientPlayers[adapter->clientNumber] == players[i]->playerId;
			if (!live) {
				adapter->role = WL_ROLE_NONE;
				adapter->hostPlayer = -1;
				adapter->clientNumber = -1;
				adapter->peerDropped = true;
			}
		} else if (adapter->role == WL_ROLE_HOST) {
			int slot;
			for (slot = 0; slot < WL_MAX_CLIENTS; ++slot) {
				if (!(adapter->clientMask & (1 << slot))) {
					continue;
				}
				int clientPid = adapter->clientPlayers[slot];
				struct GBASIOWirelessAdapter* client = clientPid >= 0 && clientPid < MAX_GBAS ? byPid[clientPid] : NULL;
				bool live = client && client->role == WL_ROLE_CLIENT
				    && client->clientNumber == slot
				    && client->hostPlayer == players[i]->playerId;
				if (!live) {
					adapter->clientMask &= ~(1 << slot);
					adapter->clientPlayers[slot] = -1;
				}
			}
		}
	}

	// RF frames: data moves only when a host transmits. The frame
	// broadcasts the host's payload to every connected client and
	// collects each client's scheduled upload.
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!players[i]) {
			continue;
		}
		struct GBASIOWirelessAdapter* host = &players[i]->adapter;
		if (host->role != WL_ROLE_HOST || !host->txDataPending) {
			continue;
		}
		host->txDataPending = false;
		host->justTransmitted = true;
		int slot;
		for (slot = 0; slot < WL_MAX_CLIENTS; ++slot) {
			if (!(host->clientMask & (1 << slot))) {
				continue;
			}
			int clientPid = host->clientPlayers[slot];
			struct GBASIOWirelessAdapter* client = clientPid >= 0 && clientPid < MAX_GBAS ? byPid[clientPid] : NULL;
			if (!client) {
				continue;
			}
			client->rxData[0] = host->txData;
			client->rxFresh = true;
			if (client->txDataPending) {
				client->txDataPending = false;
				host->rxData[1 + slot] = client->txData;
				host->rxFresh = true;
				client->txData.length = 0;
			}
		}
		host->txData.length = 0;
	}

	// Wake adapters waiting on the airwaves: a severed connection, an
	// arrived payload (client), a completed RF frame (host), or the
	// configured wait timeout.
	for (i = 0; i < coordinator->nAttached; ++i) {
		if (!players[i]) {
			continue;
		}
		struct GBASIOWirelessAdapter* adapter = &players[i]->adapter;
		if (!adapter->waitPending) {
			adapter->justTransmitted = false;
			continue;
		}
		if (adapter->peerDropped) {
			adapter->peerDropped = false;
			// byte 0: affected slot bitmask; byte 1: reason 1 (loss).
			uint32_t slots = adapter->clientNumber >= 0 ? 1u << adapter->clientNumber : 1;
			_armEvent(adapter, WL_EVENT_DISCONNECT, true, slots | 0x100);
		} else if (adapter->role == WL_ROLE_HOST && adapter->justTransmitted) {
			// byte 0: clients that acked the frame (all of them; the
			// emulated radio never drops), bits 8-11: newly inactive.
			_armEvent(adapter, WL_EVENT_DATA, true, adapter->clientMask & 0xF);
		} else if (adapter->role != WL_ROLE_HOST && adapter->rxFresh) {
			_armEvent(adapter, WL_EVENT_DATA, false, 0);
		} else {
			uint32_t timeout = adapter->setup & 0xFF;
			if (adapter->waitTicks < 0xFF) {
				++adapter->waitTicks;
			}
			if (timeout && adapter->waitTicks >= timeout * RF_TICKS_PER_FRAME) {
				_armEvent(adapter, WL_EVENT_TIMEOUT, false, 0);
			}
		}
		adapter->justTransmitted = false;
	}
}

void _wirelessEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	UNUSED(cyclesLate);
	struct GBASIOWirelessDriver* wireless = context;
	struct GBASIOWirelessCoordinator* coordinator = wireless->coordinator;
	MutexLock(&coordinator->mutex);
	struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, wireless->wirelessId);
	mASSERT_LOG(GBA_SIO, player->playerId >= 0 && player->playerId < 4, "Invalid wireless player ID %i", player->playerId);

	bool wasDetach = false;
	if (player->queue && player->queue->type == WL_EV_DETACH) {
		mLOG(GBA_SIO, DEBUG, "Wireless: player %i detached at timestamp %X, picking up the pieces",
		                      player->queue->playerId, player->queue->timestamp);
		wasDetach = true;
	}
	if (player->playerId == 0 && GBASIOWirelessTime(player) - coordinator->cycle >= 0) {
		// We are the clock owner; advance the shared clock. (If we just
		// became the owner via a detach we may briefly lag it.)
		_advanceCycle(coordinator, player);
		if (coordinator->nextRfTick < 0) {
			if (coordinator->nAttached < 2) {
				// Nobody to synchronize with; commit in place.
				_rfCommit(coordinator);
			} else if (!coordinator->waiting) {
				struct GBASIOWirelessEvent event = {
					.type = WL_EV_RF_TICK,
					.playerId = 0,
					.timestamp = GBASIOWirelessTime(player),
				};
				_enqueueEvent(coordinator, &event, TARGET_ALL & ~TARGET(0));
				GBASIOWirelessCoordinatorWaitOnPlayers(coordinator, player);
			}
			coordinator->nextRfTick += RF_TICK_INTERVAL;
		} else {
			GBASIOWirelessCoordinatorWakePlayers(coordinator);
		}
	}

	int32_t nextEvent = _untilNextSync(coordinator, player);
	while (true) {
		struct GBASIOWirelessEvent* event = player->queue;
		if (!event) {
			break;
		}
		if (event->timestamp > GBASIOWirelessTime(player)) {
			break;
		}
		player->queue = event->next;
		mLOG(GBA_SIO, DEBUG, "Wireless: got event of type %X from %i at timestamp %X",
		                      event->type, event->playerId, event->timestamp);
		switch (event->type) {
		case WL_EV_ATTACH:
		case WL_EV_DETACH:
			// Airwaves membership already changed under the coordinator
			// lock; nothing per-player to do.
			break;
		case WL_EV_RF_TICK:
			GBASIOWirelessCoordinatorAckPlayer(coordinator, player);
			break;
		}
		event->next = player->freeList;
		player->freeList = event;
	}
	if (player->queue && player->queue->timestamp - GBASIOWirelessTime(player) < nextEvent) {
		nextEvent = player->queue->timestamp - GBASIOWirelessTime(player);
	}

	// Apply any airwaves consequences to our own core: an armed event
	// frame can go out if the GBA is already parked as a ready slave.
	if (player->adapter.eventArmed) {
		_deliverPending(player);
	}

	if (player->playerId != 0 && nextEvent <= WIRELESS_INTERVAL) {
		// Park when there is nothing to do before the next sync point;
		// see lockstep.c for the full story — with every core on one
		// thread, a non-positive reschedule would spin forever.
		if (!player->queue || wasDetach || nextEvent < 1) {
			GBASIOWirelessPlayerSleep(player);
			if (nextEvent < 4) {
				nextEvent = 4;
			}
			_verifyAwake(coordinator);
		}
	}

	if (nextEvent < 1) {
		// See lockstep.c: log and clamp instead of asserting or
		// spinning.
		if (coordinator->underflows < 32) {
			++coordinator->underflows;
			mLOG(GBA_SIO, ERROR, "Wireless reschedule underflow (%i) for player %i; clamping to one interval",
			     nextEvent, player->playerId);
		}
		nextEvent = WIRELESS_INTERVAL;
	}
	MutexUnlock(&coordinator->mutex);

	mTimingSchedule(timing, &wireless->event, nextEvent);
}

int32_t GBASIOWirelessTime(struct GBASIOWirelessPlayer* player) {
	return mTimingCurrentTime(&player->driver->d.p->p->timing) - player->cycleOffset;
}

void GBASIOWirelessCoordinatorWaitOnPlayers(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessPlayer* player) {
	mASSERT_LOG(GBA_SIO, !coordinator->waiting, "Wireless desynchronized: coordinator still waiting");
	mASSERT_LOG(GBA_SIO, !player->asleep, "Wireless desynchronized: player asleep");
	mASSERT_LOG(GBA_SIO, player->playerId == 0, "Wireless desynchronized: invalid player %i attempting to coordinate", player->playerId);
	if (coordinator->nAttached < 2) {
		return;
	}

	_advanceCycle(coordinator, player);
	mLOG(GBA_SIO, DEBUG, "Wireless: primary waiting for players to ack");
	coordinator->waiting = ((1 << coordinator->nAttached) - 1) & ~TARGET(player->playerId);
	GBASIOWirelessPlayerSleep(player);
	GBASIOWirelessCoordinatorWakePlayers(coordinator);

	_verifyAwake(coordinator);
}

void GBASIOWirelessCoordinatorWakePlayers(struct GBASIOWirelessCoordinator* coordinator) {
	int i;
	for (i = 1; i < coordinator->nAttached; ++i) {
		if (!coordinator->attachedPlayers[i]) {
			continue;
		}
		struct GBASIOWirelessPlayer* player = TableLookup(&coordinator->players, coordinator->attachedPlayers[i]);
		GBASIOWirelessPlayerWake(player);
	}
}

void GBASIOWirelessPlayerWake(struct GBASIOWirelessPlayer* player) {
	if (!player->asleep) {
		return;
	}
	player->asleep = false;
	player->driver->user->wake(player->driver->user);
}

void GBASIOWirelessCoordinatorAckPlayer(struct GBASIOWirelessCoordinator* coordinator, struct GBASIOWirelessPlayer* player) {
	if (player->playerId == 0) {
		return;
	}
	coordinator->waiting &= ~TARGET(player->playerId);
	if (!coordinator->waiting) {
		mLOG(GBA_SIO, DEBUG, "Wireless: all players parked, committing RF tick and waking primary");
		// Everyone — the primary asleep in WaitOnPlayers, every other
		// secondary parked by its own ack — is at this tick: commit the
		// airwaves exchange.
		_rfCommit(coordinator);

		struct GBASIOWirelessPlayer* runner = TableLookup(&coordinator->players, coordinator->attachedPlayers[0]);
		GBASIOWirelessPlayerWake(runner);
	}
	GBASIOWirelessPlayerSleep(player);
}

void GBASIOWirelessPlayerSleep(struct GBASIOWirelessPlayer* player) {
	if (player->asleep) {
		return;
	}
	player->asleep = true;
	player->driver->user->sleep(player->driver->user);
	player->driver->d.p->p->cpu->nextEvent = 0;
	GBAInterrupt(player->driver->d.p->p);
}
