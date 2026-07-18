/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef GBA_SIO_WIRELESS_H
#define GBA_SIO_WIRELESS_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/lockstep.h>
#include <mgba/core/timing.h>
#include <mgba/internal/gba/sio.h>
#include <mgba-util/table.h>
#include <mgba-util/threading.h>

// The GBA Wireless Adapter (AGB-015, "RFU"): each attached core talks to
// its own emulated adapter over SIO NORMAL-32, entirely locally; the
// adapters share one coordinator (the airwaves), which synchronizes the
// cores only at a fixed-cadence RF tick. At a tick every adapter is
// parked, and the coordinator commits all cross-adapter state in player
// order: broadcasts publish, connect requests resolve, data mailboxes
// swap, and wait states wake. Between ticks nothing an adapter does is
// visible to any other, which is what makes the link deterministic and
// snapshot-exact under rollback (see mgba-siolink).
//
// The coordinator reuses the cooperative lockstep model: one thread
// drives every core, mLockstepUser sleep/wake park a caught-up core, and
// player 0 owns the shared clock.

#define MAX_WIRELESS_EVENTS 8

// Command payload geometry (see GBATEK "GBA Wireless Adapter" and the
// gba-link-connection LinkRawWireless documentation).
#define WL_MAX_COMMAND_WORDS 23
#define WL_MAX_REPLY_WORDS 30
#define WL_BROADCAST_WORDS 6
#define WL_MAX_CLIENTS 4
#define WL_HOST_DATA_BYTES 87
#define WL_CLIENT_DATA_BYTES 16
// How many broadcast entries one 0x1D reply carries: a reply caps at
// WL_MAX_REPLY_WORDS words and each visible host costs seven. This
// bounds a scanner's *snapshot*, not the airwaves: the coordinator
// itself holds any number of players (a union room's worth of host
// groups sharing spectrum), bounded only by the 64-bit parking
// bitmask — see GBASIOWirelessCoordinator.
#define WL_MAX_SCAN_RESULTS 4

enum GBASIOWirelessEventType {
	WL_EV_ATTACH,
	WL_EV_DETACH,
	WL_EV_RF_TICK,
};

// Where the adapter is in the GBA-facing serial protocol.
enum GBASIOWirelessSerialState {
	WL_SERIAL_LOGIN,      // NINTENDO handshake in progress
	WL_SERIAL_COMMAND,    // expecting a 0x9966 command header
	WL_SERIAL_PARAMS,     // collecting command parameter words
	WL_SERIAL_RESPONSE,   // clocking out the queued response
	WL_SERIAL_WAITING,    // 0x25/0x27 acked: adapter owns the bus next
	WL_SERIAL_DORMANT,    // 0x3D bye: dead until the SD reset pulse
};

// The adapter's role on the airwaves.
enum GBASIOWirelessRole {
	WL_ROLE_NONE,
	WL_ROLE_HOST,
	WL_ROLE_CLIENT,
};

struct GBASIOWirelessEvent {
	enum GBASIOWirelessEventType type;
	int32_t timestamp;
	struct GBASIOWirelessEvent* next;
	int playerId;
};

// One adapter's view of one remote payload: byte count plus data.
struct GBASIOWirelessMailbox {
	uint8_t length;
	uint8_t data[WL_HOST_DATA_BYTES + 1];
};

struct GBASIOWirelessAdapter {
	enum GBASIOWirelessSerialState serial;

	// Login handshake: index into the NI/NT/EN/DO/8001 sequence.
	int loginIndex;

	// Command assembly
	uint8_t rxCommand;
	uint8_t rxRemaining;
	uint8_t rxCount;
	uint32_t rxParams[WL_MAX_COMMAND_WORDS];

	// Response being clocked out; out is the word preloaded for the next
	// transfer (SPI full duplex: the reply to transfer N is decided
	// before N is clocked).
	uint32_t txWords[WL_MAX_REPLY_WORDS + 1];
	uint8_t txCount;
	uint8_t txIndex;
	uint32_t out;

	// A GBA that desyncs mid-command retries with a fresh header; the
	// real adapter falls back to listening after ~800us of silence. RF
	// ticks stand in for that clock: a partial command that spans two of
	// them is abandoned.
	uint8_t idleTicks;

	// Session state
	enum GBASIOWirelessRole role;
	bool hostOpen;                 // host accepting new clients
	bool broadcasting;
	bool scanning;
	uint32_t broadcast[WL_BROADCAST_WORDS];
	uint32_t setup;                // last 0x17 parameter, verbatim

	// Host: which client slots are filled, and by whom (playerId).
	uint8_t clientMask;
	int8_t clientPlayers[WL_MAX_CLIENTS];
	// Client: who we are connected (or connecting) to.
	int8_t hostPlayer;
	int8_t clientNumber;           // our slot on the host, -1 if none
	uint16_t connectTarget;        // serverId passed to 0x1F
	bool connectPending;           // connect resolves at the next RF tick
	bool connectFailed;

	// Outbound payload; an RF frame (a host transmission) consumes it.
	bool txDataPending;
	struct GBASIOWirelessMailbox txData;

	// Inbound payloads, filled by RF frames: slot 0 is the host's payload
	// (meaningful for clients), slots 1..4 are client payloads
	// (meaningful for the host).
	bool rxFresh;
	struct GBASIOWirelessMailbox rxData[WL_MAX_CLIENTS + 1];

	// Wait state (0x25/0x27): the adapter owes the GBA an event frame.
	bool waitPending;              // waiting for a trigger
	uint8_t waitTicks;             // RF ticks spent waiting (timeout clock)
	bool justTransmitted;          // host: an RF frame went out this tick
	bool peerDropped;              // our connection was severed; report 0x129
	// The armed event frame, delivered over adapter-clocked transfers,
	// followed by one ack transfer.
	bool eventArmed;
	bool slaveStartArmed;          // GBA is slave-ready for one transfer
	// An adapter-clocked transfer completed and the GBA's ISR owes us
	// one SO-high handshake; answer it by raising SI.
	bool slaveHandshake;
	uint8_t evCount;
	uint8_t evIndex;
	uint32_t evWords[2];

	// Latest broadcast scan snapshot, taken at an RF tick while scanning:
	// per visible host, the serverId word plus the six broadcast words.
	uint8_t scanCount;
	uint32_t scanResults[WL_MAX_SCAN_RESULTS * (WL_BROADCAST_WORDS + 1)];
};

struct GBASIOWirelessCoordinator {
	struct Table players;
	Mutex mutex;

	unsigned nextId;

	// PlayerId-ordered attachment, sized to the player table by
	// _reconfigPlayers — the airwaves have no fixed capacity. The only
	// machinery bound is the parking bitmask: `waiting` holds one bit
	// per player, so at most 63 secondaries park behind player 0.
	unsigned* attachedPlayers;
	size_t attachedSlots;
	// RF-commit scratch (players in id order, adapters by playerId),
	// resized alongside attachedPlayers so the commit allocates
	// nothing.
	struct GBASIOWirelessPlayer** tickPlayers;
	struct GBASIOWirelessAdapter** tickByPid;
	int nAttached;
	uint64_t waiting;

	int32_t cycle;
	int32_t nextRfTick;

	// Reschedule-underflow log budget; diagnostic only, never serialized.
	int underflows;
};

struct GBASIOWirelessPlayer {
	struct GBASIOWirelessDriver* driver;
	int playerId;
	bool asleep;
	int32_t cycleOffset;
	struct GBASIOWirelessEvent* queue;

	struct GBASIOWirelessAdapter adapter;

	struct GBASIOWirelessEvent buffer[MAX_WIRELESS_EVENTS];
	struct GBASIOWirelessEvent* freeList;
};

struct GBASIOWirelessDriver {
	struct GBASIODriver d;
	struct GBASIOWirelessCoordinator* coordinator;
	struct mTimingEvent event;
	unsigned wirelessId;

	struct mLockstepUser* user;
};

void GBASIOWirelessCoordinatorInit(struct GBASIOWirelessCoordinator*);
void GBASIOWirelessCoordinatorDeinit(struct GBASIOWirelessCoordinator*);

void GBASIOWirelessCoordinatorAttach(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessDriver*);
void GBASIOWirelessCoordinatorDetach(struct GBASIOWirelessCoordinator*, struct GBASIOWirelessDriver*);
size_t GBASIOWirelessCoordinatorAttached(struct GBASIOWirelessCoordinator*);

void GBASIOWirelessDriverCreate(struct GBASIOWirelessDriver*, struct mLockstepUser*);

// The game-visible adapter state alone (no link/sync bookkeeping): what
// a boot capture carries so a link rebuilt mid-session resumes every
// adapter where it was — the players walk into RF range with their
// sessions intact. Load after the driver is installed on its core.
void GBASIOWirelessDriverSaveAdapterState(struct GBASIOWirelessDriver*, void** state, size_t* size);
bool GBASIOWirelessDriverLoadAdapterState(struct GBASIOWirelessDriver*, const void* state, size_t size);

CXX_GUARD_END

#endif
