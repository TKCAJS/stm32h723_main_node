#pragma once
#include <stdint.h>
#include <stdbool.h>

// FDCAN2 on PB12/PB13 — the clutch servo's own bus. 250 kbit/s classic,
// 29-bit extended IDs, acceptance filter fully open.
//
// This is the STM32 port of ServoBus.cpp from esp32_T89Display_cansniff, the
// tool that proved the protocol on the bench. The transport changes (FDCAN2
// instead of TWAI, one thread instead of two) and the frame codec does not:
// lib/asmg_servo/asmg_md_can.h is the same file, byte for byte, as the one the
// sniffer was validated with. Corrections belong there, in one place, copied
// to every repo that carries it — exactly as can_ids.h is handled.
//
// Why a second controller and not another address on the main bus: see the
// SERVO CAN section of pins.h.
//
// Nothing here blocks. TX queues into the hardware FIFO and returns; RX drains
// whatever arrived; a servo that never answers shows up as a timeout counter
// climbing, never as a wait. There is exactly one outstanding request at a
// time — every reply shares one arbitration ID, so replies can only be told
// apart by which request is in flight.
//
// The acceptance filter is left open on purpose. The reply arbitration ID is
// still a [VERIFY] item in the codec, and a filter set to an ID the servo does
// not actually use would drop every reply silently while looking healthy.

typedef struct {
    uint32_t seq;
    uint32_t ms;         // millis() at capture
    uint32_t id;         // arbitration ID
    uint8_t  dlc;
    uint8_t  dir;        // SERVO_TRACE_RX / _TX / _TX_FAIL
    uint8_t  data[8];
} ServoTrace;

enum { SERVO_TRACE_RX = 0, SERVO_TRACE_TX = 1, SERVO_TRACE_TX_FAIL = 2 };

typedef struct {
    // --- link ---
    bool     replyFresh;      // a decoded reply within SERVO_STALE_MS
    uint32_t lastAnyRxId;     // arbitration ID of the last frame seen, whatever it was
    uint32_t lastGoodMs;      // millis() of the last decoded reply, 0 = never
    uint32_t timeouts;        // requests that went unanswered
    uint32_t txFails;         // TX FIFO full / peripheral refused
    uint32_t rxCount;
    uint32_t txCount;
    uint32_t latLastUs;       // request -> reply, microseconds
    uint32_t latMaxUs;

    // --- decoded values (raw device units) ---
    uint16_t position;        // 0x02 / 0x07 current position, counts
    uint16_t cmdPosition;     // 0x02 commanded position
    uint16_t current;         // 0x07 present current
    uint16_t torque;          // 0x04 present torque
    uint16_t currentLimit;    // 0x04 configured limit
    uint16_t pidP, pidI, pidD;
    uint8_t  discoveredAddr;  // 0xFD reply byte 0 (provisional decode)
    uint8_t  replyEchoCmd04;  // which cmd byte the 0x04 reply actually echoed
    bool     havePos, haveCmdPos, haveCur, haveCfg, havePid, haveAddr;

    // --- controller health ---
    bool     busOff;
    bool     errPassive;
    uint32_t busErrTx, busErrRx;
    uint32_t busRecoveries;
} ServoStatus;

// A decoded reply older than this means the feedback is STALE, not steady —
// the same distinction the ADS1115 fault counter used to draw.
#define SERVO_STALE_MS   200

bool servo_can_init();

// Drain RX, time out a dead request, send whatever is due. Call every loop.
void servo_can_poll();

// Live status. Single-threaded by construction (everything runs from loop()),
// so this is a pointer, not a snapshot copy.
const ServoStatus *servo_status();

// True while a decoded reply is fresh. This is the only honest "is the clutch
// feedback real" test — a position that has stopped updating still reads as a
// plausible number.
bool servo_link_ok();

// Last reported position in counts (0..0x7FFF). Meaningless unless
// servo_link_ok(); check first.
uint16_t servo_position();

// --- commands ---
// Target position. Never waits on a poll reply: a clutch command that queued
// behind telemetry would be a stall in the shift path by another name.
void servo_command_position(uint16_t counts, uint16_t speed);
void servo_request_read_id();     // 0xFD, broadcast, read-only
void servo_request_read_pid();    // 0x06
void servo_request_read_cfg();    // 0x04
void servo_clear_counters();

// --- bench controls (the sniffer UI's knobs, now on the serial console) ---
void     servo_set_address(uint8_t addr);
uint8_t  servo_address();
void     servo_set_big_endian(bool be);    // runtime word order — bring-up step 3
bool     servo_big_endian();
void     servo_set_polling(bool on);
bool     servo_polling();
void     servo_set_poll_cmd(uint8_t cmd);  // 0x07 pos+current, or 0x02 pos+cmdpos
uint8_t  servo_poll_cmd();
void     servo_set_poll_interval_ms(uint16_t ms);
uint16_t servo_poll_interval_ms();

// Frame trace. Pass the last seq you have seen; new entries are copied out and
// *lastSeq is advanced. Readers that fall behind skip, they never block a
// writer.
uint32_t servo_trace_read(uint32_t *lastSeq, ServoTrace *out, uint32_t maxOut);

// Sequence number the next captured frame will carry. Start a reader here to
// follow the bus from now on rather than replaying what is still in the ring.
uint32_t servo_trace_seq();
