#pragma once
#include <stdint.h>
#include <stdbool.h>

// Shift timing — a port of ShiftLogger.h from the ESP32 main node, minus the
// two things that no longer live here: the ignition-cut relay (the rear node
// owns it) and the storage (LittleFS there, the SD session log here).
//
// A shift is timed from the moment the state machine starts it — for a
// downshift that is when the clutch starts to pull — until the rear node
// reports the expected gear. No report inside SHIFT_LOG_TIMEOUT_MS and it is
// recorded as failed.
//
// NOT the ESP32's 300 ms. That cut-off sat inside the range real shifts take —
// a downshift is a clutch pull plus a 150 ms pulse plus the report — so slow
// but good shifts were written down as "300, failed" and the log could not be
// used to compare timings. The limit here is the longest the state machine
// itself will wait: 500 ms for the clutch plus 500 ms for the shift.
//
// Pure logic: nothing here touches hardware, so it runs in the PC tests.

#define SHIFT_LOG_TIMEOUT_MS   1000
#define SHIFT_LOG_RECENT       16       // power of two

enum { SHIFT_KIND_UP = 0, SHIFT_KIND_DOWN = 1, SHIFT_KIND_NEUTRAL = 2 };

typedef struct {
    uint32_t ms;            // millis() when the shift started
    uint8_t  from, to;      // 0 = N, 1..6
    uint8_t  kind;
    bool     ok;
    uint16_t rpm;           // at the start
    uint16_t timeMs;
} ShiftRecord;

// Called by the gearbox state machine.
void shift_log_start(uint8_t from, uint8_t to, uint8_t kind);
void shift_log_gear_changed(uint8_t newGear, uint8_t oldGear);

// Call every loop pass: keeps the RPM a new shift is stamped with, and times
// out a shift nobody confirmed.
void shift_log_tick(uint16_t rpm);

uint32_t shift_log_total();
uint32_t shift_log_failed();
uint16_t shift_log_average_ms();        // successful shifts only

// Most recent records, oldest first. Returns how many were copied.
uint8_t shift_log_recent(ShiftRecord *out, uint8_t max);

const char *shift_log_kind_name(uint8_t kind);
