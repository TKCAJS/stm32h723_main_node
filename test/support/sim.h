#pragma once
// The world outside gearbox.cpp / paddle.cpp / clutch.cpp, simulated: a clock,
// a clutch servo that travels at a finite speed, a rear node that confirms a
// gear some time after a shift command, the halls and the calibration flash.
//
// Included by exactly one .cpp per test program, so the definitions live here.

#include <Arduino.h>
#include "clutch.h"
#include "paddle.h"
#include "gearbox.h"
#include "cal_store.h"
#include "can_main.h"
#include "can_ids.h"
#include "servo_can.h"
#include "hall_adc.h"
#include "shift_log.h"
#include "sd_log.h"
#include <stdarg.h>
#include <stdio.h>

#define SIM_LOG_LEN 32

typedef struct {
    bool     up;
    uint16_t ms;
    uint16_t ignCutMs;
    uint8_t  target;
} SimShift;

typedef struct {
    uint32_t ms;

    // --- clutch servo ---
    uint16_t servoPos;          // where the arm is
    uint16_t servoCmd;          // where it was last told to go
    uint16_t servoRate;         // counts per ms; 0 = jammed
    bool     servoLink;         // false = no fresh feedback

    // --- rear node ---
    uint8_t  gear;
    bool     gearValid;
    bool     rearResponds;      // false = shift commands change nothing
    uint16_t rearConfirmMs;     // command -> new gear reported
    bool     confirmPending;
    uint8_t  confirmGear;
    uint32_t confirmAt;

    SimShift shifts[SIM_LOG_LEN];
    int      shiftCount;
    uint8_t  stacks[SIM_LOG_LEN];
    int      stackCount;

    uint16_t hallLeft, hallRight;
    bool     manual;
    uint16_t rpm;

    uint8_t  flash[128];
    uint32_t flashLen;          // 0 = nothing stored

    char     lastLogLine[128];  // what would have gone to the SD card
    int      logLines;
} Sim;

static Sim sim;

// ---- what the firmware calls -------------------------------------------------

uint32_t millis() { return sim.ms; }

uint16_t servo_position() { return sim.servoPos; }
bool     servo_link_ok()  { return sim.servoLink; }
void     servo_command_position(uint16_t counts, uint16_t) { sim.servoCmd = counts; }

void sd_log_line(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(sim.lastLogLine, sizeof(sim.lastLogLine), fmt, ap);
    va_end(ap);
    sim.logLines++;
}

uint16_t hall_read_left()  { return sim.hallLeft; }
uint16_t hall_read_right() { return sim.hallRight; }

bool cal_store_read(void *blob, uint32_t len) {
    if (sim.flashLen != len) return false;
    memcpy(blob, sim.flash, len);
    return true;
}

bool cal_store_write(const void *blob, uint32_t len) {
    if (len > sizeof(sim.flash)) return false;
    memcpy(sim.flash, blob, len);
    sim.flashLen = len;
    return true;
}

static void _simShift(bool up, uint16_t ms, uint16_t ignCutMs, uint8_t target) {
    if (sim.shiftCount < SIM_LOG_LEN) sim.shifts[sim.shiftCount] = { up, ms, ignCutMs, target };
    sim.shiftCount++;
    if (!sim.rearResponds) return;
    sim.confirmPending = true;
    sim.confirmGear    = (target != GEAR_UNKNOWN) ? target : (uint8_t)(sim.gear + (up ? 1 : -1));
    sim.confirmAt      = sim.ms + sim.rearConfirmMs;
}

void can_send_shift_up(uint16_t ms, uint16_t ignCutMs, uint8_t target) { _simShift(true, ms, ignCutMs, target); }
void can_send_shift_down(uint16_t ms, uint8_t target)                  { _simShift(false, ms, 0, target); }
void can_send_shift_stack(uint8_t target) {
    if (sim.stackCount < SIM_LOG_LEN) sim.stacks[sim.stackCount] = target;
    sim.stackCount++;
}

uint8_t can_gear()       { return sim.gear; }
bool    can_gear_valid() { return sim.gearValid; }

// ---- what the tests call -----------------------------------------------------

// One millisecond: the servo moves, the rear node may confirm, the gearbox runs.
static void sim_step() {
    sim.ms++;

    int32_t d = (int32_t)sim.servoCmd - (int32_t)sim.servoPos;
    if (d >  (int32_t)sim.servoRate) d =  (int32_t)sim.servoRate;
    if (d < -(int32_t)sim.servoRate) d = -(int32_t)sim.servoRate;
    sim.servoPos = (uint16_t)(sim.servoPos + d);

    if (sim.confirmPending && (int32_t)(sim.ms - sim.confirmAt) >= 0) {
        sim.confirmPending = false;
        sim.gear = sim.confirmGear;
    }

    gearbox_tick(sim.manual);
    shift_log_tick(sim.rpm);
}

static void sim_run(uint32_t ms) { while (ms--) sim_step(); }

// Run until the gearbox reaches a state, or give up. True if it got there.
static bool sim_run_until(GearboxState s, uint32_t maxMs) {
    while (maxMs--) {
        if (gearbox_state() == s) return true;
        sim_step();
    }
    return gearbox_state() == s;
}

// A healthy bike sitting in `gear`: factory calibration, clutch engaged, servo
// answering, rear node confirming 250 ms after a command (after a 150 ms pulse).
// gearbox.cpp keeps its state in file statics with no reset, so this goes
// through manual mode and back, which is its own documented resync.
static void sim_reset(uint8_t gear) {
    uint32_t now = sim.ms;
    memset(&sim, 0, sizeof(sim));
    sim.ms = now + 1000;

    clutch_cal_defaults();
    sim.servoPos  = sim.servoCmd = clutch_cal()->engagedCounts;
    sim.servoRate = 200;        // full travel in about 86 ms
    sim.servoLink = true;

    sim.gear = gear;
    sim.gearValid = true;
    sim.rearResponds = true;
    sim.rearConfirmMs = 250;

    paddle_set_override(false);
    clutch_release();
    gearbox_tick(true);
    gearbox_tick(false);
    // Let any shift the last test left being timed run out, so the next
    // test's totals start clean.
    sim.ms += SHIFT_LOG_TIMEOUT_MS;
    shift_log_tick(0);
    sim.shiftCount = sim.stackCount = sim.logLines = 0;
}
