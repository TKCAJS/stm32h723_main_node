#pragma once
#include <stdint.h>
#include <stdbool.h>

// Clutch position, in servo counts.
//
// This is what replaced the voltage layer. On the ESP32 node the clutch's
// position was an analog reading — `clutchVoltage`, taken from the servo's
// feedback pot through a divider and an ADS1115 — and every decision about the
// clutch was a comparison against a captured voltage:
//
//     clutchPulled      = (clutchVoltage <  clutchDisengageV)
//     clutchJustEngaged = (clutchVoltage >= clutchDisengageV &&
//                          clutchVoltage <= clutchJustEngagedV)
//
// The servo now reports where it actually is, in its own counts, over its own
// CAN bus (servo_can.h). Those two predicates survive unchanged in meaning and
// are the same two predicates below; only the quantity behind them changed,
// from divided volts to servo counts. Nothing else in the node should ever
// look at a raw count.
//
// Direction is NOT baked in. The old feedback was electrically inverted —
// pulling drove the voltage DOWN — and that inversion was written into every
// comparison. Which way the servo's counts run depends on how the arm is
// mounted and which end of travel is calibrated as engaged, so the direction
// is derived from the calibration instead: whichever of engaged/disengaged is
// numerically larger defines "toward disengaged" for every threshold test.
//
// One thing the analog path could not do and this one can: tell "steady" from
// "dead". A stuck ADC reading looked exactly like a clutch holding still. A
// servo that has stopped answering is visible as such — always check
// clutch_feedback_ok() before trusting a position.

typedef struct {
    uint16_t version;
    uint16_t engagedCounts;      // paddle idle — clutch fully ENGAGED
    uint16_t disengagedCounts;   // paddle max  — clutch fully DISENGAGED
    uint16_t triggerCounts;      // past this, toward disengaged, the clutch is "pulled"
    uint16_t biteCounts;         // engaged-side edge of the just-engaged window
    uint16_t moveSpeed;          // 0x01 speed word used for runtime moves
    uint16_t reserved;           // keeps the struct 4-byte aligned; must stay 0
    uint32_t crc;                // MUST remain last
} ClutchCal;

// Load calibration from flash, or fall back to defaults. Never blocks.
bool clutch_init();

// --- live state ---
uint16_t clutch_position();        // last reported counts; check feedback first
bool     clutch_feedback_ok();     // a fresh reply from the servo
float    clutch_pull_fraction();   // 0.0 at engaged, 1.0 at disengaged, clamped

// The two triggers. Both answer false when the feedback is stale — a decision
// taken on a position nobody has confirmed is worse than no decision.
bool clutch_disengaged();
bool clutch_just_engaged();

// --- commands ---
// All clamped to the calibrated travel, so a bad number cannot drive the arm
// into a stop. Queued to the servo bus; none of these wait for anything.
void clutch_command_counts(uint16_t counts);
void clutch_command_pull(float fraction);   // 0.0 engaged .. 1.0 disengaged
void clutch_release();                      // command the engaged limit
uint16_t clutch_commanded();                // last target this node sent

// --- calibration ---
const ClutchCal *clutch_cal();

// Field names for the two below: engaged, disengaged, trigger, bite, speed.
// Both reject rather than clamp — firmware is the authority, exactly as
// calValidate() was on the ESP32 node. *err is set to a printable reason on
// failure and left alone on success.
bool clutch_cal_set(const char *field, long value, const char **err);

// Capture the servo's CURRENT reported position into a field. This is the
// direct equivalent of the web UI's "capture live voltage as the threshold"
// buttons — drive the clutch to the point that matters, then name it.
bool clutch_cal_capture(const char *field, const char **err);

void clutch_cal_defaults();
bool clutch_cal_save();     // BLOCKS on a flash erase — SAFE only, see cal_store.h
bool clutch_cal_load();
