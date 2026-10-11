#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "clutch.h"

// Paddle -> clutch response curve — a port of HallSensorControl.h from the
// ESP32 main node. The curves, the piecewise bite-zone map and its rounded
// corners are carried over; what changed is the units at either end:
//
//   ESP32                                    here
//   12-bit analogRead(), hallMin/hallMax     16-bit ADC2 (hall_adc.h), an
//     on pin 5, pin 4 rescaled onto it         idle/pulled pair per paddle,
//                                              each normalised on its own
//   output: servo angle, idle..fully pulled  output: servo counts, engaged..
//                                              disengaged (clutch.h)
//   piecewise zone in hall counts / degrees  0..10000 of paddle / of clutch travel
//   HALL_IDLE_DEADBAND added at capture      the same deadband, applied when
//                                              normalising, so a capture is raw
//   settings in NVS, own save                fields in ClutchCal: one blob, one
//                                              `cal save`
//
// Who drives the clutch, as on the ESP32: the paddle only while the gearbox is
// idle; the state machine owns it during a shift. Manual mode maps the paddle
// straight onto the calibrated travel (the ESP32 mapped 0..180 deg with no
// limits — here the travel clamp still applies).
//
// Either paddle works the clutch: the further-pulled one wins. A paddle whose
// idle and pulled calibration are equal is ignored — that is how one paddle is
// switched off on a bench with only one wired. A reading well outside a
// paddle's calibrated range (a disconnected or failed sensor) is treated as
// idle and counted, rather than allowed to pull the clutch.

// Same numbering as HallResponseTypes.h, so a stored value means the same.
typedef enum {
    PADDLE_LINEAR      = 0,
    PADDLE_LOG         = 1,     // more sensitive at the start of travel
    PADDLE_EXP         = 2,     // more sensitive at the end
    PADDLE_SMOOTH      = 3,     // S-curve
    PADDLE_POWER       = 4,     // x^strength
    PADDLE_PIECEWISE   = 5,     // resolution concentrated on the bite zone
    PADDLE_CURVE_COUNT
} PaddleCurve;

typedef enum {
    PADDLE_OFF = 0,             // someone else owns the clutch
    PADDLE_CURVED,              // auto mode, gearbox idle
    PADDLE_DIRECT,              // manual mode: linear over the calibrated travel
} PaddleMode;

#define PADDLE_FULL             10000   // paddle position units: 0 idle .. 10000 pulled

// 30 counts of the ESP32's 12-bit ADC, in this one's 16 bits. Noise at rest
// inside this band reads as idle, so the clutch does not twitch off engaged.
#define HALL_IDLE_DEADBAND      480

// --- pure maps, no I/O ---

// Paddle position 0..PADDLE_FULL from two raw hall readings. *fault, if
// given, is set when a reading was rejected as out of range.
uint16_t paddle_from_halls(const ClutchCal *c, uint16_t left, uint16_t right, bool *fault);

// Paddle position -> clutch target in servo counts, through the selected
// curve. Always inside the calibrated travel.
uint16_t paddle_to_counts(const ClutchCal *c, uint16_t paddle);

// --- the live path ---

// Call every loop with fresh hall readings. Commands the clutch when the
// mode allows it and the console has not taken it over. Never blocks.
void paddle_tick(uint16_t left, uint16_t right, PaddleMode mode);

// The console's clutch commands (pos, pull, nudge) take the clutch away from
// the paddle until `release` or `paddle on` hands it back — otherwise the
// paddle would overwrite them on the next loop pass.
void paddle_set_override(bool on);
bool paddle_override();

uint16_t paddle_position();         // last computed, 0..PADDLE_FULL
uint16_t paddle_target();           // last computed clutch target, counts
uint32_t paddle_fault_count();      // readings rejected as out of range

const char *paddle_curve_name(uint16_t curve);
int         paddle_curve_parse(const char *name);   // -1 if unknown
