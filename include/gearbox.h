#pragma once
#include <stdint.h>
#include <stdbool.h>

// The gearbox state machine — a port of src/main_node/GearboxStateMachine.* in
// T89_2xESP32_front-rear_with_CanBUS. The states, the transition table, the
// timeouts and the stacked-downshift rules are carried over unchanged; that
// logic is sound and hard-won. Only the I/O underneath it changed:
//
//   ESP32                                  here
//   clutchServo->write(clutchFullyPull)    clutch_command_pull(1.0)
//   clutchServo->write(clutchIdlePos)      clutch_release()
//   clutchDisengaged (clutchVoltage <      clutch_disengaged()  — servo counts,
//     clutchDisengageV, via ADS1115)         false while feedback is stale
//   bite point: walk the servo angle out   clutch_command_bite() — the bite point
//     2 deg / 4 ms while watching volts      is now a position, so it is commanded
//                                            directly and clutch_at_bite() confirms
//   canSendShift*()                        can_send_shift_*()
//   Serial.println() on every transition   a ring of transitions, read from the
//                                            console (`gb`) — nothing prints from
//                                            the control path
//
// Pattern: 1st <-> N <-> 2nd -> 3rd -> 4th -> 5th -> 6th
//
// Shift timing goes to shift_log.h, as the ShiftLogger hooks did. Not carried
// over (yet): the matrix shift letter (no matrix driver) — the letter is kept
// here for whoever reads it. Shift timings live in the calibration blob (clutch.h), edited with `cal`.
//
// Nothing here blocks. gearbox_tick() runs every loop pass.

#define IGN_CUT_DEFAULT_MS          30      // as MainCan.h on the ESP32 node
#define MANUAL_SHIFT_DURATION_MS    100     // as ManualMode.h

// A shifting state that lasts this long is called failed. Unchanged from the
// ESP32 node. Here because it is also the upper limit on a shift timing.
#define GEARBOX_SHIFT_TIMEOUT_MS    500

typedef enum {
    // Idle states, one per gear (must stay contiguous — see _isIdle)
    GB_IDLE_NEUTRAL = 0,
    GB_IDLE_GEAR_1,
    GB_IDLE_GEAR_2,
    GB_IDLE_GEAR_3,
    GB_IDLE_GEAR_4,
    GB_IDLE_GEAR_5,
    GB_IDLE_GEAR_6,

    // Shifting states (must stay contiguous — see _isShifting)
    GB_NEUTRAL_DOWN_SHIFTING,
    GB_NEUTRAL_UP_SHIFTING,
    GB_UPSHIFTING,
    GB_DOWNSHIFT_CLUTCH_ENGAGING,
    GB_DOWNSHIFT_SHIFTING,
    // Between stacked downshifts the clutch returns only as far as the bite
    // point rather than all the way to idle, skipping the dead travel. Counts
    // as a shifting state so no unrelated shift command is accepted mid-stack.
    GB_DOWNSHIFT_TO_BITE_POINT,

    // Clutch interlock for shifts out of neutral
    GB_WAITING_FOR_CLUTCH_SHIFT_DOWN,
    GB_WAITING_FOR_CLUTCH_SHIFT_UP,

    GB_ERROR_SHIFT_TIMEOUT,

    GB_STATE_COUNT
} GearboxState;

typedef struct {
    uint32_t     ms;
    GearboxState from;
    GearboxState to;
    uint8_t      gear;          // confirmed gear at the time
    uint8_t      expected;
    uint8_t      target;        // stacked downshift target, 0 = none
} GearboxTrace;

void gearbox_init();

// Paddle presses, auto mode. Press edges only.
void gearbox_shift_up_pressed();
void gearbox_shift_down_pressed();
void gearbox_neutral_pressed();     // direction decided by the current gear

// Call every loop pass. Syncs the gear from the rear node (in both modes, as
// the ESP32 node did), and runs the state machine unless manualMode is set.
// Leaving manual mode resynchronises to the idle state for the current gear.
void gearbox_tick(bool manualMode);

// Manual mode: raw shift pulses to the rear node, no clutch, no interlock.
// One pulse at a time, MANUAL_SHIFT_DURATION_MS apart.
void gearbox_manual_shift(bool up);

// --- state ---
GearboxState gearbox_state();
const char  *gearbox_state_name(GearboxState s);
bool         gearbox_is_idle();
bool         gearbox_is_shifting();         // shifting or waiting for the clutch
uint8_t      gearbox_gear();                // 0 = N, 1..6 — as last confirmed
uint8_t      gearbox_expected_gear();
uint8_t      gearbox_target_gear();         // stacked downshift target, 0 = none
char         gearbox_shift_letter();        // U D N C, or 0 — for a display
uint32_t     gearbox_abort_count();         // downshifts refused: clutch not confirmed
uint32_t     gearbox_timeout_count();       // shifts that hit the generic timeout
uint32_t     gearbox_rejected_count();      // presses with no valid transition

// Transition history, oldest first. Returns how many entries were copied.
uint8_t gearbox_trace(GearboxTrace *out, uint8_t max);
