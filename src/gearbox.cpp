#include <Arduino.h>
#include "gearbox.h"
#include "clutch.h"
#include "can_main.h"
#include "can_ids.h"
#include "shift_log.h"

// ---------------------------------------------------------------------------
// Timeouts — unchanged from the ESP32 node
// ---------------------------------------------------------------------------
// (GEARBOX_SHIFT_TIMEOUT_MS is in gearbox.h.)

// The driver is being waited on to pull the clutch by hand (neutral paths).
#define CLUTCH_WAIT_TIMEOUT_MS      200

// The servo is being waited on to physically pull the clutch past the trigger
// (downshift path). Must comfortably exceed real servo travel. On expiry the
// shift is ABORTED, never sent: an unconfirmed clutch means shifting against
// drive.
#define DOWNSHIFT_SERVO_TIMEOUT_MS  500

// Cap on the return to the bite point between stacked downshifts. On expiry
// the clutch returns fully to idle rather than aborting — a missed bite point
// costs speed, not the shift. Also bounds the wait if CAN never confirms.
#define STACK_BITE_TIMEOUT_MS       300

#define TRACE_LEN                   16      // power of two

typedef enum {
    EV_NEUTRAL_DOWN_PRESSED,
    EV_NEUTRAL_UP_PRESSED,
    EV_SHIFT_DOWN_PRESSED,
    EV_SHIFT_UP_PRESSED,
    EV_CLUTCH_PULLED,
    EV_GEAR_CHANGED,
    EV_TIMEOUT,
} GearboxEvent;

typedef struct {
    GearboxState from;
    GearboxEvent event;
    GearboxState to;
} Transition;

// Shift completion is not in this table: _updateShiftTiming() transitions
// straight to the idle state for the expected gear when the timing expires.
static const Transition _table[] = {
    // FROM NEUTRAL — between 1st and 2nd, full shift either way (clutch required)
    { GB_IDLE_NEUTRAL, EV_SHIFT_DOWN_PRESSED, GB_WAITING_FOR_CLUTCH_SHIFT_DOWN },  // N -> 1st
    { GB_IDLE_NEUTRAL, EV_SHIFT_UP_PRESSED,   GB_WAITING_FOR_CLUTCH_SHIFT_UP   },  // N -> 2nd

    // FROM 1ST
    { GB_IDLE_GEAR_1, EV_NEUTRAL_UP_PRESSED, GB_NEUTRAL_UP_SHIFTING },             // 1st -> N (no clutch)
    { GB_IDLE_GEAR_1, EV_SHIFT_UP_PRESSED,   GB_UPSHIFTING          },             // 1st -> 2nd (ign cut)

    // FROM 2ND
    { GB_IDLE_GEAR_2, EV_NEUTRAL_DOWN_PRESSED, GB_NEUTRAL_DOWN_SHIFTING     },     // 2nd -> N (no clutch)
    { GB_IDLE_GEAR_2, EV_SHIFT_DOWN_PRESSED,   GB_DOWNSHIFT_CLUTCH_ENGAGING },     // 2nd -> 1st (servo, skip N)
    { GB_IDLE_GEAR_2, EV_SHIFT_UP_PRESSED,     GB_UPSHIFTING                },     // 2nd -> 3rd (ign cut)

    // FROM 3RD-5TH — up/down only, no neutral access
    { GB_IDLE_GEAR_3, EV_SHIFT_DOWN_PRESSED, GB_DOWNSHIFT_CLUTCH_ENGAGING },
    { GB_IDLE_GEAR_3, EV_SHIFT_UP_PRESSED,   GB_UPSHIFTING                },
    { GB_IDLE_GEAR_4, EV_SHIFT_DOWN_PRESSED, GB_DOWNSHIFT_CLUTCH_ENGAGING },
    { GB_IDLE_GEAR_4, EV_SHIFT_UP_PRESSED,   GB_UPSHIFTING                },
    { GB_IDLE_GEAR_5, EV_SHIFT_DOWN_PRESSED, GB_DOWNSHIFT_CLUTCH_ENGAGING },
    { GB_IDLE_GEAR_5, EV_SHIFT_UP_PRESSED,   GB_UPSHIFTING                },

    // FROM 6TH — top gear, down only
    { GB_IDLE_GEAR_6, EV_SHIFT_DOWN_PRESSED, GB_DOWNSHIFT_CLUTCH_ENGAGING },

    // CLUTCH INTERLOCK — shifts out of neutral wait for the clutch (abort on timeout)
    { GB_WAITING_FOR_CLUTCH_SHIFT_DOWN, EV_CLUTCH_PULLED, GB_NEUTRAL_DOWN_SHIFTING },
    { GB_WAITING_FOR_CLUTCH_SHIFT_DOWN, EV_TIMEOUT,       GB_IDLE_NEUTRAL          },
    { GB_WAITING_FOR_CLUTCH_SHIFT_UP,   EV_CLUTCH_PULLED, GB_NEUTRAL_UP_SHIFTING   },
    { GB_WAITING_FOR_CLUTCH_SHIFT_UP,   EV_TIMEOUT,       GB_IDLE_NEUTRAL          },

    // RACING DOWNSHIFT — the shift fires the moment the clutch is confirmed pulled
    { GB_DOWNSHIFT_CLUTCH_ENGAGING, EV_CLUTCH_PULLED, GB_DOWNSHIFT_SHIFTING },
};
#define TABLE_LEN (sizeof(_table) / sizeof(_table[0]))

static GearboxState _state       = GB_IDLE_NEUTRAL;
static uint32_t     _stateStart  = 0;

static int  _gear      = 0;     // 0 = N, 1..6 — confirmed by CAN
static int  _expected  = 0;     // gear expected once the current shift completes
static int  _target    = 0;     // stacked downshift target, 0 = none

static bool     _shiftActive  = false;
static bool     _shiftIsUp    = false;
static uint32_t _shiftStart   = 0;
static uint16_t _shiftDur     = 0;

static bool     _wasManual    = false;
static bool     _manualActive = false;
static uint32_t _manualStart  = 0;

static char     _letter       = 0;

static uint32_t _aborts = 0, _timeouts = 0, _rejected = 0;

static GearboxTrace _trace[TRACE_LEN];
static uint32_t     _traceSeq = 0;

static bool _processEvent(GearboxEvent ev);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool _isIdle(GearboxState s)     { return s >= GB_IDLE_NEUTRAL && s <= GB_IDLE_GEAR_6; }
static bool _isShifting(GearboxState s) { return s >= GB_NEUTRAL_DOWN_SHIFTING && s <= GB_DOWNSHIFT_TO_BITE_POINT; }

static GearboxState _idleFor(int gear) {
    if (gear < 0 || gear > 6) return GB_IDLE_NEUTRAL;
    return (GearboxState)(GB_IDLE_NEUTRAL + gear);
}

static uint32_t _elapsed() { return millis() - _stateStart; }

static void _clearStack() {
    if (_target > 0) {
        _target = 0;
        can_send_shift_stack(0);
    }
}

static void _activateShift(bool up, uint16_t ms, uint16_t ignCutMs, uint8_t target) {
    if (up) can_send_shift_up(ms, ignCutMs, target);
    else    can_send_shift_down(ms, target);
    _shiftActive = true;
    _shiftIsUp   = up;
    _shiftStart  = millis();
    _shiftDur    = ms;
}

// ---------------------------------------------------------------------------
// State entry
// ---------------------------------------------------------------------------

static void _enterIdle() {
    clutch_release();
    _shiftActive = false;
}

static void _enterShifting() {
    // Track the expected gear so completion lands in the right idle state.
    int from = _gear;
    switch (_state) {
        case GB_NEUTRAL_DOWN_SHIFTING:     _expected = (from == 2) ? 0 : 1; break;
        case GB_NEUTRAL_UP_SHIFTING:       _expected = (from == 1) ? 0 : 2; break;
        case GB_UPSHIFTING:                _expected = from + 1;            break;
        case GB_DOWNSHIFT_CLUTCH_ENGAGING:
        case GB_DOWNSHIFT_SHIFTING:        _expected = from - 1;            break;
        default:                           _expected = from;                break;
    }

    const ClutchCal *t = clutch_cal();
    switch (_state) {
        case GB_NEUTRAL_DOWN_SHIFTING:
            // Downshift solenoid: 2nd -> N (half-shift) or N -> 1st (full shift)
            shift_log_start((uint8_t)from, (uint8_t)_expected, (from == 2) ? SHIFT_KIND_NEUTRAL : SHIFT_KIND_DOWN);
            _activateShift(false, (from == 0) ? t->shiftDownMs : t->neutralDownMs,
                           0, (uint8_t)_expected);
            _letter = (_expected == 0) ? 'N' : 'D';
            break;

        case GB_NEUTRAL_UP_SHIFTING:
            // Upshift solenoid, no ignition cut: 1st -> N (half-shift) or N -> 2nd (full shift)
            shift_log_start((uint8_t)from, (uint8_t)_expected, (from == 1) ? SHIFT_KIND_NEUTRAL : SHIFT_KIND_UP);
            _activateShift(true, (from == 0) ? t->shiftUpMs : t->neutralUpMs,
                           0, (uint8_t)_expected);
            _letter = (_expected == 0) ? 'N' : 'U';
            break;

        case GB_UPSHIFTING:
            shift_log_start((uint8_t)from, (uint8_t)_expected, SHIFT_KIND_UP);
            _activateShift(true, t->shiftUpMs, t->ignCutMs, (uint8_t)_expected);
            _letter = 'U';
            break;

        case GB_DOWNSHIFT_CLUTCH_ENGAGING:
            // Pull the clutch; the shift fires when the servo confirms it.
            // Timed from here, so a downshift's time includes the clutch pull.
            shift_log_start((uint8_t)from, (uint8_t)_expected, SHIFT_KIND_DOWN);
            clutch_command_pull(1.0f);
            _letter = 'D';
            break;

        case GB_DOWNSHIFT_SHIFTING:
            _activateShift(false, t->shiftDownMs, 0, (uint8_t)_expected);
            break;

        default:
            break;
    }
}

static void _enterBitePoint() {
    // On the ESP32 the bite point was a voltage with no volts-to-degrees map,
    // so the servo was walked out a step at a time while the feedback was
    // watched. It is a position now, so it is simply commanded; the servo's
    // own speed setting shapes the stroke.
    clutch_command_bite();
    _letter = 'D';
}

static void _enterError() {
    _shiftActive = false;
    clutch_release();
    _clearStack();      // aborted shift — drop any queued downshifts
}

static void _transitionTo(GearboxState next) {
    if (next == _state) return;

    GearboxTrace *t = &_trace[_traceSeq++ & (TRACE_LEN - 1)];
    t->ms       = millis();
    t->from     = _state;
    t->to       = next;
    t->gear     = (uint8_t)_gear;
    t->expected = (uint8_t)_expected;
    t->target   = (uint8_t)_target;

    _state      = next;
    _stateStart = millis();

    if (_isIdle(next)) {
        _enterIdle();
    } else if (next == GB_DOWNSHIFT_TO_BITE_POINT) {
        // Deliberately NOT through _enterShifting(): that recomputes the
        // expected gear from _gear, which still holds the old gear until CAN
        // confirms — it would rewind _expected to the gear just left.
        _enterBitePoint();
    } else if (_isShifting(next)) {
        _enterShifting();
    } else if (next == GB_WAITING_FOR_CLUTCH_SHIFT_DOWN || next == GB_WAITING_FOR_CLUTCH_SHIFT_UP) {
        _letter = 'C';
    } else if (next == GB_ERROR_SHIFT_TIMEOUT) {
        _enterError();
    }
}

// ---------------------------------------------------------------------------
// State update
// ---------------------------------------------------------------------------

static void _updateDownshiftClutchWait() {
    // The servo is travelling. Only once its feedback confirms the clutch is
    // actually pulled is it safe to send the downshift. clutch_disengaged() is
    // a threshold crossing and answers false on stale feedback.
    if (clutch_disengaged()) {
        _processEvent(EV_CLUTCH_PULLED);
        return;
    }

    // Never send the shift unconfirmed — abort instead. _enterError() returns
    // the clutch and drops any stack; the error state then recovers to the
    // current gear.
    if (_elapsed() >= DOWNSHIFT_SERVO_TIMEOUT_MS) {
        _aborts++;
        _transitionTo(GB_ERROR_SHIFT_TIMEOUT);
    }
}

static void _updateToBitePoint() {
    // At the bite point: hold. The next shift fires from here the moment CAN
    // confirms the gear (gearbox_tick's gear sync).
    if (clutch_at_bite()) return;

    // Not there in time: return fully to idle, so a mis-set bite point or a
    // slow servo costs speed rather than the shift. Landing in an idle state
    // also hands the stack back to the ordinary path.
    if (_elapsed() >= STACK_BITE_TIMEOUT_MS) {
        _transitionTo(_idleFor(_expected));
    }
}

static void _updateWaiting() {
    if (clutch_disengaged()) {
        _processEvent(EV_CLUTCH_PULLED);
    } else if (_elapsed() >= CLUTCH_WAIT_TIMEOUT_MS) {
        _clearStack();
        _processEvent(EV_TIMEOUT);
    }
}

static void _updateShiftTiming() {
    if (!_shiftActive || (millis() - _shiftStart) < _shiftDur) return;
    _shiftActive = false;

    // With a stacked downshift still pending, hold at the bite point instead
    // of returning to idle — idle releases the clutch fully and the next shift
    // would have to pull back through the whole dead travel.
    if (!_shiftIsUp && _target > 0 && _expected > _target) {
        _transitionTo(GB_DOWNSHIFT_TO_BITE_POINT);
        return;
    }

    // Straight to the expected gear's idle state — don't wait for CAN, which
    // arrives later and would leave a gap where the next press lands wrong.
    _transitionTo(_idleFor(_expected));
}

static void _checkTimeouts() {
    // These two run their own timeouts and must not also be caught here.
    if (_state == GB_DOWNSHIFT_CLUTCH_ENGAGING) return;
    if (_state == GB_DOWNSHIFT_TO_BITE_POINT)   return;

    if (_isShifting(_state) && _elapsed() >= GEARBOX_SHIFT_TIMEOUT_MS) {
        _timeouts++;
        _transitionTo(GB_ERROR_SHIFT_TIMEOUT);
    }
}

static void _updateState() {
    switch (_state) {
        case GB_DOWNSHIFT_CLUTCH_ENGAGING:      _updateDownshiftClutchWait(); break;
        case GB_DOWNSHIFT_TO_BITE_POINT:        _updateToBitePoint();         break;
        case GB_WAITING_FOR_CLUTCH_SHIFT_DOWN:
        case GB_WAITING_FOR_CLUTCH_SHIFT_UP:    _updateWaiting();             break;
        case GB_ERROR_SHIFT_TIMEOUT:
            // RACING: recover immediately, to the idle state for the real gear.
            _transitionTo(_idleFor(_gear));
            break;
        default:
            // Idle states are event-driven; shifting states complete via
            // _updateShiftTiming().
            break;
    }
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

static bool _processEvent(GearboxEvent ev) {
    // Stacked downshift: a down press while any shift is in progress queues.
    if (ev == EV_SHIFT_DOWN_PRESSED && !_isIdle(_state)) {
        _target = (_target == 0) ? max(1, _expected - 1) : max(1, _target - 1);
        can_send_shift_stack((uint8_t)_target);
        return true;
    }

    // An up press always cancels a pending stack.
    if (ev == EV_SHIFT_UP_PRESSED && _target > 0) _clearStack();

    if (ev == EV_GEAR_CHANGED) {
        _transitionTo(_idleFor(_gear));
        return true;
    }

    for (unsigned i = 0; i < TABLE_LEN; i++) {
        if (_table[i].from == _state && _table[i].event == ev) {
            _transitionTo(_table[i].to);
            return true;
        }
    }
    _rejected++;
    return false;
}

static void _setGear(int gear) {
    if (gear < 0 || gear > 6 || gear == _gear) return;
    int old = _gear;
    _gear = gear;

    _processEvent(EV_GEAR_CHANGED);
    shift_log_gear_changed((uint8_t)gear, (uint8_t)old);

    // Stacked downshift: CAN confirmed the gear — fire the next or clear.
    if (_target > 0) {
        if (_gear > _target && _state == GB_DOWNSHIFT_TO_BITE_POINT) {
            // Holding at the bite point. Transition straight in — _processEvent()
            // must NOT be used here: outside an idle state it treats a down
            // press as another stack request.
            _transitionTo(GB_DOWNSHIFT_CLUTCH_ENGAGING);
        } else if (_gear > _target && _isIdle(_state)) {
            _processEvent(EV_SHIFT_DOWN_PRESSED);
        } else {
            _clearStack();      // reached the target, or the shift failed
        }
    }
}

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

void gearbox_init() {
    _state      = _idleFor(_gear);
    _stateStart = millis();
}

void gearbox_shift_up_pressed()   { _processEvent(EV_SHIFT_UP_PRESSED); }
void gearbox_shift_down_pressed() { _processEvent(EV_SHIFT_DOWN_PRESSED); }

void gearbox_neutral_pressed() {
    if      (_gear == 1) _processEvent(EV_NEUTRAL_UP_PRESSED);
    else if (_gear == 2) _processEvent(EV_NEUTRAL_DOWN_PRESSED);
    else                 _rejected++;   // not in 1st or 2nd
}

void gearbox_tick(bool manualMode) {
    // The rear node is authoritative for gear position, in either mode.
    if (can_gear_valid()) _setGear((int)can_gear());

    if (manualMode) {
        if (_manualActive && (millis() - _manualStart) >= MANUAL_SHIFT_DURATION_MS)
            _manualActive = false;
        _wasManual = true;
        return;
    }

    if (_wasManual) {
        // Back from manual: whatever state was left behind is stale.
        _wasManual    = false;
        _shiftActive  = false;
        _clearStack();
        _transitionTo(_idleFor(_gear));
    }

    _updateShiftTiming();
    _checkTimeouts();
    _updateState();
}

void gearbox_manual_shift(bool up) {
    if (_manualActive) return;
    if (up) can_send_shift_up(MANUAL_SHIFT_DURATION_MS, IGN_CUT_DEFAULT_MS, GEAR_UNKNOWN);
    else    can_send_shift_down(MANUAL_SHIFT_DURATION_MS, GEAR_UNKNOWN);
    _manualActive = true;
    _manualStart  = millis();
}

GearboxState gearbox_state()        { return _state; }
bool     gearbox_is_idle()          { return _isIdle(_state); }
bool     gearbox_is_shifting()      { return _isShifting(_state) ||
                                             _state == GB_WAITING_FOR_CLUTCH_SHIFT_DOWN ||
                                             _state == GB_WAITING_FOR_CLUTCH_SHIFT_UP; }
uint8_t  gearbox_gear()             { return (uint8_t)_gear; }
uint8_t  gearbox_expected_gear()    { return (uint8_t)_expected; }
uint8_t  gearbox_target_gear()      { return (uint8_t)_target; }
char     gearbox_shift_letter()     { return _letter; }
uint32_t gearbox_abort_count()      { return _aborts; }
uint32_t gearbox_timeout_count()    { return _timeouts; }
uint32_t gearbox_rejected_count()   { return _rejected; }

const char *gearbox_state_name(GearboxState s) {
    switch (s) {
        case GB_IDLE_NEUTRAL:                   return "IDLE_N";
        case GB_IDLE_GEAR_1:                    return "IDLE_1";
        case GB_IDLE_GEAR_2:                    return "IDLE_2";
        case GB_IDLE_GEAR_3:                    return "IDLE_3";
        case GB_IDLE_GEAR_4:                    return "IDLE_4";
        case GB_IDLE_GEAR_5:                    return "IDLE_5";
        case GB_IDLE_GEAR_6:                    return "IDLE_6";
        case GB_NEUTRAL_DOWN_SHIFTING:          return "NEUTRAL_DOWN_SHIFTING";
        case GB_NEUTRAL_UP_SHIFTING:            return "NEUTRAL_UP_SHIFTING";
        case GB_UPSHIFTING:                     return "UPSHIFTING";
        case GB_DOWNSHIFT_CLUTCH_ENGAGING:      return "DOWNSHIFT_CLUTCH_ENGAGING";
        case GB_DOWNSHIFT_SHIFTING:             return "DOWNSHIFT_SHIFTING";
        case GB_DOWNSHIFT_TO_BITE_POINT:        return "DOWNSHIFT_TO_BITE_POINT";
        case GB_WAITING_FOR_CLUTCH_SHIFT_DOWN:  return "WAIT_CLUTCH_DOWN";
        case GB_WAITING_FOR_CLUTCH_SHIFT_UP:    return "WAIT_CLUTCH_UP";
        case GB_ERROR_SHIFT_TIMEOUT:            return "ERROR_SHIFT_TIMEOUT";
        default:                                return "?";
    }
}

uint8_t gearbox_trace(GearboxTrace *out, uint8_t max) {
    uint32_t n     = _traceSeq < TRACE_LEN ? _traceSeq : TRACE_LEN;
    if (n > max) n = max;
    uint32_t first = _traceSeq - n;
    for (uint32_t i = 0; i < n; i++) out[i] = _trace[(first + i) & (TRACE_LEN - 1)];
    return (uint8_t)n;
}
