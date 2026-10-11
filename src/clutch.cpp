#include <Arduino.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "clutch.h"
#include "cal_store.h"
#include "servo_can.h"
#include "asmg_md_can.h"
#include "hall_adc.h"
#include "paddle.h"
#include "gearbox.h"

#define CLUTCH_CAL_VERSION  3       // 2: paddle response fields added
                                    // 3: gearbox shift timings added

// Factory travel, carried across from the ESP32 node rather than invented.
// There the limits were servo ANGLES, 42 deg and 137 deg of a 0-180 range
// (CLUTCH_SERVO_DEFAULT_MIN/MAX in CalConfig.h). The same fractions of this
// servo's 0..0x7FFF count range land here. They are a starting point to
// calibrate away from, not numbers to trust: as the clutch wears the real stop
// moves, and finding it means widening the limits and easing the arm out until
// it stops moving.
#define DEFAULT_ENGAGED     ((uint16_t)(asmg::kPositionMax * 42 / 180))   // 7645
#define DEFAULT_DISENGAGED  ((uint16_t)(asmg::kPositionMax * 137 / 180))  // 24939

// Not fastest. Until the linkage is proven, a servo with enough torque to
// break it should not be told to go there as quickly as it can.
#define DEFAULT_SPEED       0x0100

static ClutchCal _cal;
static uint16_t  _commanded = 0;

// ---------------------------------------------------------------------------
// Direction
// ---------------------------------------------------------------------------
// Counts may rise or fall as the clutch is pulled, depending on how the arm is
// mounted. Everything below asks these two instead of assuming a sign.

static bool _countsRise() { return _cal.disengagedCounts >= _cal.engagedCounts; }

// True once pos has reached thr on the way toward disengaged.
static bool _pastToward(uint16_t pos, uint16_t thr) {
    return _countsRise() ? (pos >= thr) : (pos <= thr);
}

static uint16_t _clampTravel(uint16_t counts) {
    uint16_t lo = _countsRise() ? _cal.engagedCounts : _cal.disengagedCounts;
    uint16_t hi = _countsRise() ? _cal.disengagedCounts : _cal.engagedCounts;
    if (counts < lo) return lo;
    if (counts > hi) return hi;
    return counts;
}

// ---------------------------------------------------------------------------
// CRC — same table-less IEEE 802.3 routine as the ESP32 node's CalConfig.h, so
// a blob means the same thing on both.
// ---------------------------------------------------------------------------
static uint32_t _crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
    }
    return ~crc;
}

static void _seal(ClutchCal *c) {
    c->version = CLUTCH_CAL_VERSION;
    c->crc = _crc32((const uint8_t *)c, sizeof(ClutchCal) - sizeof(c->crc));
}

static bool _sealed_ok(const ClutchCal *c) {
    if (c->version != CLUTCH_CAL_VERSION) return false;
    return _crc32((const uint8_t *)c, sizeof(ClutchCal) - sizeof(c->crc)) == c->crc;
}

// ---------------------------------------------------------------------------
// Validation — reject, never clamp
// ---------------------------------------------------------------------------
static const char *_validate(const ClutchCal *c) {
    if (c->engagedCounts    > asmg::kPositionMax) return "engaged out of range (0-32767)";
    if (c->disengagedCounts > asmg::kPositionMax) return "disengaged out of range (0-32767)";
    if (c->triggerCounts    > asmg::kPositionMax) return "trigger out of range (0-32767)";
    if (c->biteCounts       > asmg::kPositionMax) return "bite out of range (0-32767)";
    if (c->moveSpeed        > asmg::kSpeedSlowest) return "speed out of range (0-1280)";
    if (c->engagedCounts == c->disengagedCounts)
        return "engaged and disengaged must differ";

    uint16_t lo = c->disengagedCounts >= c->engagedCounts ? c->engagedCounts : c->disengagedCounts;
    uint16_t hi = c->disengagedCounts >= c->engagedCounts ? c->disengagedCounts : c->engagedCounts;
    if (c->triggerCounts < lo || c->triggerCounts > hi) return "trigger outside calibrated travel";
    if (c->biteCounts    < lo || c->biteCounts    > hi) return "bite outside calibrated travel";

    // The bite point is the engaged-side edge of the just-engaged window and
    // the trigger is its disengaged-side edge. Cross them over and the window
    // is empty, so clutch_just_engaged() would never fire and the fault would
    // look like a dead sensor rather than a bad number.
    bool rise = (c->disengagedCounts >= c->engagedCounts);
    bool biteIsEngagedSide = rise ? (c->biteCounts <= c->triggerCounts)
                                  : (c->biteCounts >= c->triggerCounts);
    if (!biteIsEngagedSide) return "bite must sit on the engaged side of trigger";

    // --- paddle response ---
    bool leftOff  = c->hallLeftIdle  == c->hallLeftPulled;
    bool rightOff = c->hallRightIdle == c->hallRightPulled;
    if (leftOff && rightOff) return "both paddles off (idle == pulled on each) — the clutch would never move";
    if (!leftOff  && abs((int32_t)c->hallLeftPulled  - (int32_t)c->hallLeftIdle)  <= 2 * HALL_IDLE_DEADBAND)
        return "left paddle travel too small (idle and pulled too close)";
    if (!rightOff && abs((int32_t)c->hallRightPulled - (int32_t)c->hallRightIdle) <= 2 * HALL_IDLE_DEADBAND)
        return "right paddle travel too small (idle and pulled too close)";
    if (c->curve >= PADDLE_CURVE_COUNT)                       return "curve: linear|log|exp|smooth|power|piecewise";
    if (c->curveStrength < 10 || c->curveStrength > 500)       return "strength out of range (10-500 = 0.1-5.0)";
    if (c->pwPaddleEnd > PADDLE_FULL)                          return "pw_paddle_end out of range (0-10000)";
    if (c->pwPaddleStart >= c->pwPaddleEnd)                    return "pw_paddle_start must be below pw_paddle_end";
    if (c->pwBlend > PADDLE_FULL / 2)                          return "pw_blend out of range (0-5000)";
    if (c->pwClutchEnd > PADDLE_FULL)                          return "pw_clutch_end out of range (0-10000)";
    if (c->pwClutchStart > c->pwClutchEnd)                     return "pw_clutch_start must not exceed pw_clutch_end";

    // --- shift timings ---
    // A pulse as long as the state machine's own timeout would be reported as
    // a failed shift every time, so the limit is that timeout, not a guess.
    const uint16_t pulses[] = { c->shiftUpMs, c->shiftDownMs, c->neutralUpMs, c->neutralDownMs };
    for (unsigned i = 0; i < sizeof(pulses) / sizeof(pulses[0]); i++)
        if (pulses[i] < 1 || pulses[i] >= GEARBOX_SHIFT_TIMEOUT_MS)
            return "shift times must be 1-499 ms (below the shift timeout)";
    if (c->ignCutMs >= GEARBOX_SHIFT_TIMEOUT_MS) return "ign_cut_ms out of range (0-499)";

    return NULL;
}

void clutch_cal_defaults() {
    memset(&_cal, 0, sizeof(_cal));
    _cal.version          = CLUTCH_CAL_VERSION;
    _cal.engagedCounts    = DEFAULT_ENGAGED;
    _cal.disengagedCounts = DEFAULT_DISENGAGED;
    // 60 % of travel for the downshift gate, 20 % for the bite point. On the
    // ESP32 both were captured live and neither had a meaningful default; these
    // exist so the predicates answer something sane before calibration.
    _cal.triggerCounts    = (uint16_t)(DEFAULT_ENGAGED + (DEFAULT_DISENGAGED - DEFAULT_ENGAGED) * 0.60f);
    _cal.biteCounts       = (uint16_t)(DEFAULT_ENGAGED + (DEFAULT_DISENGAGED - DEFAULT_ENGAGED) * 0.20f);
    _cal.moveSpeed        = DEFAULT_SPEED;

    // Paddle response — HallSensorControl.h's defaults, carried across. Hall
    // readings x16 for this ADC's 16 bits against the ESP32's 12; the
    // piecewise zone re-expressed as fractions of paddle travel and of clutch
    // travel (the ESP32 had it in hall counts and servo degrees: hall
    // 1000..3000 of 780..3330, servo 98..115 deg of 42..137).
    _cal.hallLeftIdle     = 780  * 16;
    _cal.hallLeftPulled   = 3330 * 16;
    _cal.hallRightIdle    = 1000 * 16;
    _cal.hallRightPulled  = 3600 * 16;
    _cal.curve            = PADDLE_LOG;
    _cal.curveStrength    = 200;            // 2.0
    _cal.pwPaddleStart    = 863;            // (1000-780)/(3330-780)
    _cal.pwPaddleEnd      = 8706;           // (3000-780)/(3330-780)
    _cal.pwBlend          = 471;            // 120/(3330-780)
    _cal.pwClutchStart    = 5895;           // (98-42)/(137-42)
    _cal.pwClutchEnd      = 7684;           // (115-42)/(137-42)

    // Shift timings — the ESP32 node's defaults.
    _cal.shiftUpMs        = 150;
    _cal.shiftDownMs      = 150;
    _cal.neutralUpMs      = 40;
    _cal.neutralDownMs    = 40;
    _cal.ignCutMs         = IGN_CUT_DEFAULT_MS;
    _seal(&_cal);
}

bool clutch_cal_load() {
    ClutchCal tmp;
    if (!cal_store_read(&tmp, sizeof(tmp))) return false;
    if (!_sealed_ok(&tmp)) return false;
    if (_validate(&tmp) != NULL) return false;
    _cal = tmp;
    return true;
}

bool clutch_cal_save() {
    ClutchCal tmp = _cal;
    if (_validate(&tmp) != NULL) return false;
    _seal(&tmp);
    return cal_store_write(&tmp, sizeof(tmp));
}

bool clutch_init() {
    clutch_cal_defaults();
    bool fromFlash = clutch_cal_load();
    _commanded = _cal.engagedCounts;
    return fromFlash;
}

// ---------------------------------------------------------------------------
// Live state
// ---------------------------------------------------------------------------

uint16_t clutch_position()    { return servo_position(); }
bool     clutch_feedback_ok() { return servo_link_ok(); }

float clutch_pull_fraction() {
    int32_t span = (int32_t)_cal.disengagedCounts - (int32_t)_cal.engagedCounts;
    if (span == 0) return 0.0f;
    float f = ((int32_t)clutch_position() - (int32_t)_cal.engagedCounts) / (float)span;
    if (f < 0.0f) return 0.0f;
    if (f > 1.0f) return 1.0f;
    return f;
}

bool clutch_disengaged() {
    if (!clutch_feedback_ok()) return false;
    return _pastToward(clutch_position(), _cal.triggerCounts);
}

bool clutch_just_engaged() {
    if (!clutch_feedback_ok()) return false;
    uint16_t pos = clutch_position();
    return !_pastToward(pos, _cal.triggerCounts) && _pastToward(pos, _cal.biteCounts);
}

bool clutch_at_bite() {
    if (!clutch_feedback_ok()) return false;
    int32_t span = (int32_t)_cal.disengagedCounts - (int32_t)_cal.engagedCounts;
    int32_t tol  = (span < 0 ? -span : span) / 100 + 1;
    // Distance from the bite point toward disengaged; <= 0 is the engaged side.
    int32_t d = (int32_t)clutch_position() - (int32_t)_cal.biteCounts;
    if (!_countsRise()) d = -d;
    return d <= tol;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

void clutch_command_counts(uint16_t counts) {
    _commanded = _clampTravel(counts);
    servo_command_position(_commanded, _cal.moveSpeed);
}

void clutch_command_pull(float fraction) {
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    int32_t span = (int32_t)_cal.disengagedCounts - (int32_t)_cal.engagedCounts;
    clutch_command_counts((uint16_t)((int32_t)_cal.engagedCounts + (int32_t)(span * fraction)));
}

void clutch_release() { clutch_command_counts(_cal.engagedCounts); }
void clutch_command_bite() { clutch_command_counts(_cal.biteCounts); }

uint16_t clutch_commanded()   { return _commanded; }
const ClutchCal *clutch_cal() { return &_cal; }

// ---------------------------------------------------------------------------
// Calibration edits
// ---------------------------------------------------------------------------

static const struct { const char *name; size_t off; } _fields[] = {
    { "engaged",    offsetof(ClutchCal, engagedCounts)    },
    { "disengaged", offsetof(ClutchCal, disengagedCounts) },
    { "trigger",    offsetof(ClutchCal, triggerCounts)    },
    { "bite",       offsetof(ClutchCal, biteCounts)       },
    { "speed",      offsetof(ClutchCal, moveSpeed)        },
    { "paddle_l_idle",   offsetof(ClutchCal, hallLeftIdle)    },
    { "paddle_l_pulled", offsetof(ClutchCal, hallLeftPulled)  },
    { "paddle_r_idle",   offsetof(ClutchCal, hallRightIdle)   },
    { "paddle_r_pulled", offsetof(ClutchCal, hallRightPulled) },
    { "curve",           offsetof(ClutchCal, curve)           },
    { "strength",        offsetof(ClutchCal, curveStrength)   },
    { "pw_paddle_start", offsetof(ClutchCal, pwPaddleStart)   },
    { "pw_paddle_end",   offsetof(ClutchCal, pwPaddleEnd)     },
    { "pw_clutch_start", offsetof(ClutchCal, pwClutchStart)   },
    { "pw_clutch_end",   offsetof(ClutchCal, pwClutchEnd)     },
    { "pw_blend",        offsetof(ClutchCal, pwBlend)         },
    { "shift_up_ms",     offsetof(ClutchCal, shiftUpMs)       },
    { "shift_down_ms",   offsetof(ClutchCal, shiftDownMs)     },
    { "neutral_up_ms",   offsetof(ClutchCal, neutralUpMs)     },
    { "neutral_down_ms", offsetof(ClutchCal, neutralDownMs)   },
    { "ign_cut_ms",      offsetof(ClutchCal, ignCutMs)        },
};

const char *clutch_cal_field_names() {
    return "engaged disengaged trigger bite speed paddle_l_idle paddle_l_pulled "
           "paddle_r_idle paddle_r_pulled curve strength pw_paddle_start pw_paddle_end "
           "pw_clutch_start pw_clutch_end pw_blend shift_up_ms shift_down_ms "
           "neutral_up_ms neutral_down_ms ign_cut_ms";
}

static bool _apply(const char *field, long value, const char **err) {
    if (value < 0 || value > 0xFFFF) { *err = "value out of range"; return false; }

    for (unsigned i = 0; i < sizeof(_fields) / sizeof(_fields[0]); i++) {
        if (strcmp(field, _fields[i].name) != 0) continue;

        // Edit a copy, validate the WHOLE result, then promote — a single
        // field can only be judged against the others it has to agree with.
        ClutchCal tmp = _cal;
        *(uint16_t *)((uint8_t *)&tmp + _fields[i].off) = (uint16_t)value;
        const char *bad = _validate(&tmp);
        if (bad) { *err = bad; return false; }
        _seal(&tmp);
        _cal = tmp;
        return true;
    }

    *err = "unknown field — see `cal` for the list";
    return false;
}

bool clutch_cal_get(const char *field, long *out) {
    for (unsigned i = 0; i < sizeof(_fields) / sizeof(_fields[0]); i++) {
        if (strcmp(field, _fields[i].name) != 0) continue;
        *out = *(const uint16_t *)((const uint8_t *)&_cal + _fields[i].off);
        return true;
    }
    return false;
}

bool clutch_cal_set(const char *field, long value, const char **err) {
    return _apply(field, value, err);
}

bool clutch_cal_capture(const char *field, const char **err) {
    // Paddles: the raw hall reading, held where it matters.
    if (!strcmp(field, "paddle_l_idle") || !strcmp(field, "paddle_l_pulled"))
        return _apply(field, (long)hall_read_left(), err);
    if (!strcmp(field, "paddle_r_idle") || !strcmp(field, "paddle_r_pulled"))
        return _apply(field, (long)hall_read_right(), err);

    // The piecewise paddle zone: where the paddle is now, in paddle units.
    if (!strcmp(field, "pw_paddle_start") || !strcmp(field, "pw_paddle_end"))
        return _apply(field, (long)paddle_from_halls(&_cal, hall_read_left(), hall_read_right(), NULL), err);

    if (!strcmp(field, "speed") || !strcmp(field, "curve") ||
        !strcmp(field, "strength") || !strcmp(field, "pw_blend") ||
        strstr(field, "_ms")) {
        *err = "not a position — use cal set";
        return false;
    }

    // Everything else is a clutch position.
    if (!clutch_feedback_ok()) { *err = "no fresh servo feedback — nothing to capture"; return false; }

    // The piecewise clutch zone is a fraction of travel, not a raw count.
    if (!strcmp(field, "pw_clutch_start") || !strcmp(field, "pw_clutch_end"))
        return _apply(field, (long)lroundf(clutch_pull_fraction() * PADDLE_FULL), err);

    return _apply(field, (long)clutch_position(), err);
}
