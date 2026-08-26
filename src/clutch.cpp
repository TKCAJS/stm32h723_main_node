#include <Arduino.h>
#include <string.h>
#include <stddef.h>
#include "clutch.h"
#include "cal_store.h"
#include "servo_can.h"
#include "asmg_md_can.h"

#define CLUTCH_CAL_VERSION  1

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
    _cal.reserved         = 0;
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
};

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

    *err = "unknown field (engaged|disengaged|trigger|bite|speed)";
    return false;
}

bool clutch_cal_set(const char *field, long value, const char **err) {
    return _apply(field, value, err);
}

bool clutch_cal_capture(const char *field, const char **err) {
    if (strcmp(field, "speed") == 0) { *err = "speed is not a position — use cal set speed"; return false; }
    if (!clutch_feedback_ok()) { *err = "no fresh servo feedback — nothing to capture"; return false; }
    return _apply(field, (long)clutch_position(), err);
}
