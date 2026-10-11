#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "paddle.h"

static bool     _override = false;
static uint16_t _position = 0;
static uint16_t _target   = 0;
static uint32_t _faults   = 0;

// ---------------------------------------------------------------------------
// Hall -> paddle position
// ---------------------------------------------------------------------------

// One paddle, normalised on its own calibration. Returns -1 for a reading
// rejected as out of range, so the caller can tell "idle" from "broken".
static int32_t _normalise(uint16_t raw, uint16_t idle, uint16_t pulled) {
    int32_t span = (int32_t)pulled - (int32_t)idle;
    if (span == 0) return 0;                        // paddle switched off

    // Out of range by more than 10 % of travel either side: not a paddle
    // position, a sensor fault. Treated as idle, never as pulled.
    int32_t aspan  = span < 0 ? -span : span;
    int32_t margin = aspan / 10;
    int32_t lo = (span > 0 ? idle : pulled) - margin;
    int32_t hi = (span > 0 ? pulled : idle) + margin;
    if ((int32_t)raw < lo || (int32_t)raw > hi) return -1;

    int32_t db  = span > 0 ? HALL_IDLE_DEADBAND : -HALL_IDLE_DEADBAND;
    int32_t num = (int32_t)raw - ((int32_t)idle + db);
    int32_t den = span - db;                        // validation keeps |span| > 2*deadband
    int32_t p   = (int32_t)((int64_t)num * PADDLE_FULL / den);
    if (p < 0) p = 0;
    if (p > PADDLE_FULL) p = PADDLE_FULL;
    return p;
}

uint16_t paddle_from_halls(const ClutchCal *c, uint16_t left, uint16_t right, bool *fault) {
    int32_t l = _normalise(left,  c->hallLeftIdle,  c->hallLeftPulled);
    int32_t r = _normalise(right, c->hallRightIdle, c->hallRightPulled);
    if (fault) *fault = (l < 0 || r < 0);
    if (l < 0) l = 0;
    if (r < 0) r = 0;
    return (uint16_t)(l > r ? l : r);               // the further-pulled paddle wins
}

// ---------------------------------------------------------------------------
// Curves — HallSensorControl::applyHallCurve(), unchanged
// ---------------------------------------------------------------------------

static float _curve(float x, uint16_t type, float s) {
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    switch (type) {
        case PADDLE_LOG:
            if (s <= 0.0f) s = 0.1f;
            return logf(1.0f + x * (expf(s) - 1.0f)) / s;
        case PADDLE_EXP:
            if (s <= 0.0f) s = 0.1f;
            return (expf(x * s) - 1.0f) / (expf(s) - 1.0f);
        case PADDLE_SMOOTH: {
            float sm = x * x * (3.0f - 2.0f * x);
            return x + (sm - x) * (s / 3.0f);
        }
        case PADDLE_POWER:
            if (s <= 0.0f) s = 1.0f;
            return powf(x, s);
        case PADDLE_LINEAR:
        default:
            return x;
    }
}

static float _mapf(float x, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) return outMin;
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// HallSensorControl::hallToPiecewise(), in paddle units in and counts out.
// Quadratic-Bezier corners: control point on the breakpoint, endpoints w out
// along each segment — C1-continuous and monotonic, so it never reverses the
// way a two-line weighted blend can. Works for counts running either way.
static float _piecewise(const ClutchCal *c, int32_t x) {
    const int32_t xMin = 0, xMax = PADDLE_FULL;
    const int32_t xS = c->pwPaddleStart, xE = c->pwPaddleEnd;
    const float   y0   = c->engagedCounts, y1 = c->disengagedCounts;
    const float   yS   = y0 + (y1 - y0) * c->pwClutchStart / (float)PADDLE_FULL;
    const float   yE   = y0 + (y1 - y0) * c->pwClutchEnd   / (float)PADDLE_FULL;
    const int32_t blend = c->pwBlend;

    if (blend > 0) {
        int32_t midLen = xE - xS;
        // Clamp each corner so the two cannot cross each other, the far
        // breakpoint, or run past either end of travel.
        int32_t wS = min(blend, min(xS - xMin, midLen / 2));
        int32_t wE = min(blend, min(xMax - xE, midLen / 2));

        if (wS > 0 && x > xS - wS && x < xS + wS) {
            float mLo  = (yS - y0) / (float)(xS - xMin);
            float mMid = (yE - yS) / (float)(xE - xS);
            float tau  = (float)(x - (xS - wS)) / (2.0f * wS);
            float omt  = 1.0f - tau;
            return yS + wS * (mMid * tau * tau - mLo * omt * omt);
        }
        if (wE > 0 && x > xE - wE && x < xE + wE) {
            float mMid = (yE - yS) / (float)(xE - xS);
            float mHi  = (y1 - yE) / (float)(xMax - xE);
            float tau  = (float)(x - (xE - wE)) / (2.0f * wE);
            float omt  = 1.0f - tau;
            return yE + wE * (mHi * tau * tau - mMid * omt * omt);
        }
    }

    if (x <= xS) return _mapf(x, xMin, xS, y0, yS);
    if (x <= xE) return _mapf(x, xS, xE, yS, yE);
    return _mapf(x, xE, xMax, yE, y1);
}

static uint16_t _clampTravel(const ClutchCal *c, float counts) {
    float lo = c->engagedCounts < c->disengagedCounts ? c->engagedCounts : c->disengagedCounts;
    float hi = c->engagedCounts < c->disengagedCounts ? c->disengagedCounts : c->engagedCounts;
    if (counts < lo) counts = lo;
    if (counts > hi) counts = hi;
    return (uint16_t)lroundf(counts);
}

uint16_t paddle_to_counts(const ClutchCal *c, uint16_t paddle) {
    if (paddle > PADDLE_FULL) paddle = PADDLE_FULL;
    if (c->curve == PADDLE_PIECEWISE) return _clampTravel(c, _piecewise(c, paddle));

    float x = (float)paddle / PADDLE_FULL;
    float f = _curve(x, c->curve, c->curveStrength / 100.0f);
    return _clampTravel(c, c->engagedCounts + f * ((float)c->disengagedCounts - c->engagedCounts));
}

// ---------------------------------------------------------------------------
// Live path
// ---------------------------------------------------------------------------

void paddle_tick(uint16_t left, uint16_t right, PaddleMode mode) {
    const ClutchCal *c = clutch_cal();

    bool fault = false;
    _position = paddle_from_halls(c, left, right, &fault);
    if (fault) _faults++;

    if (mode == PADDLE_DIRECT) {
        _target = _clampTravel(c, c->engagedCounts +
                  (float)_position / PADDLE_FULL * ((float)c->disengagedCounts - c->engagedCounts));
    } else {
        _target = paddle_to_counts(c, _position);
    }

    if (mode == PADDLE_OFF || _override) return;

    // Compared against what the clutch was last told, not against our own
    // last target: the state machine releases the clutch on its way back to
    // idle, and the paddle must win it back even if it has not moved.
    if (clutch_commanded() != _target) clutch_command_counts(_target);
}

void paddle_set_override(bool on) { _override = on; }
bool paddle_override()            { return _override; }
uint16_t paddle_position()        { return _position; }
uint16_t paddle_target()          { return _target; }
uint32_t paddle_fault_count()     { return _faults; }

static const char *const _names[PADDLE_CURVE_COUNT] = {
    "linear", "log", "exp", "smooth", "power", "piecewise",
};

const char *paddle_curve_name(uint16_t curve) {
    return curve < PADDLE_CURVE_COUNT ? _names[curve] : "?";
}

int paddle_curve_parse(const char *name) {
    for (int i = 0; i < PADDLE_CURVE_COUNT; i++) if (!strcmp(name, _names[i])) return i;
    if (!strcmp(name, "logarithmic")) return PADDLE_LOG;
    if (!strcmp(name, "exponential")) return PADDLE_EXP;
    if (!strcmp(name, "custom"))      return PADDLE_POWER;   // the ESP32's name for it
    return -1;
}
