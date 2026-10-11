#include <Arduino.h>
#include "shift_log.h"
#include "sd_log.h"

static bool        _active = false;
static ShiftRecord _cur;
static uint16_t    _rpm = 0;

static uint32_t _total = 0, _failed = 0;
static uint32_t _okTimeSum = 0;

static ShiftRecord _recent[SHIFT_LOG_RECENT];
static uint32_t    _seq = 0;

const char *shift_log_kind_name(uint8_t kind) {
    switch (kind) {
        case SHIFT_KIND_UP:      return "up";
        case SHIFT_KIND_DOWN:    return "down";
        case SHIFT_KIND_NEUTRAL: return "neutral";
        default:                 return "?";
    }
}

static void _finish(bool ok, uint32_t timeMs) {
    _active     = false;
    _cur.ok     = ok;
    _cur.timeMs = (uint16_t)(timeMs > 0xFFFF ? 0xFFFF : timeMs);

    _total++;
    if (ok) _okTimeSum += _cur.timeMs;
    else    _failed++;

    _recent[_seq++ & (SHIFT_LOG_RECENT - 1)] = _cur;
    sd_log_line("SHIFT,%u,%u,%u,%u,%s,%u", _cur.from, _cur.to, _cur.rpm, _cur.timeMs,
                shift_log_kind_name(_cur.kind), ok ? 1 : 0);
}

void shift_log_start(uint8_t from, uint8_t to, uint8_t kind) {
    // A shift still being timed when the next one starts never got its gear.
    if (_active) _finish(false, millis() - _cur.ms);

    _active   = true;
    _cur.ms   = millis();
    _cur.from = from;
    _cur.to   = to;
    _cur.kind = kind;
    _cur.rpm  = _rpm;
}

void shift_log_gear_changed(uint8_t newGear, uint8_t oldGear) {
    if (!_active) return;
    if (oldGear == _cur.from && newGear == _cur.to) _finish(true, millis() - _cur.ms);
}

void shift_log_tick(uint16_t rpm) {
    _rpm = rpm;
    if (_active && (millis() - _cur.ms) >= SHIFT_LOG_TIMEOUT_MS) _finish(false, SHIFT_LOG_TIMEOUT_MS);
}

uint32_t shift_log_total()  { return _total; }
uint32_t shift_log_failed() { return _failed; }

uint16_t shift_log_average_ms() {
    uint32_t ok = _total - _failed;
    return ok ? (uint16_t)(_okTimeSum / ok) : 0;
}

uint8_t shift_log_recent(ShiftRecord *out, uint8_t max) {
    uint32_t n = _seq < SHIFT_LOG_RECENT ? _seq : SHIFT_LOG_RECENT;
    if (n > max) n = max;
    uint32_t first = _seq - n;
    for (uint32_t i = 0; i < n; i++) out[i] = _recent[(first + i) & (SHIFT_LOG_RECENT - 1)];
    return (uint8_t)n;
}
