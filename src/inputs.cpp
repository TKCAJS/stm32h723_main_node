#include <Arduino.h>
#include "inputs.h"
#include "pins.h"

// Debounce is done in the handler by ignoring further edges on the same input
// for a short window. A mechanical paddle bounces for a few ms; a real press is
// tens of ms. The first edge is always taken and timestamped, so debouncing
// costs no latency — unlike a settle-then-read scheme.
#define DEBOUNCE_MS   8

#define QUEUE_LEN     16      // power of two

static const uint32_t _pin[INPUT_COUNT] = {
    PIN_SHIFT_UP, PIN_SHIFT_DOWN, PIN_NEUTRAL,
    PIN_MANUAL_MODE, PIN_LOG_MARK, PIN_ARM_SAFE
};

static volatile InputEvent _q[QUEUE_LEN];
static volatile uint8_t    _head = 0;      // written by handlers only
static volatile uint8_t    _tail = 0;      // written by inputs_pop() only
static volatile uint32_t   _dropped = 0;
static volatile uint32_t   _lastEdgeMs[INPUT_COUNT] = {0};

// Single producer (interrupt) / single consumer (loop), so head and tail each
// have exactly one writer and no lock is needed.
static void _push(InputId id) {
    uint32_t now = millis();
    if (now - _lastEdgeMs[id] < DEBOUNCE_MS) return;
    _lastEdgeMs[id] = now;

    uint8_t next = (uint8_t)((_head + 1) & (QUEUE_LEN - 1));
    if (next == _tail) {          // full — drop, and say so
        _dropped++;
        return;
    }

    _q[_head].id      = id;
    _q[_head].pressed = (digitalRead(_pin[id]) == LOW);
    _q[_head].at_ms   = now;
    _head = next;
}

static void _isr_shift_up()    { _push(INPUT_SHIFT_UP); }
static void _isr_shift_down()  { _push(INPUT_SHIFT_DOWN); }
static void _isr_neutral()     { _push(INPUT_NEUTRAL); }
static void _isr_manual()      { _push(INPUT_MANUAL_MODE); }
static void _isr_log_mark()    { _push(INPUT_LOG_MARK); }
static void _isr_arm_safe()    { _push(INPUT_ARM_SAFE); }

void inputs_init() {
    for (int i = 0; i < INPUT_COUNT; i++) pinMode(_pin[i], INPUT_PULLUP);

    // CHANGE, not FALLING: releases matter too (hold-to-stack, arm/safe state).
    attachInterrupt(digitalPinToInterrupt(PIN_SHIFT_UP),    _isr_shift_up,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SHIFT_DOWN),  _isr_shift_down, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_NEUTRAL),     _isr_neutral,    CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_MANUAL_MODE), _isr_manual,     CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LOG_MARK),    _isr_log_mark,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ARM_SAFE),    _isr_arm_safe,   CHANGE);
}

bool inputs_pop(InputEvent *out) {
    if (_tail == _head) return false;
    // C++ has no copy assignment for volatile structs — copy field by field.
    out->id      = _q[_tail].id;
    out->pressed = _q[_tail].pressed;
    out->at_ms   = _q[_tail].at_ms;
    _tail = (uint8_t)((_tail + 1) & (QUEUE_LEN - 1));
    return true;
}

bool inputs_level(InputId id) {
    return digitalRead(_pin[id]) == LOW;
}

uint32_t inputs_dropped() {
    return _dropped;
}
