#pragma once
#include <stdint.h>
#include <stdbool.h>

// Paddle and switch inputs, captured by EXTI interrupt.
//
// This is the reason the node is moving to STM32 at all. On the ESP32 these
// were polled edge-detects in loop(), so a press that began and ended inside a
// loop stall was not delayed — it was never seen. Here the edge raises an
// interrupt, is debounced and timestamped in the handler, and queues an event.
// The main loop can be as busy as it likes; the press is already recorded.

typedef enum {
    INPUT_SHIFT_UP = 0,
    INPUT_SHIFT_DOWN,
    INPUT_NEUTRAL,
    INPUT_MANUAL_MODE,
    INPUT_LOG_MARK,
    INPUT_ARM_SAFE,
    INPUT_COUNT
} InputId;

typedef struct {
    InputId  id;
    bool     pressed;    // true = went active (LOW), false = released
    uint32_t at_ms;      // millis() captured in the handler, not at drain time
} InputEvent;

void inputs_init();

// Pop the oldest event. Returns false when the queue is empty.
// Call until it returns false; never blocks.
bool inputs_pop(InputEvent *out);

// Live level, for the switches where state matters more than the edge.
bool inputs_level(InputId id);

// Diagnostics: events dropped because the queue was full. Should stay 0 —
// anything else means the loop is not draining often enough.
uint32_t inputs_dropped();
