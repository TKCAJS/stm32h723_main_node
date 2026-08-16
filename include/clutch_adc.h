#pragma once
#include <stdint.h>
#include <stdbool.h>

// Analog inputs — the ADS1115 replacement.
//
// ADC1 owns the clutch servo feedback as a DIFFERENTIAL channel (PC0/PC1),
// free-running with 64x hardware oversampling. Reading it is a register access:
// no I2C, no bus to wedge, no call that can block. That is the entire point —
// the ADS was fine electrically, but its bus could stall the CPU.
//
// ADC2 owns the two paddle halls, single-ended.
//
// Do not call Arduino analogRead() anywhere in this project; it would grab ADC1
// and reconfigure it out from under the differential channel.

bool clutch_adc_init();

// Latest free-running differential result. Never blocks.
// Signed: differential zero sits at mid-scale, so this is (INP - INN).
int32_t clutch_adc_raw();

// Same reading in volts at the ADC pin (post-divider — thresholds are captured
// in these same volts, so absolute scale never matters).
float clutch_adc_volts();

// Paddle halls, 0..65535. A conversion is a few microseconds, bounded and
// with no bus involved.
uint16_t hall_read_left();
uint16_t hall_read_right();
