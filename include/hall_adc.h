#pragma once
#include <stdint.h>
#include <stdbool.h>

// Paddle hall sensors — the node's only analog inputs.
//
// This module used to be clutch_adc.h and owned two ADCs: ADC1 held the clutch
// servo's position feedback as a differential channel (the ADS1115 replacement)
// and ADC2 held the halls. The clutch servo now reports its own position in
// counts over its CAN bus (servo_can.h), so the entire analog feedback path —
// divider, ground-return conductor, differential channel — is gone. What was
// won by reading differentially against the servo's own ground is won more
// completely by not measuring an analog quantity at all.
//
// ADC2 keeps the halls, single-ended, one conversion at a time on demand. A
// conversion is a few microseconds inside the chip: bounded, and with no bus
// another device can stall.
//
// Do not call Arduino analogRead() anywhere in this project. It resolves most
// pins to ADC1 and reconfigures whatever it finds there.

bool hall_adc_init();

// Paddle halls, 0..65535. Never blocks.
uint16_t hall_read_left();
uint16_t hall_read_right();
