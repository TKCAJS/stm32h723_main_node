#pragma once

// ============================================================================
// T89 main node pin map — STM32H723ZGT6 (WeAct board)
// ============================================================================
// Nothing is built yet, so this is the document to argue with before any wire
// is cut. Two hard constraints shaped it:
//
//  1. EXTI lines are per PIN NUMBER, not per port. PB4 and PA4 cannot both be
//     interrupt inputs — they share EXTI4. Every interrupt-driven input below
//     therefore has a distinct pin number (4,5,6,7,8,9 on port B; 0,1,2 on E).
//
//  2. Never call Arduino analogRead() in this project. It resolves a pin to the
//     first ADC in the pinmap (ADC1 for most pins) and reconfigures it out from
//     under whatever else was using it. Analog reads go through hall_adc.h,
//     which owns ADC2 explicitly. The paddle halls are the ONLY analog inputs
//     left — clutch position now arrives over the servo CAN bus below.
// ============================================================================


// ---------------------------------------------------------------------------
// MAIN CAN bus — FDCAN1, 500 kbit/s classic, 29-bit extended IDs
// ---------------------------------------------------------------------------
// Same pins as stm32h723_transmitter_node, so the two nodes stay
// interchangeable on the bench and the bring-up code ports verbatim.
#define PIN_CAN_RX          PD0     // FDCAN1_RX, AF9
#define PIN_CAN_TX          PD1     // FDCAN1_TX, AF9


// ---------------------------------------------------------------------------
// SERVO CAN bus — FDCAN2, 250 kbit/s classic, 29-bit extended IDs
// ---------------------------------------------------------------------------
// The clutch servo (Wingxine ASMG-MD, native-CAN "ZDY" variant) is commanded
// and read over its own bus. It gets a SECOND controller, not a second address
// on the main bus, for three reasons:
//
//   1. Rate. Clutch telemetry is polled request/reply at 50 Hz — every poll is
//      two frames. On the main bus that traffic would sit underneath the shift
//      commands and gear reports, for no benefit to any other node.
//   2. Bit rate. The servo ships at 250 kbit/s; the bike bus is 500 kbit/s and
//      is not being re-rated to suit one actuator.
//   3. Blast radius. The servo's arbitration IDs are the manufacturer's
//      (0x18EF0201, J1939-shaped) and sit outside our can_ids.h scheme
//      entirely. A confused servo cannot talk over a gear report it can't see.
//
// FDCAN2_RX/TX are only offered on PB5/PB6 or PB12/PB13. PB5 and PB6 are
// paddle EXTI inputs, so PB12/PB13 it is — neither is an interrupt input, so
// no EXTI line is consumed.
#define PIN_SERVO_CAN_RX    PB12    // FDCAN2_RX, AF9
#define PIN_SERVO_CAN_TX    PB13    // FDCAN2_TX, AF9

// ---------------------------------------------------------------------------
// Freed by the servo CAN migration — do not silently reuse
// ---------------------------------------------------------------------------
// PC0 / PC1 carried the ADS1115-replacement differential feedback pair, and PA6
// carried the servo's PWM. Both paths are gone: the servo reports its own
// position in counts over CAN and is commanded the same way, so there is no
// analog feedback to divide, no ground return to run back, and no pulse to
// keep exact. The divider, the feedback conductor and the PWM line all come out
// of the loom.
//
// Left unassigned deliberately. PC0/PC1 are the only ADC1 differential pair
// brought out on this board, so they are worth keeping free for a future
// differential sensor rather than spending on a spare GPIO.

// ---------------------------------------------------------------------------
// Clutch paddle hall sensors — ADC2, single-ended
// ---------------------------------------------------------------------------
// Two paddles, scaled and blended in firmware exactly as HallSensorControl.h
// does today. The only analog inputs left on the node. Kept on ADC2 rather
// than ADC1 so that PC0/PC1's differential pair stays available untouched.
#define PIN_HALL_LEFT       PA2     // ADC12_INP14
#define PIN_HALL_RIGHT      PA3     // ADC12_INP15
#define ADC_CH_HALL_LEFT    ADC_CHANNEL_14
#define ADC_CH_HALL_RIGHT   ADC_CHANNEL_15


// ---------------------------------------------------------------------------
// RPM — MAX9926 clean digital pulse into a hardware counter
// ---------------------------------------------------------------------------
// TIM2 in external-clock mode is the direct equivalent of the ESP32's PCNT:
// the counter advances in hardware, the CPU only reads it. TIM2 is 32-bit, so
// unlike PCNT's int16 there is no wrap to manage. The timer's input filter
// (ICxF) replaces pcnt_set_filter_value(800) (~10 us).
//
// Keeping the MAX9926 for now — its adaptive peak threshold tracks a VR
// amplitude that varies ~1000:1 across the rev range, which an internal
// comparator on a fixed DAC threshold does badly.
#define PIN_RPM_IN          PA0     // TIM2_CH1/ETR, AF1

// ---------------------------------------------------------------------------
// 16x16 NeoMatrix (4x 8x8 WS2812 panels)
// ---------------------------------------------------------------------------
// TIM1_CH1 + DMA. 256 LEDs is ~7.7 ms of bitstream that costs the CPU nothing,
// versus the ESP32's blocking show(). TIM1 is on APB2 with DMA burst support.
#define PIN_MATRIX_DATA     PA8     // TIM1_CH1, AF1


// ---------------------------------------------------------------------------
// Inputs — all EXTI, all distinct pin numbers (see note 1 at top)
// ---------------------------------------------------------------------------
// The whole point of the migration: a paddle press raises an interrupt and is
// timestamped immediately. It cannot be missed by a busy main loop, which is
// exactly how presses were being lost on the ESP32's polled edge detect.
// All active LOW with pull-ups.
#define PIN_SHIFT_UP        PB4     // EXTI4
#define PIN_SHIFT_DOWN      PB5     // EXTI5
#define PIN_NEUTRAL         PB6     // EXTI6
#define PIN_MANUAL_MODE     PB7     // EXTI7  — real toggle switch, not a long-press
#define PIN_LOG_MARK        PB8     // EXTI8  — stamp a marker into the SD log
#define PIN_ARM_SAFE        PB9     // EXTI9  — inhibit all shift output (pit safety)

// Diagnostic encoder (KY-040). Driver lifts straight from
// stm32h723_transmitter_node's encoder.cpp — only these three defines change.
#define PIN_ENC_CLK         PE0     // EXTI0
#define PIN_ENC_DT          PE1     // EXTI1
#define PIN_ENC_SW          PE2     // EXTI2

#define PIN_USER_BTN        PC13    // WeAct onboard button, active LOW


// ---------------------------------------------------------------------------
// Onboard 0.96" ST7735 status LCD (160x80) — fixed by the WeAct board
// ---------------------------------------------------------------------------
// Software SPI on its own pins, as in the transmitter node: at a few Hz of text
// the bit-bang cost is negligible and it can never contend with another bus.
// Diagnostics only — the rider reads the NeoMatrix, not this.
#define PIN_LCD_BLK         PE10    // active low (P-FET switch)
#define PIN_LCD_CS          PE11
#define PIN_LCD_SCK         PE12
#define PIN_LCD_DC          PE13
#define PIN_LCD_MOSI        PE14    // panel RST tied to NRST


// ---------------------------------------------------------------------------
// SD logging — SDMMC1, 4-bit
// ---------------------------------------------------------------------------
// Fixed peripheral pins. Rule for later: writes go to a RAM ring and flush when
// idle. Never touch the card inside a shift.
#define PIN_SD_D0           PC8
#define PIN_SD_D1           PC9
#define PIN_SD_D2           PC10
#define PIN_SD_D3           PC11
#define PIN_SD_CK           PC12
#define PIN_SD_CMD          PD2

// ---------------------------------------------------------------------------
// USB CDC — calibration link (see README; the console also runs on a UART)
// ---------------------------------------------------------------------------
#define PIN_USB_DM          PA11
#define PIN_USB_DP          PA12
