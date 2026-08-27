# GPIO map

Assigned pins, from `include/pins.h` — that file is the source of truth; this is a
quick-reference copy of it. If the two disagree, `pins.h` wins.

| Pin | Function |
|---|---|
| PD0 | FDCAN1_RX (main bus) |
| PD1 | FDCAN1_TX (main bus) |
| PB12 | FDCAN2_RX (servo bus) |
| PB13 | FDCAN2_TX (servo bus) |
| PA2 | HALL_LEFT (ADC12_INP14) |
| PA3 | HALL_RIGHT (ADC12_INP15) |
| PA0 | RPM_IN (TIM2_CH1/ETR) |
| PA8 | MATRIX_DATA (NeoMatrix, TIM1_CH1) |
| PB4 | SHIFT_UP (EXTI4) |
| PB5 | SHIFT_DOWN (EXTI5) |
| PB6 | NEUTRAL (EXTI6) |
| PB7 | MANUAL_MODE (EXTI7) |
| PB8 | LOG_MARK (EXTI8) |
| PB9 | ARM_SAFE (EXTI9) |
| PE0 | ENC_CLK (EXTI0) |
| PE1 | ENC_DT (EXTI1) |
| PE2 | ENC_SW (EXTI2) |
| PC13 | USER_BTN (onboard WeAct button) |
| PE10 | LCD_BLK (backlight, active low) |
| PE11 | LCD_CS |
| PE12 | LCD_SCK |
| PE13 | LCD_DC |
| PE14 | LCD_MOSI (panel RST tied to NRST) |
| PC8 | SD_D0 |
| PC9 | SD_D1 |
| PC10 | SD_D2 |
| PC11 | SD_D3 |
| PC12 | SD_CK |
| PD2 | SD_CMD |
| PA11 | USB_DM |
| PA12 | USB_DP |

Left unassigned on purpose: PC0/PC1 (freed ADC1 differential pair, kept spare).
