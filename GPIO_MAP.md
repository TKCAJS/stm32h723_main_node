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
| PA0 | RPM_IN (TIM2_CH1/ETR) — also the variant's default UART4 TX, see note |
| PA8 | MATRIX_DATA (NeoMatrix, TIM1_CH1) — shared with USB SOF, see note |
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
| PA9 | **reserved** — USB_OTG_HS_VBUS, claimed by the USB stack |
| PA10 | **reserved** — USB_OTG_HS_ID, claimed by the USB stack |
| PA11 | USB_DM (OTG_HS internal FS PHY) |
| PA12 | USB_DP (OTG_HS internal FS PHY) |

Left unassigned on purpose: PC0/PC1 (freed ADC1 differential pair, kept spare) and
PA6 (freed when the clutch servo stopped being PWM).

## Two pins that are not as free as they look

The USB device stack configures **every** pin in the OTG_HS pin map, not just D-/D+.
With USB CDC enabled that is PA8, PA9, PA10, PA11 and PA12.

- **PA9 / PA10** carry VBUS and ID. Nothing else may use them, which is why they are
  listed above despite no `PIN_` define naming them.
- **PA8** is in that map as SOF, and is also `PIN_MATRIX_DATA`. SOF is not needed for
  CDC, so this is an ordering rule rather than a clash: the NeoMatrix driver has to
  claim PA8 back **after** `Serial.begin()`.
- **PA0** is the generic H723ZG variant's default UART4 TX for `Serial`, and is
  `PIN_RPM_IN`. That conflict is what put the console on USB CDC instead of the UART;
  leaving it on the UART would have the two fighting over the pin, and the symptom
  would be a quietly wrong RPM reading rather than an obvious failure.
