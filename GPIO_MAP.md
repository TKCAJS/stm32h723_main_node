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
| PC6 | MATRIX_DATA (NeoMatrix, TIM8_CH1) |
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
| PD5 | fallback console TX (USART2) — only in a non-USB build |
| PD6 | fallback console RX (USART2) — only in a non-USB build |
| PA9 | **reserved** — USB_OTG_HS_VBUS, claimed by the USB stack |
| PA10 | **reserved** — USB_OTG_HS_ID, claimed by the USB stack |
| PA11 | USB_DM (OTG_HS internal FS PHY) |
| PA12 | USB_DP (OTG_HS internal FS PHY) |

Left unassigned on purpose: PC0/PC1 (freed ADC1 differential pair, kept spare), PA6
(freed when the clutch servo stopped being PWM), and PA8 (freed when the NeoMatrix
moved to PC6 to clear USB).

## Two pins that are not as free as they look

The USB device stack configures **every** pin in the OTG_HS pin map, not just D-/D+.
With USB CDC enabled that is PA8, PA9, PA10, PA11 and PA12.

- **PA9 / PA10** carry VBUS and ID. Nothing else may use them, which is why they are
  listed above despite no `PIN_` define naming them.
- **PA8** is in that map as SOF, and used to be `PIN_MATRIX_DATA`. That was survivable
  as an ordering rule — claim it back after `Serial.begin()` — but it was a trap set
  for whoever writes the matrix driver, so the matrix moved to PC6 and PA8 is now
  spare.
- **PA0** is the generic H723ZG variant's default UART4 TX for `Serial`, and is
  `PIN_RPM_IN`. That is why the console is USB CDC rather than a UART. The fallback
  UART is pinned to PD5/PD6 so that a build with USB disabled cannot land back on
  PA0 and quietly corrupt the RPM count.

## Verified against the pin tables

Checked against `PeripheralPins.c` for this exact variant
(`STM32H7xx/H723Z(E-G)T_H730ZBT_H733ZGT`) in the Arduino core, which is generated from
ST's package data:

PD0/PD1 FDCAN1 RX/TX (AF9) · PB12/PB13 FDCAN2 RX/TX (AF9) · PA2/PA3 ADC12_INP14/INP15
· PA0 TIM2_CH1 · PC6 TIM8_CH1 (AF3) · PC8–PC12 + PD2 SDMMC1 · PA11/PA12 USB · PD5/PD6
USART2.

The LCD pins are stronger than that: they are proven in service on
`stm32h723_transmitter_node`, same board, same panel. The SD pins are the weakest —
valid for SDMMC1, but no driver has ever driven them here.

Two near-misses worth knowing, neither of them live: **PB13** is also SDMMC1_D0 (we
use PC8 for D0), and **PB8/PB9** are also FDCAN1 RX/TX alternates (we use PD0/PD1).
