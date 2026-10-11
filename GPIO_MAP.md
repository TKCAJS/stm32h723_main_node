# GPIO map

Assigned pins, from `include/pins.h` — that file is the source of truth; this is a
quick-reference copy of it. If the two disagree, `pins.h` wins. For what each pin
connects to and the parts it needs, see `WIRING.md`.

**Bold** = actually needs external wiring on this build. Everything else is either
already on the board (LCD, SD slot, USB-C, onboard button) or not wired yet
(diagnostic encoder — no driver touches it in `src/` yet, fallback UART).

| Pin | Function |
|---|---|
| **PD0** | **FDCAN1_RX (main bus)** |
| **PD1** | **FDCAN1_TX (main bus)** |
| **PB12** | **FDCAN2_RX (servo bus)** |
| **PB13** | **FDCAN2_TX (servo bus)** |
| **PA2** | **HALL_LEFT (ADC12_INP14)** |
| **PA3** | **HALL_RIGHT (ADC12_INP15)** |
| **PA0** | **RPM_IN (TIM2_CH1/ETR)** — also the variant's default UART4 TX, see note |
| **PC6** | **MATRIX_DATA (NeoMatrix, TIM8_CH1)** |
| **PB4** | **SHIFT_UP (EXTI4)** |
| **PB5** | **SHIFT_DOWN (EXTI5)** |
| **PB6** | **NEUTRAL (EXTI6)** |
| **PB7** | **MANUAL_MODE (EXTI7)** |
| **PB8** | **LOG_MARK (EXTI8)** |
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
| PD5 | fallback console TX (USART2) — only in a build with the USB flags removed |
| PD6 | fallback console RX (USART2) — only in a build with the USB flags removed |
| PA9 | **reserved** — USB_OTG_HS_VBUS, claimed by the USB stack |
| PA10 | **reserved** — USB_OTG_HS_ID, claimed by the USB stack |
| PA11 | USB_DM — **console** (`Serial` = USB CDC, OTG_HS internal FS PHY) |
| PA12 | USB_DP — **console** (`Serial` = USB CDC, OTG_HS internal FS PHY) |

Left unassigned on purpose: PC0/PC1 (freed ADC1 differential pair, kept spare), PA6
(freed when the clutch servo stopped being PWM), PB9 (was the ARM/SAFE switch,
dropped), and PA8 (freed when the NeoMatrix moved to PC6 to clear USB).

## Console

| Build | `Serial` is | Pins | Set by |
|---|---|---|---|
| Default | USB CDC (OTG_HS, internal FS PHY) | PA11 D− / PA12 D+, on the board's USB-C | `-DUSBCON -DUSBD_USE_CDC -DUSE_USB_HS -DUSE_USB_HS_IN_FS` |
| Fallback | USART2 | PD5 TX / PD6 RX | `-DPIN_SERIAL_TX=PD5 -DPIN_SERIAL_RX=PD6`, live only once the four USB flags are removed |

Both are set in `platformio.ini`. Enabling USB also claims PA8–PA10 (below). The
connection is 115200 8N1, local echo off (the node echoes), and CR, LF or CRLF all end
a line. The node is bike-powered, so the USB link is data-only: VBUS sensing is off,
and nothing should be powered from, or feed power back into, the host's VBUS.

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
