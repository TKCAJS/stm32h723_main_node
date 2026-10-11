# Wiring sheet — test module

Bench wiring checklist for the STM32H723ZG (WeAct) main node. Pin choices come from
`include/pins.h`, which is the source of truth; `GPIO_MAP.md` is the short table. This
sheet adds what each pin connects to, the parts it needs, and whether firmware drives
it yet.

**Logic is 3.3 V everywhere.** No 5 V signal goes straight onto any pin in this list,
and the two analog inputs in particular must never see more than 3.3 V.

**Driver column:** *yes* = firmware in this repo drives the pin today; *not yet* = the
pin is reserved but no driver exists, so wiring it is safe but nothing will happen.

## CAN buses — each needs its own 3.3 V transceiver

| Pin | Signal | Connects to | Driver |
|---|---|---|---|
| PD0 | FDCAN1_RX — main bike bus | transceiver #1 RXD | yes |
| PD1 | FDCAN1_TX — main bike bus | transceiver #1 TXD | yes |
| PB12 | FDCAN2_RX — clutch servo bus | transceiver #2 RXD | yes |
| PB13 | FDCAN2_TX — clutch servo bus | transceiver #2 TXD | yes |

- Two transceivers, two separate CANH/CANL pairs. The servo bus is a second bus, not a
  stub off the bike bus.
- Use a 3.3 V transceiver (e.g. SN65HVD230 / TJA1051T/3); a 5 V-only part's RXD would
  drive 5 V into the pin.
- 120 Ω across CANH–CANL at each end of each bus. On the bench, one node + the servo
  means one 120 Ω at the board and one at the servo (check whether the servo has it
  built in before adding a second).
- Main bus: 500 kbit/s. Servo bus: 250 kbit/s, the servo's factory rate.

## Clutch paddle halls — analog, ADC2

| Pin | Signal | Connects to | Driver |
|---|---|---|---|
| PA2 | HALL_LEFT | left paddle hall output | yes |
| PA3 | HALL_RIGHT | right paddle hall output | yes |

- **0–3.3 V max.** Power the halls from 3.3 V, or divide a 5 V sensor's output down. The
  pins have no pull resistor configured.
- Run the sensor ground back to the board's analog ground, close to the pin.

## Shift and mode inputs — interrupt-driven, active LOW

All five use the internal pull-up: wire each switch **between the pin and GND**. No
external resistor needed on the bench.

| Pin | Signal | Closed (to GND) means | Driver |
|---|---|---|---|
| PB4 | SHIFT_UP | paddle pressed | yes |
| PB5 | SHIFT_DOWN | paddle pressed | yes |
| PB6 | NEUTRAL | button pressed | yes |
| PB7 | MANUAL_MODE | manual mode (toggle switch, not a button) | yes |
| PB8 | LOG_MARK | stamp a marker into the log | yes |

- PB4 is NJTRST at reset; harmless, SWD (PA13/PA14) is what the ST-Link uses.
- On the bike these runs want filtering (RC + Schmitt or similar). For the test module,
  bare switches are fine.

## RPM input — hardware counter

| Pin | Signal | Connects to | Driver |
|---|---|---|---|
| PA0 | RPM_IN (TIM2_CH1/ETR) | MAX9926 digital output | not yet |

- The MAX9926 output is open-drain: pull it up to **3.3 V**, not 5 V. If it sits on a
  breakout board, check what that board already pulls it up to.
- Keep the VR front end physically away from any radio — this is why the node has none.

## NeoMatrix — 16×16 WS2812

| Pin | Signal | Connects to | Driver |
|---|---|---|---|
| PC6 | MATRIX_DATA (TIM8_CH1 + DMA) | matrix DIN, via level shifter | not yet |

- WS2812 at 5 V wants a data high above ~3.5 V: put a 74AHCT125 (or similar) between
  PC6 and DIN, plus ~330 Ω series at DIN.
- The matrix takes its 5 V from its own supply, not the board. Share ground.

## Diagnostic encoder — KY-040

| Pin | Signal | Connects to | Driver |
|---|---|---|---|
| PE0 | ENC_CLK | KY-040 CLK | not yet |
| PE1 | ENC_DT | KY-040 DT | not yet |
| PE2 | ENC_SW | KY-040 SW | not yet |

- Power the KY-040 (its "+") from **3.3 V**, so its on-board pull-ups stay at 3.3 V.

## USB console

| Pin | Signal | Connects to | Driver |
|---|---|---|---|
| PA11 | USB D− | board's USB-C (already wired) | yes |
| PA12 | USB D+ | board's USB-C (already wired) | yes |
| PA9 | **reserved** — VBUS sense, claimed by USB | leave unconnected | — |
| PA10 | **reserved** — ID, claimed by USB | leave unconnected | — |
| PD5 | fallback console TX (USART2) | USB-UART RX, only if USB is disabled | build option |
| PD6 | fallback console RX (USART2) | USB-UART TX, only if USB is disabled | build option |

- Nothing to wire for the normal console: it is the board's own USB-C. 115200 8N1,
  local echo off.
- Once the node runs off bike power, the USB link must be data-only: don't let the
  board and the host feed each other's 5 V.

## On the WeAct board already — no wiring

| Pin(s) | Function | Driver |
|---|---|---|
| PE10–PE14 | 0.96" ST7735 status LCD (BLK, CS, SCK, DC, MOSI) | yes |
| PC8–PC12, PD2 | microSD slot, SDMMC1 4-bit | not yet |
| PC13 | onboard user button | not yet |
| PA13 / PA14 | SWD for the ST-Link | — |

## Leave free

| Pin(s) | Why |
|---|---|
| PA6 | freed when the clutch servo stopped being PWM |
| PC0 / PC1 | freed ADC1 differential pair, kept for a future sensor |
| PA8 | in the USB pin map as SOF; freed when the matrix moved to PC6 |
| PB9 | spare; was the ARM/SAFE switch, now dropped |

## Power and ground

- One common ground for the board, both transceivers, the halls, the MAX9926, the
  matrix supply and the servo's CAN ground.
- The clutch servo has its own supply; only its CAN pair and ground reach this board.
- Bench first power-up: servo bus connected but servo unpowered,
  check the LCD comes up and `status` answers on the console, then power the servo.
