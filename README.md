# T89 main node — STM32H723ZGT6

Replacement for the ESP32-S3 main node. Bench/bring-up stage; nothing is wired yet,
so the pin map in `include/pins.h` is a proposal to argue with, not a record.

## Why move

Two separate reasons, worth keeping distinct because they need different fixes.

**Interference.** `MAX9926_EMI_notes.md` in the T89 repo records RPM dropping to zero
*in step with WiFi TX*. The dominant RF aggressor was the main node's own radio, sitting
next to the VR front end. This node has no radio at all, and the display node's radio
moves to the other end of the bike.

**Blocking.** The paddles went unresponsive at high revs because a blocking I2C read sat
in `loop()` next to a *polled* paddle edge-detect: a press that began and ended inside a
stall was never seen. That is an architecture bug, and an STM32 would have reproduced it
faithfully. It is fixed here by construction — paddles are EXTI interrupts, timestamped
in the handler.

## The rule

Nothing in `loop()` may block. No bus wait, no `delay()`, no retry loop. Where a
peripheral could stall, it gets DMA, a hardware counter, or a free-running conversion.
`maxLoopUs` on the status LCD is the check: low hundreds of microseconds, and `drop`
must never leave 0.

## Pin map

Full detail and rationale in `include/pins.h`. Summary:

| Function | Pins | Notes |
|---|---|---|
| FDCAN1 | PD0 RX / PD1 TX | same as `stm32h723_transmitter_node`, 500k classic |
| Clutch feedback | **PC0 (INP) / PC1 (INN)** | ADC1 ch10 **differential**, 16-bit, 64× oversampled |
| Paddle halls | PA2 / PA3 | ADC2, single-ended |
| RPM | PA0 | TIM2 external clock — the PCNT equivalent, 32-bit |
| Clutch servo | PA6 | TIM3_CH1 hardware PWM |
| NeoMatrix | PA8 | TIM1_CH1 + DMA |
| Paddles/switches | PB4–PB9 | EXTI, distinct pin numbers, active LOW |
| Encoder | PE0/PE1/PE2 | KY-040, driver lifts from the transmitter node |
| Status LCD | PE10–PE14 | onboard ST7735, fixed by the board |
| SD | PC8–PC12, PD2 | SDMMC1 4-bit |
| USB CDC | PA11 / PA12 | calibration link |

### Replacing the ADS1115

The ADS was never the problem — its *bus* was. What made it good was reading
**differentially** against the servo's own ground, rejecting the ground bounce that made
the single-ended GPIO15 read useless. ADC1 keeps that and drops the bus:

- **PC0** ← divided servo feedback (keep the 0.591 divider; H723 pins are not 5 V tolerant)
- **PC1** ← the **servo's own ground return**, run back as its own conductor. Not a
  convenient board ground. Measuring across the loom instead of across the servo throws
  away the entire benefit.

16-bit with 64× hardware oversampling, free-running, so a read is a register access that
cannot block or fail. Differential zero sits at mid-scale, so with a 0–2.96 V input only
the upper half of the range is used — still ~15 bits across the travel, far more than the
job needs.

## Confirm before wiring

- **PC0/PC1 = ADC1_INP10/ADC1_INN10** on the H723ZG package, from the datasheet pin table.
  `INN[N]` shares a pin with `INP[N+1]`, so an off-by-one is silent — it just reads dead.
  Only `ADC_CH_CLUTCH` in `pins.h` changes if it is wrong.
- PA2/PA3 as ADC12_INP14/INP15, same table.
- SDMMC1 and the onboard LCD pins against the WeAct board schematic.

## State

Implemented and building: pin map, EXTI input capture with debounce and a timestamped
event queue, FDCAN1 to the rear node, ADC1 differential clutch feedback, ADC2 halls,
diagnostic LCD, loop-time watchdog.

Not yet: the gearbox state machine, RPM counter, servo PWM, NeoMatrix driver, SD logging,
USB CDC command set. The state machine is a deliberate port of
`src/main_node/GearboxStateMachine.*` — that logic is sound and hard-won; it is only the
I/O underneath it being replaced.

**Never call `analogRead()` in this project.** It resolves most pins to ADC1 and
reconfigures it, tearing down the differential channel. Analog goes through
`clutch_adc.h`.

## Calibration UI

No radio here, so this node cannot serve the pages. The driving requirement is the pit
lane: an Android tablet, no internet. That rules out more than it first appears —
**Chrome on Android has no Web Serial API at all** (desktop and ChromeOS only), and
`file://` is not a secure context, so an HTML file opened off an SD card cannot reach USB
either. Any answer that ends in "the tablet talks to the node over USB" is dead.

**Primary path — the display node already does this.** `P4Display_node`'s `PitServer.cpp`
serves `VIEWER_HTML` from PROGMEM over the SoftAP the C6 brings up. The calibration pages
belong there, next to the viewer page, with the dash proxying get/set to this node over
CAN. Tablet joins the AP, browses to the dash. No internet, no USB, works on Android.

`CAN_MAIN_CFG_REQ` (MSGTYPE 0x40) is already reserved for this. A key/value protocol fits
an 8-byte frame: `[seq][status][key:2][value:4]`, with the key table in a shared header so
the dash and this node cannot disagree about the meaning of a parameter.

Two rules for that channel:
- **Writes only while ARM/SAFE reads SAFE** (or the engine is stopped). Nobody changes
  shift timings from a tablet mid-session.
- Values live in this node's own flash. The dash is an editor, never the source of truth —
  the node must run standalone with the dash unplugged.

**Secondary path — laptop on the bench.** The same page over Web Serial on USB CDC, no
dash involved. Convenient for deep bring-up work, but it is the desktop-only path.

**Always available.** A plain text command set on USB CDC, so a terminal works when
neither of the above does.
