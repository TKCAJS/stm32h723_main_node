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
| FDCAN1 — bike bus | PD0 RX / PD1 TX | same as `stm32h723_transmitter_node`, 500k classic |
| FDCAN2 — servo bus | **PB12 RX / PB13 TX** | clutch servo only, 250k classic |
| Paddle halls | PA2 / PA3 | ADC2, single-ended — the only analog left |
| RPM | PA0 | TIM2 external clock — the PCNT equivalent, 32-bit |
| NeoMatrix | PA8 | TIM1_CH1 + DMA |
| Paddles/switches | PB4–PB9 | EXTI, distinct pin numbers, active LOW |
| Encoder | PE0/PE1/PE2 | KY-040, driver lifts from the transmitter node |
| Status LCD | PE10–PE14 | onboard ST7735, fixed by the board |
| SD | PC8–PC12, PD2 | SDMMC1 4-bit |
| USB CDC | PA11 / PA12 | calibration link (the console also runs on a UART) |

### Replacing the ADS1115 — and then the whole analog path

First pass kept the measurement and dropped the bus: ADC1 read the servo's feedback
pot **differentially** (PC0/PC1) against the servo's own ground, 16-bit and 64×
oversampled, free-running so a read could not block. That was the right answer while
the actuator was a hobby servo with a pot and three wires.

The actuator is now a **Wingxine ASMG-MD** on native CAN, so the analog path is gone
entirely rather than improved:

- **No PWM out.** Position is a `0x01` frame carrying target and travel speed.
- **No feedback in.** The servo reports its own position, in counts, in reply to a
  `0x07` poll — along with present current, which the pot never offered.
- **No divider, no feedback conductor, no ADS1115, no `clutchVoltage`.** PC0/PC1 and
  PA6 come out of the loom and are left unassigned in `pins.h`.

What the differential read was buying — rejection of ground bounce across the loom —
is not needed by a number that was never analog on the wire. What it could never buy
is the distinction the CAN path gets for free: **steady and dead now look different.**
A stuck ADC reading and a clutch holding still were the same number. A servo that has
stopped answering is visible as such, and every predicate in `clutch.h` answers false
while the feedback is stale rather than deciding on a position nobody confirmed.

### Why the servo gets its own controller

FDCAN2, not a second address on the bike bus. Three reasons, in order of weight:

1. **Rate.** Clutch telemetry is request/reply at 50 Hz — two frames per poll, all
   day, of interest to exactly one node.
2. **Bit rate.** The servo ships at 250 kbit/s. The bike bus is 500 kbit/s and is not
   being re-rated to suit one actuator.
3. **Blast radius.** The servo's arbitration IDs are the manufacturer's
   (`0x18EF0201`, J1939-shaped) and sit outside `can_ids.h` entirely. A confused
   servo cannot talk over a gear report it cannot see.

The transport is a port of `ServoBus.cpp` from `esp32_T89Display_cansniff` — the
touchscreen sniffer that proved the protocol on the bench. The frame codec
(`lib/asmg_servo/asmg_md_can.h`) is that project's file byte for byte, so a
correction found on the bench is a correction here, in one place. Its `[VERIFY]`
items — reply arbitration ID, 16-bit word order, what the `0xFD` reply actually
carries — are still bench questions; the console below can settle each one on the
real node, and the acceptance filter is left fully open so a wrong assumption shows
up as an odd ID rather than as silence.

One deliberate difference from the bench tool: a position command **jumps the poll
queue**. The sniffer sent nothing while a reply was outstanding, which is right for a
diagnostic and wrong for a clutch — it would put a stall of up to a reply timeout
back into the shift path.

### Two triggers, carried across intact

Every clutch decision on the ESP32 node was a comparison against a captured voltage:

```
clutchPulled      = (clutchVoltage <  clutchDisengageV)
clutchJustEngaged = (clutchVoltage >= clutchDisengageV && clutchVoltage <= clutchJustEngagedV)
```

Both survive in `clutch.h` as `clutch_disengaged()` and `clutch_just_engaged()`, with
the same meaning and the same role in the state machine to come. Only the quantity
changed, from divided volts to servo counts.

The **inversion did not survive, on purpose.** The old feedback ran backwards —
pulling drove the voltage down — and that fact was written into every comparison in
the codebase. Which way the servo's counts run depends on how the arm is mounted, so
direction is derived from the calibration instead: whichever of `engaged`/`disengaged`
is numerically larger defines "toward disengaged" for every threshold test.

## Confirm before wiring

- **PB12/PB13 = FDCAN2_RX/FDCAN2_TX** on the H723ZG package. The only other option is
  PB5/PB6, which are paddle EXTI inputs — so if this is wrong, two paddles move.
- PA2/PA3 as ADC12_INP14/INP15, from the datasheet pin table.
- SDMMC1 and the onboard LCD pins against the WeAct board schematic.
- The servo bus needs its own transceiver and its own 120 Ω termination pair. It is a
  second bus, not a stub off the first.

## State

Implemented: pin map, EXTI input capture with debounce and a timestamped event queue,
FDCAN1 to the rear node, FDCAN2 to the clutch servo with the polled position engine,
the clutch position/trigger layer, calibration in flash, ADC2 halls, the serial
console, diagnostic LCD, loop-time watchdog.

Not compiled yet — the PlatformIO registry is unreachable from the machine this was
written on, so none of it has been through a toolchain, let alone hardware.

Not yet: the gearbox state machine, RPM counter, the paddle→clutch response curve
(`HallSensorControl`'s piecewise/log mapping, which now targets
`clutch_command_pull()` instead of a servo angle), NeoMatrix driver, SD logging.
The state machine is a deliberate port of
`src/main_node/GearboxStateMachine.*` — that logic is sound and hard-won; it is only the
I/O underneath it being replaced.

**Never call `analogRead()` in this project.** It resolves most pins to ADC1 and
reconfigures whatever it finds there. Analog goes through `hall_adc.h`.

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

**Always available, and now built.** A plain text command set on the serial port —
`src/serial_cli.cpp`. This is the web interface's replacement, not a stopgap: there is
no server on this node and there never will be, so every knob `WebInterface.h` had is
here, plus the sniffer's bench controls, because settling the servo protocol has to
stay repeatable on the real node.

`Serial` is whichever port the build maps it to — the variant's UART today; add
`-DUSBCON -DUSBD_USE_CDC` to `build_flags` to move it to USB CDC on PA11/PA12. The
console does not care which, and does not block on either: input is drained a bounded
number of bytes per pass, output goes into a ring emptied only as fast as the port
will take it without waiting. Under pressure it drops **whole lines** and counts them
— a half-written line is worse than a missing one, and a console is never allowed to
be the thing that stalls `loop()`.

```
status              full health dump: both buses, clutch, loop time
mon [ms|off]        one live line, streaming
trace on|off        raw servo-bus frames — ms, direction, ID, 8 bytes

poll on|off         position polling (ON at boot: it is the only position sense)
pollms <ms>         poll period          pollcmd 2|7   0x02 pos+cmdpos, 0x07 pos+current
addr <n> / adopt    servo payload address, or take it from the last readid reply
endian be|le        16-bit word order at runtime
readid readpid readcfg      one-shot reads          clear   zero the counters

pos <counts> [spd]  raw position, NOT clamped by calibration
nudge <+/-delta>    relative, clamped to calibrated travel
pull <percent>      0 = engaged, 100 = disengaged
release             command the engaged limit

cal                        show it, alongside the live position
cal capture <field>        name the CURRENT position: engaged|disengaged|trigger|bite
cal set <field> <value>    engaged|disengaged|trigger|bite|speed
cal save | load | defaults
```

`cal capture` is the direct equivalent of the web page's "capture live voltage as the
threshold" buttons: drive the clutch to the point that matters, then name it. Values
are rejected rather than clamped, and a single field is validated against the whole
struct — firmware is the authority, exactly as `calValidate()` was on the ESP32.

`pos` is deliberately the one command that ignores the calibrated travel. Finding a
worn clutch's real stop means going outside the current limits to look for it; `nudge`
and `pull` stay inside them.

Two rules, enforced not just documented:

- **Every write is refused unless ARM/SAFE reads SAFE.** Reads are always allowed.
- **Values live in this node's own flash** (`cal_store.cpp`, last 128 KB sector).
  `cal save` erases that sector, which stalls the core for the best part of a second
  on a single-bank part — the one place this firmware knowingly breaks its own rule,
  which is why it is reachable only while SAFE and never from a control path.

Destructive servo commands — save centre `0x08`, set bitrate `0x09`, set ID `0xFE`,
factory reset `0xFC` — are unreachable from the console, the same line the sniffer
drew.

### Bench sequence on the real node

The sniffer's bring-up list, minus the steps it already answered:

1. `poll off`, then `readid` → `status` shows the reply's arbitration ID. If it is not
   `kReplyCanIdAssumed`, fix that constant in `asmg_md_can.h` (and in the sniffer).
2. `adopt`, or `addr 0x00`.
3. `pos 256 1280` — a small move means big-endian is right. A lunge toward half scale
   means `endian le`; retest, then bake the answer into `ASMG_WORD_BIG_ENDIAN`.
4. `poll on`, `mon` — nudge to each mechanical stop with the arm **disconnected**, and
   `cal capture engaged` / `cal capture disengaged` at them.
5. Reconnect, find the bite point and the downshift gate by feel: `cal capture bite`,
   `cal capture trigger`. Then `cal save`.
6. Watch `status` over a few thousand polls — `latency max` and `timeouts` are the two
   numbers that say whether this bus is trustworthy inside a shift.
