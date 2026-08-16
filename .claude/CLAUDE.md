# stm32h723_main_node — working rules

## Git: commit to `main`, never branch

Commit and push directly to `main`. **Do not create feature branches**, and do not
leave work sitting on one. This overrides the general "if on the default branch,
branch first" default — it does not apply to this project.

There is one developer working across several machines, with GitHub as the single
source of truth. Every machine sits on `main` and pulls `main`, so work parked on a
branch is invisible to all of them. This has already cost real time: in
`P4Display_node`, six commits including a verified touch-panel fix sat on a branch
while another machine pulled `main`, got none of it, and reflashed a build that
predated the fix. Touch stopped working and the work looked lost.

If a stray branch does exist, merge it back into `main` and delete it rather than
continuing on it.

At the start of a session, `git pull` on `main` first. If `main` looks older than
expected, check `git branch -r` for stray branches before concluding work is missing.
A stale local tree still builds cleanly, so nothing warns you — check, do not assume.

## Nothing in `loop()` may block

This node exists because the ESP32 main node lost paddle presses at high revs: a
blocking I2C read sat in `loop()` next to a *polled* paddle edge-detect, so a press
that began and ended inside the stall was never seen.

No bus wait, no `delay()`, no retry loop in the main path. Where a peripheral could
stall it gets DMA, a hardware counter, or a free-running conversion. Inputs arrive by
EXTI and are timestamped in the handler, so their timing is recorded before the loop
ever looks at them.

`maxLoopUs` on the status LCD is the check: low hundreds of microseconds, and `drop`
must never leave 0.

## Never call `analogRead()`

It resolves most pins to ADC1 and reconfigures it, which would tear down the
free-running differential channel that replaced the ADS1115. All analog goes through
`clutch_adc.h`, which owns ADC1 and ADC2 explicitly.

## The pin map is a proposal until the board is built

`include/pins.h` is the single source of truth and the thing to review before any
wire is cut. It carries its own rationale, including which assignments still need
confirming against the H723ZG datasheet and the WeAct schematic.

## Related repos

- `T89_2xESP32_front-rear_with_CanBUS` — the ESP32 node this replaces; the gearbox
  state machine to be ported lives in `src/main_node/GearboxStateMachine.*`
- `stm32h723_transmitter_node` — same board and framework; CAN bring-up, the status
  LCD driver and the KY-040 encoder driver were all lifted from it
- `P4Display_node` — the dash; its `PitServer` will host the calibration pages
- `lib/can_ids/can_ids.h` is shared verbatim across all of them — it is the bus
  contract, so change it in one place and copy, never edit one copy in isolation
