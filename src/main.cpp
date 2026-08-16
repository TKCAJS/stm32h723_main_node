// T89 main node — STM32H723ZGT6. Bring-up scaffold.
//
// What this is: the hardware layer, proving the pin map on the bench. Inputs,
// CAN to the rear node, clutch feedback, and a diagnostic readout.
//
// What this is NOT yet: the gearbox state machine. That is a deliberate port
// from src/main_node/GearboxStateMachine.* in the T89 repo and wants its own
// review — the logic is sound and hard-won, it is only the I/O underneath it
// that is being replaced.
//
// The rule this node exists to enforce: nothing in loop() may block. No bus
// wait, no delay(), no retry loop. Paddles arrive by interrupt and are already
// timestamped by the time the loop looks at them, so even if the loop does
// stall, a press is queued rather than lost.

#include <Arduino.h>
#include "pins.h"
#include "inputs.h"
#include "can_main.h"
#include "clutch_adc.h"
#include "status_lcd.h"
#include "can_ids.h"

// Clutch thresholds, in volts at the ADC pin. Placeholders until captured live
// through the calibration UI, exactly as on the ESP32 node: the numbers are
// meaningless in the abstract, only their position relative to the travel is.
static float _clutchDisengageV = 1.8f;

// Loop-time watchdog, carried over from the ESP32 node's v214 fix. There the
// number was the evidence that a blocking I2C read was eating shift commands.
// Here it should sit in the low hundreds of microseconds and stay there.
static uint32_t _maxLoopUs  = 0;
static uint32_t _loopStalls = 0;
#define LOOP_STALL_US 20000

static uint32_t _upCount = 0, _downCount = 0, _neutralCount = 0, _markCount = 0;
static bool     _manualMode = false;
static bool     _armed      = true;

static void handle_input(const InputEvent *e) {
    switch (e->id) {
        case INPUT_SHIFT_UP:
            if (e->pressed) _upCount++;
            break;

        case INPUT_SHIFT_DOWN:
            if (e->pressed) _downCount++;
            break;

        case INPUT_NEUTRAL:
            if (e->pressed) _neutralCount++;
            break;

        // A switch, not a long-press. The ESP32 node overloaded a button hold
        // for this; a rider in leathers should not have to time a press.
        case INPUT_MANUAL_MODE:
            _manualMode = e->pressed;
            break;

        // Stamps the moment into the log so a session can be searched for
        // "that felt wrong" rather than scrubbed end to end.
        case INPUT_LOG_MARK:
            if (e->pressed) _markCount++;
            break;

        // Hard inhibit while working on the bike. Software must treat this as
        // authoritative; the wiring should also break the output path so a
        // firmware bug cannot move the box in the pits.
        case INPUT_ARM_SAFE:
            _armed = !e->pressed;
            break;

        default:
            break;
    }
    // Shift commands are deliberately not sent here yet — the state machine
    // port owns that decision, including the clutch and stacking rules.
}

void setup() {
    Serial.begin(115200);

    status_lcd_init();
    status_lcd_banner("T89 MAIN H723", true);

    inputs_init();
    _armed = !inputs_level(INPUT_ARM_SAFE);

    if (!clutch_adc_init()) {
        status_lcd_banner("ADC FAIL", false);
        while (true) delay(1000);      // no clutch feedback, no safe shifting
    }

    if (!can_init()) {
        status_lcd_banner("CAN FAIL", false);
        while (true) delay(1000);
    }

    status_lcd_banner("READY", true);
}

void loop() {
    const uint32_t loopStartUs = micros();

    // Drain every queued paddle event. These were timestamped in the handler,
    // so their timing is already recorded no matter how late we got here.
    InputEvent e;
    while (inputs_pop(&e)) handle_input(&e);

    can_poll();

    // Free-running ADC: a register read, never a wait.
    float clutchV = clutch_adc_volts();

    // TODO: rpm from the TIM2 hardware counter (PA0), once wired.
    uint16_t rpm = 0;
    can_tick(rpm, _manualMode);

    StatusInfo s = {};
    s.gear             = can_gear();
    s.gearValid        = can_gear_valid();
    s.rpm              = rpm;
    s.armed            = _armed;
    s.manualMode       = _manualMode;
    s.clutchVolts      = clutchV;
    s.clutchDisengaged = (clutchV < _clutchDisengageV);
    s.hallLeft         = hall_read_left();
    s.hallRight        = hall_read_right();
    s.canRxCount       = can_rx_count();
    s.canGearAgeMs     = can_gear_age_ms();
    s.canLive          = can_bus_live();
    s.maxLoopUs        = _maxLoopUs;
    s.inputsDropped    = inputs_dropped();
    s.shiftUpCount     = _upCount;
    s.shiftDownCount   = _downCount;
    s.neutralCount     = _neutralCount;
    s.markCount        = _markCount;
    status_lcd_tick(&s);

    const uint32_t loopUs = micros() - loopStartUs;
    if (loopUs > _maxLoopUs)   _maxLoopUs = loopUs;
    if (loopUs > LOOP_STALL_US) _loopStalls++;
}
