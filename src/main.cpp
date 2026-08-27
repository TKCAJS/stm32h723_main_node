// T89 main node — STM32H723ZGT6. Bring-up scaffold.
//
// What this is: the hardware layer, proving the pin map on the bench. Inputs,
// CAN to the rear node, CAN to the clutch servo, and a diagnostic readout.
//
// What this is NOT yet: the gearbox state machine. That is a deliberate port
// from src/main_node/GearboxStateMachine.* in the T89 repo and wants its own
// review — the logic is sound and hard-won, it is only the I/O underneath it
// that is being replaced. The clutch half of that I/O is now done: what was
// PWM out and an ADS1115 voltage in is a servo on its own CAN bus, commanded
// and read in counts (clutch.h). The state machine's two clutch predicates
// survive verbatim in meaning — clutch_disengaged() and clutch_just_engaged().
//
// The rule this node exists to enforce: nothing in loop() may block. No bus
// wait, no delay(), no retry loop. Paddles arrive by interrupt and are already
// timestamped by the time the loop looks at them, so even if the loop does
// stall, a press is queued rather than lost.

#include <Arduino.h>
#include "pins.h"
#include "inputs.h"
#include "can_main.h"
#include "servo_can.h"
#include "clutch.h"
#include "hall_adc.h"
#include "status_lcd.h"
#include "serial_cli.h"
#include "node_status.h"
#include "can_ids.h"

// Loop-time watchdog, carried over from the ESP32 node's v214 fix. There the
// number was the evidence that a blocking I2C read was eating shift commands.
// Here it should sit in the low hundreds of microseconds and stay there.
//
// One legitimate exception: `cal save` erases a flash sector and stalls the
// core for the best part of a second. It is reachable only while ARM/SAFE
// reads SAFE, and it will leave a mark here — that mark is expected.
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

    if (!hall_adc_init()) {
        status_lcd_banner("ADC FAIL", false);
        while (true) delay(1000);      // no paddle position, no clutch control
    }

    if (!can_init()) {
        status_lcd_banner("CAN FAIL", false);
        while (true) delay(1000);
    }

    // The servo bus. A failure here is fatal for the same reason the ADS1115
    // was: without it there is no clutch position and no way to move the
    // clutch, so there is no safe shifting either.
    if (!servo_can_init()) {
        status_lcd_banner("SERVO CAN FAIL", false);
        while (true) delay(1000);
    }

    // Calibration comes from flash when it is there and valid. When it is not,
    // the factory travel stands in — deliberately not a fault: an uncalibrated
    // node still has to boot and be driveable to the point where it can be
    // calibrated over the console.
    clutch_init();

    serial_cli_init();

    status_lcd_banner("READY", true);
}

void loop() {
    const uint32_t loopStartUs = micros();

    // Drain every queued paddle event. These were timestamped in the handler,
    // so their timing is already recorded no matter how late we got here.
    InputEvent e;
    while (inputs_pop(&e)) handle_input(&e);

    can_poll();

    // Servo bus: drain replies, retire a dead request, send what is due. Like
    // can_poll() it only ever moves what has already arrived.
    servo_can_poll();

    // TODO: rpm from the TIM2 hardware counter (PA0), once wired.
    uint16_t rpm = 0;
    can_tick(rpm, _manualMode);

    StatusInfo s = {};
    s.gear              = can_gear();
    s.gearValid         = can_gear_valid();
    s.rpm               = rpm;
    s.armed             = _armed;
    s.manualMode        = _manualMode;

    // The clutch line, in servo counts. Reading it is now two register-free
    // accessor calls over state the servo bus already delivered — no ADC, no
    // divider, and a "stale" that means what it says.
    s.clutchCounts      = clutch_position();
    s.clutchFeedbackOk  = clutch_feedback_ok();
    s.clutchDisengaged  = clutch_disengaged();
    s.clutchJustEngaged = clutch_just_engaged();
    s.clutchCommanded   = clutch_commanded();

    s.hallLeft          = hall_read_left();
    s.hallRight         = hall_read_right();

    s.canRxCount        = can_rx_count();
    s.canGearAgeMs      = can_gear_age_ms();
    s.canLive           = can_bus_live();

    const ServoStatus *sv = servo_status();
    s.servoRxCount      = sv->rxCount;
    s.servoTimeouts     = sv->timeouts;
    s.servoLatUs        = sv->latLastUs;
    s.servoBusOff       = sv->busOff;

    s.maxLoopUs         = _maxLoopUs;
    s.inputsDropped     = inputs_dropped();
    s.shiftUpCount      = _upCount;
    s.shiftDownCount    = _downCount;
    s.neutralCount      = _neutralCount;
    s.markCount         = _markCount;

    status_lcd_tick(&s);
    serial_cli_tick(&s);

    const uint32_t loopUs = micros() - loopStartUs;
    if (loopUs > _maxLoopUs)   _maxLoopUs = loopUs;
    if (loopUs > LOOP_STALL_US) _loopStalls++;
}
