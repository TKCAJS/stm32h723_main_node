// T89 main node — STM32H723ZGT6.
//
// Inputs, CAN to the rear node, CAN to the clutch servo, the gearbox state
// machine, and a diagnostic readout.
//
// The state machine (gearbox.h) is a port of src/main_node/GearboxStateMachine.*
// in the T89 repo — the logic is unchanged, only the I/O underneath it was
// replaced. What was PWM out and an ADS1115 voltage in is a servo on its own
// CAN bus, commanded and read in counts (clutch.h).
//
// The paddle->clutch response curve (paddle.h) is a port of HallSensorControl.h.
//
// Not yet: the RPM counter, the NeoMatrix and SD logging.
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
#include "gearbox.h"
#include "paddle.h"
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
// core for the best part of a second. It only ever runs with the engine
// stopped (queued until then), and it will leave a mark here — that mark is expected.
static uint32_t _maxLoopUs  = 0;
static uint32_t _loopStalls = 0;
#define LOOP_STALL_US 20000

// At or above this the engine counts as running, and `cal save` — which stalls
// the core for the flash erase — is deferred until it stops. Calibration itself
// (capture, set, jogging the clutch) works with the engine running; that is
// how a bite point is found. Until the RPM counter is wired rpm reads 0, so a
// save always goes straight through on the bench.
#define ENGINE_RUNNING_RPM  500

static uint32_t _upCount = 0, _downCount = 0, _neutralCount = 0, _markCount = 0;
static bool     _manualMode = false;

static void handle_input(const InputEvent *e) {
    switch (e->id) {
        // Auto mode: presses go to the state machine. Manual mode: raw shift
        // pulses to the rear node — no clutch, no interlock — as ManualMode.h
        // did on the ESP32 node. Neutral has no manual action.
        case INPUT_SHIFT_UP:
            if (!e->pressed) break;
            _upCount++;
            if (_manualMode) gearbox_manual_shift(true);
            else             gearbox_shift_up_pressed();
            break;

        case INPUT_SHIFT_DOWN:
            if (!e->pressed) break;
            _downCount++;
            if (_manualMode) gearbox_manual_shift(false);
            else             gearbox_shift_down_pressed();
            break;

        case INPUT_NEUTRAL:
            if (!e->pressed) break;
            _neutralCount++;
            if (!_manualMode) gearbox_neutral_pressed();
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

        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);

    status_lcd_init();
    status_lcd_banner("T89 MAIN H723", true);

    inputs_init();
    // A switch, so read where it already sits: one left ON across a power
    // cycle produces no edge, and would otherwise boot into auto.
    _manualMode = inputs_level(INPUT_MANUAL_MODE);

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
    gearbox_init();

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

    // After both buses have been drained, so the gear and the clutch position
    // it decides on are the freshest this pass has.
    gearbox_tick(_manualMode);

    // Paddle -> clutch, after the state machine so it knows who owns the
    // clutch this pass: the paddle while the gearbox is idle, the state
    // machine during a shift (as the ESP32's clutch task stood off whenever
    // the gearbox was not idle). Manual mode is a straight map.
    const uint16_t hallL = hall_read_left();
    const uint16_t hallR = hall_read_right();
    paddle_tick(hallL, hallR, _manualMode        ? PADDLE_DIRECT
                            : gearbox_is_idle()  ? PADDLE_CURVED
                                                 : PADDLE_OFF);

    // TODO: rpm from the TIM2 hardware counter (PA0), once wired.
    uint16_t rpm = 0;
    can_tick(rpm, _manualMode);

    StatusInfo s = {};
    s.gear              = can_gear();
    s.gearValid         = can_gear_valid();
    s.rpm               = rpm;
    s.engineRunning     = rpm >= ENGINE_RUNNING_RPM;
    s.manualMode        = _manualMode;
    s.gbState           = (uint8_t)gearbox_state();
    s.gbShifting        = gearbox_is_shifting();
    s.gbLetter          = gearbox_shift_letter();
    s.gbAborts          = gearbox_abort_count();

    // The clutch line, in servo counts. Reading it is now two register-free
    // accessor calls over state the servo bus already delivered — no ADC, no
    // divider, and a "stale" that means what it says.
    s.clutchCounts      = clutch_position();
    s.clutchFeedbackOk  = clutch_feedback_ok();
    s.clutchDisengaged  = clutch_disengaged();
    s.clutchJustEngaged = clutch_just_engaged();
    s.clutchCommanded   = clutch_commanded();

    s.hallLeft          = hallL;
    s.hallRight         = hallR;
    s.paddlePos         = paddle_position();

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
