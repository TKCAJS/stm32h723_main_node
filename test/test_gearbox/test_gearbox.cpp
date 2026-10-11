// Gearbox state machine against a simulated clutch servo and rear node.
// Run with: pio test -e native

#include <unity.h>
#include "sim.h"

void setUp() {}
void tearDown() {}

static const ClutchCal *cal() { return clutch_cal(); }

static void expect_shift(int i, bool up, uint16_t ms, uint16_t ignCutMs, uint8_t target) {
    TEST_ASSERT_TRUE_MESSAGE(i < sim.shiftCount, "shift command was never sent");
    TEST_ASSERT_EQUAL(up, sim.shifts[i].up);
    TEST_ASSERT_EQUAL_UINT16(ms, sim.shifts[i].ms);
    TEST_ASSERT_EQUAL_UINT16(ignCutMs, sim.shifts[i].ignCutMs);
    TEST_ASSERT_EQUAL_UINT8(target, sim.shifts[i].target);
}

// ---------------------------------------------------------------------------
// Upshift
// ---------------------------------------------------------------------------

static void test_upshift_sends_one_pulse_and_completes_on_time() {
    sim_reset(2);
    gearbox_shift_up_pressed();

    TEST_ASSERT_EQUAL(GB_UPSHIFTING, gearbox_state());
    TEST_ASSERT_EQUAL(1, sim.shiftCount);
    expect_shift(0, true, 150, 30, 3);

    sim_run(149);
    TEST_ASSERT_EQUAL(GB_UPSHIFTING, gearbox_state());
    sim_run(1);
    // Lands in the expected gear's idle state without waiting for the rear node.
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_3, gearbox_state());

    // An upshift is an ignition cut, never a clutch move.
    TEST_ASSERT_EQUAL_UINT16(cal()->engagedCounts, clutch_commanded());
}

static void test_upshift_timing_follows_the_calibration() {
    sim_reset(2);
    const char *err = "";
    TEST_ASSERT_TRUE(clutch_cal_set("shift_up_ms", 80, &err));
    TEST_ASSERT_TRUE(clutch_cal_set("ign_cut_ms", 12, &err));

    gearbox_shift_up_pressed();
    expect_shift(0, true, 80, 12, 3);

    sim_run(79);
    TEST_ASSERT_EQUAL(GB_UPSHIFTING, gearbox_state());
    sim_run(1);
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_3, gearbox_state());
}

static void test_no_upshift_from_sixth_and_no_downshift_from_first() {
    sim_reset(6);
    uint32_t rejected = gearbox_rejected_count();
    gearbox_shift_up_pressed();
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_6, gearbox_state());

    sim_reset(1);
    gearbox_shift_down_pressed();       // 1st -> N is the neutral button's job
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_1, gearbox_state());

    TEST_ASSERT_EQUAL(0, sim.shiftCount);
    TEST_ASSERT_EQUAL_UINT32(rejected + 2, gearbox_rejected_count());
}

// ---------------------------------------------------------------------------
// Downshift
// ---------------------------------------------------------------------------

static void test_downshift_waits_for_the_clutch_before_shifting() {
    sim_reset(3);
    gearbox_shift_down_pressed();

    TEST_ASSERT_EQUAL(GB_DOWNSHIFT_CLUTCH_ENGAGING, gearbox_state());
    TEST_ASSERT_EQUAL_UINT16(cal()->disengagedCounts, clutch_commanded());
    TEST_ASSERT_EQUAL_MESSAGE(0, sim.shiftCount, "shift sent before the clutch was confirmed pulled");

    // Not a millisecond before the servo is past the trigger.
    while (sim.servoPos < cal()->triggerCounts) {
        TEST_ASSERT_EQUAL(0, sim.shiftCount);
        sim_step();
    }
    TEST_ASSERT_EQUAL(GB_DOWNSHIFT_SHIFTING, gearbox_state());
    expect_shift(0, false, 150, 0, 2);

    sim_run(150);
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_2, gearbox_state());
    TEST_ASSERT_EQUAL_UINT16(cal()->engagedCounts, clutch_commanded());
}

static void test_downshift_is_aborted_when_the_servo_never_gets_there() {
    sim_reset(3);
    sim.servoRate = 0;                  // jammed
    uint32_t aborts = gearbox_abort_count();

    gearbox_shift_down_pressed();
    sim_run(499);
    TEST_ASSERT_EQUAL(GB_DOWNSHIFT_CLUTCH_ENGAGING, gearbox_state());
    sim_run(2);

    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_3, gearbox_state());
    TEST_ASSERT_EQUAL_MESSAGE(0, sim.shiftCount, "shifted against drive");
    TEST_ASSERT_EQUAL_UINT32(aborts + 1, gearbox_abort_count());
    TEST_ASSERT_EQUAL_UINT16(cal()->engagedCounts, clutch_commanded());
}

static void test_downshift_is_aborted_on_stale_feedback() {
    sim_reset(3);
    sim.servoLink = false;              // the arm moves, but nothing confirms it
    uint32_t aborts = gearbox_abort_count();

    gearbox_shift_down_pressed();
    sim_run(499);
    TEST_ASSERT_EQUAL_UINT16(cal()->disengagedCounts, sim.servoPos);    // it really is pulled
    sim_run(2);

    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_3, gearbox_state());
    TEST_ASSERT_EQUAL_MESSAGE(0, sim.shiftCount, "shifted on a position nobody confirmed");
    TEST_ASSERT_EQUAL_UINT32(aborts + 1, gearbox_abort_count());
}

// ---------------------------------------------------------------------------
// Neutral
// ---------------------------------------------------------------------------

static void test_neutral_from_first_is_a_half_upshift_without_ignition_cut() {
    sim_reset(1);
    gearbox_neutral_pressed();
    TEST_ASSERT_EQUAL(GB_NEUTRAL_UP_SHIFTING, gearbox_state());
    expect_shift(0, true, 40, 0, 0);
    sim_run(40);
    TEST_ASSERT_EQUAL(GB_IDLE_NEUTRAL, gearbox_state());
}

static void test_neutral_from_second_is_a_half_downshift() {
    sim_reset(2);
    gearbox_neutral_pressed();
    TEST_ASSERT_EQUAL(GB_NEUTRAL_DOWN_SHIFTING, gearbox_state());
    expect_shift(0, false, 40, 0, 0);
    sim_run(40);
    TEST_ASSERT_EQUAL(GB_IDLE_NEUTRAL, gearbox_state());
}

static void test_neutral_button_does_nothing_above_second() {
    sim_reset(4);
    uint32_t rejected = gearbox_rejected_count();
    gearbox_neutral_pressed();
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_4, gearbox_state());
    TEST_ASSERT_EQUAL(0, sim.shiftCount);
    TEST_ASSERT_EQUAL_UINT32(rejected + 1, gearbox_rejected_count());
}

static void test_leaving_neutral_needs_the_clutch_pulled() {
    sim_reset(0);
    gearbox_shift_down_pressed();
    TEST_ASSERT_EQUAL(GB_WAITING_FOR_CLUTCH_SHIFT_DOWN, gearbox_state());

    // Nobody pulls it: back to neutral, nothing sent.
    sim_run(199);
    TEST_ASSERT_EQUAL(GB_WAITING_FOR_CLUTCH_SHIFT_DOWN, gearbox_state());
    sim_run(1);
    TEST_ASSERT_EQUAL(GB_IDLE_NEUTRAL, gearbox_state());
    TEST_ASSERT_EQUAL(0, sim.shiftCount);

    // The rider pulls the paddle first, then asks: a full downshift into 1st.
    clutch_command_pull(1.0f);
    sim_run(100);
    gearbox_shift_down_pressed();
    sim_run(1);
    TEST_ASSERT_EQUAL(GB_NEUTRAL_DOWN_SHIFTING, gearbox_state());
    expect_shift(0, false, 150, 0, 1);

    // And up into 2nd the same way, with no ignition cut.
    sim_reset(0);
    clutch_command_pull(1.0f);
    sim_run(100);
    gearbox_shift_up_pressed();
    sim_run(1);
    TEST_ASSERT_EQUAL(GB_NEUTRAL_UP_SHIFTING, gearbox_state());
    expect_shift(0, true, 150, 0, 2);
}

// ---------------------------------------------------------------------------
// Stacked downshifts
// ---------------------------------------------------------------------------

// 5th, three quick down presses: must end in 2nd after exactly three shifts.
static void run_stack_5_to_2(uint16_t rearConfirmMs, bool expectBiteHold) {
    sim_reset(5);
    sim.rearConfirmMs = rearConfirmMs;
    uint32_t aborts = gearbox_abort_count();

    gearbox_shift_down_pressed();
    gearbox_shift_down_pressed();
    gearbox_shift_down_pressed();
    TEST_ASSERT_EQUAL(2, sim.stackCount);
    TEST_ASSERT_EQUAL_UINT8(3, sim.stacks[0]);
    TEST_ASSERT_EQUAL_UINT8(2, sim.stacks[1]);
    TEST_ASSERT_EQUAL_UINT8(2, gearbox_target_gear());

    bool sawBiteHold = false;
    for (int i = 0; i < 3000; i++) {
        sim_step();
        if (gearbox_state() == GB_DOWNSHIFT_TO_BITE_POINT) {
            sawBiteHold = true;
            TEST_ASSERT_EQUAL_UINT16(cal()->biteCounts, clutch_commanded());
        }
        // No shift may ever be sent with the clutch short of the trigger.
        if (gearbox_state() == GB_DOWNSHIFT_SHIFTING && i > 0)
            TEST_ASSERT_TRUE(sim.servoCmd == cal()->disengagedCounts);
    }

    TEST_ASSERT_EQUAL(3, sim.shiftCount);
    expect_shift(0, false, 150, 0, 4);
    expect_shift(1, false, 150, 0, 3);
    expect_shift(2, false, 150, 0, 2);
    TEST_ASSERT_EQUAL_UINT8(2, sim.gear);
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_2, gearbox_state());
    TEST_ASSERT_EQUAL_UINT8(0, gearbox_target_gear());
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sim.stacks[sim.stackCount - 1], "the display was never told the stack cleared");
    TEST_ASSERT_EQUAL_UINT16(cal()->engagedCounts, clutch_commanded());
    TEST_ASSERT_EQUAL_UINT32(aborts, gearbox_abort_count());
    TEST_ASSERT_EQUAL(expectBiteHold, sawBiteHold);
}

static void test_stack_when_the_rear_node_confirms_after_the_pulse() { run_stack_5_to_2(250, true); }
static void test_stack_when_the_rear_node_confirms_during_the_pulse() { run_stack_5_to_2(100, false); }

static void test_stack_never_goes_below_first() {
    sim_reset(2);
    for (int i = 0; i < 4; i++) gearbox_shift_down_pressed();
    TEST_ASSERT_EQUAL_UINT8(1, gearbox_target_gear());

    sim_run(2000);
    TEST_ASSERT_EQUAL(1, sim.shiftCount);
    TEST_ASSERT_EQUAL_UINT8(1, sim.gear);
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_1, gearbox_state());
}

static void test_an_up_press_cancels_the_stack_but_not_the_shift_in_flight() {
    sim_reset(5);
    gearbox_shift_down_pressed();
    gearbox_shift_down_pressed();
    TEST_ASSERT_EQUAL_UINT8(3, gearbox_target_gear());

    gearbox_shift_up_pressed();
    TEST_ASSERT_EQUAL_UINT8(0, gearbox_target_gear());

    sim_run(2000);
    TEST_ASSERT_EQUAL(1, sim.shiftCount);
    TEST_ASSERT_EQUAL_UINT8(4, sim.gear);
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_4, gearbox_state());
}

static void test_a_stack_is_dropped_when_the_downshift_aborts() {
    sim_reset(5);
    sim.servoRate = 0;
    gearbox_shift_down_pressed();
    gearbox_shift_down_pressed();
    sim_run(600);

    TEST_ASSERT_EQUAL(0, sim.shiftCount);
    TEST_ASSERT_EQUAL_UINT8(0, gearbox_target_gear());
    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_5, gearbox_state());
}

// ---------------------------------------------------------------------------
// Manual mode
// ---------------------------------------------------------------------------

static void test_manual_mode_sends_raw_pulses_one_at_a_time() {
    sim_reset(3);
    sim.manual = true;
    sim_run(1);

    gearbox_manual_shift(true);
    gearbox_manual_shift(true);         // ignored: one is still in progress
    TEST_ASSERT_EQUAL(1, sim.shiftCount);
    expect_shift(0, true, MANUAL_SHIFT_DURATION_MS, IGN_CUT_DEFAULT_MS, GEAR_UNKNOWN);

    sim_run(MANUAL_SHIFT_DURATION_MS);
    gearbox_manual_shift(false);
    TEST_ASSERT_EQUAL(2, sim.shiftCount);
    expect_shift(1, false, MANUAL_SHIFT_DURATION_MS, 0, GEAR_UNKNOWN);

    // Manual mode never touches the clutch.
    TEST_ASSERT_EQUAL_UINT16(cal()->engagedCounts, clutch_commanded());
}

static void test_leaving_manual_mode_resyncs_to_the_reported_gear() {
    sim_reset(3);
    gearbox_shift_down_pressed();       // leave a shift half done
    sim.manual = true;
    sim_run(1);
    sim.gear = 5;                       // the rider shifted by hand meanwhile
    sim_run(1);
    sim.manual = false;
    sim_run(1);

    TEST_ASSERT_EQUAL(GB_IDLE_GEAR_5, gearbox_state());
    TEST_ASSERT_EQUAL_UINT8(5, gearbox_gear());
}

// ---------------------------------------------------------------------------
// Shift timing log
// ---------------------------------------------------------------------------

static void test_a_confirmed_shift_is_logged_with_its_time() {
    sim_reset(2);
    sim.rpm = 9000;
    sim_run(1);
    sim.rearConfirmMs = 180;
    uint32_t total = shift_log_total(), failed = shift_log_failed();

    gearbox_shift_up_pressed();
    sim_run(400);

    TEST_ASSERT_EQUAL_UINT32(total + 1, shift_log_total());
    TEST_ASSERT_EQUAL_UINT32(failed, shift_log_failed());

    ShiftRecord r[SHIFT_LOG_RECENT];
    uint8_t n = shift_log_recent(r, SHIFT_LOG_RECENT);
    TEST_ASSERT_TRUE(n > 0);
    const ShiftRecord *last = &r[n - 1];
    TEST_ASSERT_EQUAL_UINT8(2, last->from);
    TEST_ASSERT_EQUAL_UINT8(3, last->to);
    TEST_ASSERT_EQUAL_UINT8(SHIFT_KIND_UP, last->kind);
    TEST_ASSERT_TRUE(last->ok);
    TEST_ASSERT_EQUAL_UINT16(9000, last->rpm);
    TEST_ASSERT_UINT16_WITHIN(1, 180, last->timeMs);

    TEST_ASSERT_EQUAL_STRING("SHIFT,2,3,9000,180,up,1", sim.lastLogLine);
}

static void test_a_shift_the_rear_node_never_confirms_is_logged_as_failed() {
    sim_reset(4);
    sim.rearResponds = false;
    uint32_t total = shift_log_total(), failed = shift_log_failed();

    gearbox_shift_up_pressed();
    sim_run(SHIFT_LOG_TIMEOUT_MS - 1);
    TEST_ASSERT_EQUAL_UINT32(total, shift_log_total());
    sim_run(2);

    TEST_ASSERT_EQUAL_UINT32(total + 1, shift_log_total());
    TEST_ASSERT_EQUAL_UINT32(failed + 1, shift_log_failed());
    TEST_ASSERT_EQUAL_STRING("SHIFT,4,5,0,1000,up,0", sim.lastLogLine);
}

static void test_a_downshift_time_includes_the_clutch_pull() {
    sim_reset(3);
    sim.rearConfirmMs = 100;
    gearbox_shift_down_pressed();
    sim_run(400);

    ShiftRecord r[SHIFT_LOG_RECENT];
    uint8_t n = shift_log_recent(r, SHIFT_LOG_RECENT);
    const ShiftRecord *last = &r[n - 1];
    TEST_ASSERT_EQUAL_UINT8(SHIFT_KIND_DOWN, last->kind);
    TEST_ASSERT_TRUE(last->ok);
    // Pull to the trigger at 200 counts/ms is about 52 ms, then 100 to confirm.
    TEST_ASSERT_UINT16_WITHIN(5, 152, last->timeMs);
}

// The ESP32 logger called anything over 300 ms failed and wrote 300.
static void test_a_slow_shift_is_logged_with_its_real_time_not_as_failed() {
    sim_reset(4);
    sim.servoRate = 40;                 // a lazy servo: about 260 ms to the trigger
    sim.rearConfirmMs = 200;
    uint32_t failed = shift_log_failed();

    gearbox_shift_down_pressed();
    sim_run(900);

    ShiftRecord r[SHIFT_LOG_RECENT];
    uint8_t n = shift_log_recent(r, SHIFT_LOG_RECENT);
    const ShiftRecord *last = &r[n - 1];
    TEST_ASSERT_TRUE(last->ok);
    TEST_ASSERT_UINT16_WITHIN(5, 460, last->timeMs);
    TEST_ASSERT_EQUAL_UINT32(failed, shift_log_failed());
}

static void test_neutral_shifts_are_logged_as_neutral() {
    sim_reset(1);
    sim.rearConfirmMs = 60;
    gearbox_neutral_pressed();
    sim_run(200);
    TEST_ASSERT_EQUAL_STRING("SHIFT,1,0,0,60,neutral,1", sim.lastLogLine);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_upshift_sends_one_pulse_and_completes_on_time);
    RUN_TEST(test_upshift_timing_follows_the_calibration);
    RUN_TEST(test_no_upshift_from_sixth_and_no_downshift_from_first);
    RUN_TEST(test_downshift_waits_for_the_clutch_before_shifting);
    RUN_TEST(test_downshift_is_aborted_when_the_servo_never_gets_there);
    RUN_TEST(test_downshift_is_aborted_on_stale_feedback);
    RUN_TEST(test_neutral_from_first_is_a_half_upshift_without_ignition_cut);
    RUN_TEST(test_neutral_from_second_is_a_half_downshift);
    RUN_TEST(test_neutral_button_does_nothing_above_second);
    RUN_TEST(test_leaving_neutral_needs_the_clutch_pulled);
    RUN_TEST(test_stack_when_the_rear_node_confirms_after_the_pulse);
    RUN_TEST(test_stack_when_the_rear_node_confirms_during_the_pulse);
    RUN_TEST(test_stack_never_goes_below_first);
    RUN_TEST(test_an_up_press_cancels_the_stack_but_not_the_shift_in_flight);
    RUN_TEST(test_a_stack_is_dropped_when_the_downshift_aborts);
    RUN_TEST(test_manual_mode_sends_raw_pulses_one_at_a_time);
    RUN_TEST(test_leaving_manual_mode_resyncs_to_the_reported_gear);
    RUN_TEST(test_a_confirmed_shift_is_logged_with_its_time);
    RUN_TEST(test_a_shift_the_rear_node_never_confirms_is_logged_as_failed);
    RUN_TEST(test_a_downshift_time_includes_the_clutch_pull);
    RUN_TEST(test_a_slow_shift_is_logged_with_its_real_time_not_as_failed);
    RUN_TEST(test_neutral_shifts_are_logged_as_neutral);
    return UNITY_END();
}
