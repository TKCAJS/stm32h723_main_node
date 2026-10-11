// Calibration, clutch predicates and the paddle -> clutch maps.
// Run with: pio test -e native

#include <unity.h>
#include "sim.h"

void setUp() { sim_reset(3); }
void tearDown() {}

static const ClutchCal *cal() { return clutch_cal(); }

static bool set(const char *field, long v) {
    const char *err = "";
    return clutch_cal_set(field, v, &err);
}

// Counts fall as the clutch is pulled — the arm mounted the other way round.
// Every single edit is validated against the others, so the two ends can only
// be swapped in an order where each step is itself a valid calibration.
static void reverse_servo_direction() {
    uint16_t engaged = cal()->engagedCounts;
    TEST_ASSERT_TRUE(set("bite", engaged));
    TEST_ASSERT_TRUE(set("trigger", engaged));
    TEST_ASSERT_TRUE(set("disengaged", engaged - 6000));
    TEST_ASSERT_TRUE(set("trigger", engaged - 4000));
    TEST_ASSERT_TRUE(set("bite", engaged - 1000));
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

static void test_shift_times_must_stay_below_the_shift_timeout() {
    const char *names[] = { "shift_up_ms", "shift_down_ms", "neutral_up_ms", "neutral_down_ms" };
    for (const char *n : names) {
        TEST_ASSERT_FALSE_MESSAGE(set(n, 0), n);
        TEST_ASSERT_TRUE_MESSAGE(set(n, 1), n);
        TEST_ASSERT_TRUE_MESSAGE(set(n, GEARBOX_SHIFT_TIMEOUT_MS - 1), n);
        TEST_ASSERT_FALSE_MESSAGE(set(n, GEARBOX_SHIFT_TIMEOUT_MS), n);
    }
    TEST_ASSERT_TRUE(set("ign_cut_ms", 0));
    TEST_ASSERT_FALSE(set("ign_cut_ms", GEARBOX_SHIFT_TIMEOUT_MS));
}

static void test_a_rejected_edit_changes_nothing() {
    ClutchCal before = *cal();
    TEST_ASSERT_FALSE(set("bite", before.triggerCounts + 100));     // wrong side of the trigger
    TEST_ASSERT_FALSE(set("disengaged", before.engagedCounts));     // zero travel
    TEST_ASSERT_FALSE(set("speed", 1281));
    TEST_ASSERT_FALSE(set("curve", PADDLE_CURVE_COUNT));
    TEST_ASSERT_FALSE(set("pw_paddle_start", before.pwPaddleEnd));
    TEST_ASSERT_FALSE(set("no_such_field", 1));
    TEST_ASSERT_EQUAL_MEMORY(&before, cal(), sizeof(ClutchCal));
}

static void test_every_named_field_reads_back_what_was_set() {
    char names[512];
    strncpy(names, clutch_cal_field_names(), sizeof(names) - 1);
    names[sizeof(names) - 1] = '\0';
    int count = 0;
    for (char *f = strtok(names, " "); f; f = strtok(NULL, " "), count++) {
        long v = -1;
        TEST_ASSERT_TRUE_MESSAGE(clutch_cal_get(f, &v), f);
        TEST_ASSERT_TRUE_MESSAGE(set(f, v), f);                     // its own value is always valid
    }
    TEST_ASSERT_EQUAL(21, count);
}

static void test_save_and_load_round_trip_including_the_shift_times() {
    TEST_ASSERT_TRUE(set("shift_up_ms", 123));
    TEST_ASSERT_TRUE(set("curve", PADDLE_PIECEWISE));
    TEST_ASSERT_TRUE(clutch_cal_save());

    clutch_cal_defaults();
    TEST_ASSERT_EQUAL_UINT16(150, cal()->shiftUpMs);
    TEST_ASSERT_TRUE(clutch_cal_load());
    TEST_ASSERT_EQUAL_UINT16(123, cal()->shiftUpMs);
    TEST_ASSERT_EQUAL_UINT16(PADDLE_PIECEWISE, cal()->curve);
}

static void test_a_corrupt_or_old_blob_is_refused() {
    TEST_ASSERT_FALSE_MESSAGE(clutch_cal_load(), "loaded from empty flash");

    TEST_ASSERT_TRUE(clutch_cal_save());
    sim.flash[offsetof(ClutchCal, biteCounts)] ^= 0x01;
    TEST_ASSERT_FALSE_MESSAGE(clutch_cal_load(), "loaded a blob with a bad CRC");

    TEST_ASSERT_TRUE(clutch_cal_save());
    sim.flashLen -= 2;                                              // an older, shorter layout
    TEST_ASSERT_FALSE_MESSAGE(clutch_cal_load(), "loaded a blob of the wrong size");
}

// ---------------------------------------------------------------------------
// Clutch predicates
// ---------------------------------------------------------------------------

static void test_clutch_predicates_follow_the_position() {
    sim.servoPos = cal()->engagedCounts;
    TEST_ASSERT_FALSE(clutch_disengaged());
    TEST_ASSERT_FALSE(clutch_just_engaged());
    TEST_ASSERT_TRUE(clutch_at_bite());                             // on its engaged side counts

    sim.servoPos = cal()->biteCounts + 10;
    TEST_ASSERT_FALSE(clutch_disengaged());
    TEST_ASSERT_TRUE(clutch_just_engaged());

    sim.servoPos = cal()->triggerCounts;
    TEST_ASSERT_TRUE(clutch_disengaged());
    TEST_ASSERT_FALSE(clutch_just_engaged());
    TEST_ASSERT_FALSE(clutch_at_bite());
}

static void test_clutch_predicates_work_with_the_servo_reversed() {
    reverse_servo_direction();

    sim.servoPos = cal()->engagedCounts;
    TEST_ASSERT_FALSE(clutch_disengaged());

    sim.servoPos = cal()->biteCounts - 10;
    TEST_ASSERT_TRUE(clutch_just_engaged());

    sim.servoPos = cal()->triggerCounts - 1;
    TEST_ASSERT_TRUE(clutch_disengaged());
    TEST_ASSERT_TRUE(clutch_pull_fraction() > 0.6f);

    clutch_command_counts(0);                                       // outside travel: clamped
    TEST_ASSERT_EQUAL_UINT16(cal()->disengagedCounts, clutch_commanded());
}

static void test_stale_feedback_never_reads_as_pulled() {
    sim.servoPos = cal()->disengagedCounts;
    sim.servoLink = false;
    TEST_ASSERT_FALSE(clutch_disengaged());
    TEST_ASSERT_FALSE(clutch_just_engaged());
    TEST_ASSERT_FALSE(clutch_at_bite());
}

static void test_commands_are_clamped_to_the_calibrated_travel() {
    clutch_command_counts(0);
    TEST_ASSERT_EQUAL_UINT16(cal()->engagedCounts, sim.servoCmd);
    clutch_command_counts(0x7FFF);
    TEST_ASSERT_EQUAL_UINT16(cal()->disengagedCounts, sim.servoCmd);
    clutch_command_pull(2.0f);
    TEST_ASSERT_EQUAL_UINT16(cal()->disengagedCounts, sim.servoCmd);
}

// ---------------------------------------------------------------------------
// Halls -> paddle position
// ---------------------------------------------------------------------------

static void test_paddle_position_from_the_halls() {
    const ClutchCal *c = cal();
    bool fault = true;

    TEST_ASSERT_EQUAL_UINT16(0, paddle_from_halls(c, c->hallLeftIdle, c->hallRightIdle, &fault));
    TEST_ASSERT_FALSE(fault);

    // Noise at rest stays idle.
    TEST_ASSERT_EQUAL_UINT16(0, paddle_from_halls(c, c->hallLeftIdle + HALL_IDLE_DEADBAND, c->hallRightIdle, NULL));

    TEST_ASSERT_EQUAL_UINT16(PADDLE_FULL, paddle_from_halls(c, c->hallLeftPulled, c->hallRightIdle, NULL));
    // Either paddle works the clutch; the further-pulled one wins.
    TEST_ASSERT_EQUAL_UINT16(PADDLE_FULL, paddle_from_halls(c, c->hallLeftIdle, c->hallRightPulled, NULL));

    uint16_t half = paddle_from_halls(c, (uint16_t)((c->hallLeftIdle + c->hallLeftPulled) / 2), c->hallRightIdle, NULL);
    TEST_ASSERT_UINT16_WITHIN(150, PADDLE_FULL / 2, half);
}

static void test_a_disconnected_hall_is_idle_and_counted_never_pulled() {
    const ClutchCal *c = cal();
    bool fault = false;
    TEST_ASSERT_EQUAL_UINT16(0, paddle_from_halls(c, 65535, c->hallRightIdle, &fault));
    TEST_ASSERT_TRUE(fault);
    fault = false;
    TEST_ASSERT_EQUAL_UINT16(0, paddle_from_halls(c, 0, c->hallRightIdle, &fault));
    TEST_ASSERT_TRUE(fault);

    uint32_t faults = paddle_fault_count();
    paddle_tick(65535, c->hallRightIdle, PADDLE_CURVED);
    TEST_ASSERT_EQUAL_UINT32(faults + 1, paddle_fault_count());
    TEST_ASSERT_EQUAL_UINT16(c->engagedCounts, clutch_commanded());
}

static void test_a_paddle_switched_off_is_ignored() {
    TEST_ASSERT_TRUE(set("paddle_r_pulled", cal()->hallRightIdle));
    bool fault = true;
    TEST_ASSERT_EQUAL_UINT16(0, paddle_from_halls(cal(), cal()->hallLeftIdle, 60000, &fault));
    TEST_ASSERT_FALSE(fault);
    // Both off would leave the clutch with no way to move.
    TEST_ASSERT_FALSE(set("paddle_l_pulled", cal()->hallLeftIdle));
}

// ---------------------------------------------------------------------------
// Paddle position -> clutch counts
// ---------------------------------------------------------------------------

// Every curve must run end to end, stay inside the travel and never reverse.
// maxStep bounds the jump for one unit (0.01 %) of paddle travel.
static void check_curves(int maxStep) {
    const ClutchCal *c = cal();
    bool rise = c->disengagedCounts > c->engagedCounts;
    uint16_t lo = rise ? c->engagedCounts : c->disengagedCounts;
    uint16_t hi = rise ? c->disengagedCounts : c->engagedCounts;

    for (int curve = 0; curve < PADDLE_CURVE_COUNT; curve++) {
        TEST_ASSERT_TRUE(set("curve", curve));
        const char *name = paddle_curve_name((uint16_t)curve);

        TEST_ASSERT_EQUAL_UINT16_MESSAGE(c->engagedCounts, paddle_to_counts(c, 0), name);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(c->disengagedCounts, paddle_to_counts(c, PADDLE_FULL), name);

        uint16_t prev = paddle_to_counts(c, 0);
        for (int p = 1; p <= PADDLE_FULL; p++) {
            uint16_t now = paddle_to_counts(c, (uint16_t)p);
            TEST_ASSERT_TRUE_MESSAGE(now >= lo && now <= hi, name);
            TEST_ASSERT_TRUE_MESSAGE(rise ? now >= prev : now <= prev, name);
            TEST_ASSERT_TRUE_MESSAGE(abs((int)now - (int)prev) <= maxStep, name);
            prev = now;
        }
    }
}

// At the factory settings nothing jumps: the steepest piecewise segment is
// about 12 counts for one unit of paddle.
static void test_every_curve_is_monotonic_and_stays_in_travel() { check_curves(16); }

static void test_every_curve_works_with_the_servo_reversed() {
    reverse_servo_direction();
    check_curves(16);
}

// The strength limits are allowed to be abrupt — power at 0.1 is close to a
// step off idle by design — but must still run end to end without reversing.
static void test_curves_hold_at_the_strength_limits() {
    TEST_ASSERT_TRUE(set("strength", 10));
    check_curves(0x7FFF);
    TEST_ASSERT_TRUE(set("strength", 500));
    check_curves(0x7FFF);
}

static void test_piecewise_maps_the_paddle_zone_onto_the_clutch_zone() {
    TEST_ASSERT_TRUE(set("curve", PADDLE_PIECEWISE));
    TEST_ASSERT_TRUE(set("pw_blend", 0));
    const ClutchCal *c = cal();
    int32_t span = (int32_t)c->disengagedCounts - c->engagedCounts;
    uint16_t zoneStart = (uint16_t)(c->engagedCounts + span * c->pwClutchStart / PADDLE_FULL);
    uint16_t zoneEnd   = (uint16_t)(c->engagedCounts + span * c->pwClutchEnd   / PADDLE_FULL);

    TEST_ASSERT_UINT16_WITHIN(1, zoneStart, paddle_to_counts(c, c->pwPaddleStart));
    TEST_ASSERT_UINT16_WITHIN(1, zoneEnd,   paddle_to_counts(c, c->pwPaddleEnd));

    // Most of the paddle's travel is spent inside the bite zone.
    TEST_ASSERT_TRUE((c->pwPaddleEnd - c->pwPaddleStart) > PADDLE_FULL / 2);
    TEST_ASSERT_TRUE((zoneEnd - zoneStart) < span / 4);
}

// ---------------------------------------------------------------------------
// Who drives the clutch
// ---------------------------------------------------------------------------

static void test_the_paddle_only_drives_the_clutch_when_it_is_allowed_to() {
    const ClutchCal *c = cal();
    uint16_t pulled = c->hallLeftPulled, idleR = c->hallRightIdle;

    paddle_tick(pulled, idleR, PADDLE_OFF);                         // the gearbox owns it
    TEST_ASSERT_EQUAL_UINT16(c->engagedCounts, clutch_commanded());
    TEST_ASSERT_EQUAL_UINT16(PADDLE_FULL, paddle_position());       // still measured

    paddle_set_override(true);                                      // the console owns it
    paddle_tick(pulled, idleR, PADDLE_CURVED);
    TEST_ASSERT_EQUAL_UINT16(c->engagedCounts, clutch_commanded());

    paddle_set_override(false);
    paddle_tick(pulled, idleR, PADDLE_CURVED);
    TEST_ASSERT_EQUAL_UINT16(c->disengagedCounts, clutch_commanded());

    // The gearbox releases on its way back to idle; the paddle must win the
    // clutch back even though it has not moved.
    clutch_release();
    paddle_tick(pulled, idleR, PADDLE_CURVED);
    TEST_ASSERT_EQUAL_UINT16(c->disengagedCounts, clutch_commanded());
}

static void test_manual_mode_maps_the_paddle_straight_onto_the_travel() {
    const ClutchCal *c = cal();
    TEST_ASSERT_TRUE(set("curve", PADDLE_PIECEWISE));
    uint16_t midHall = (uint16_t)((c->hallLeftIdle + c->hallLeftPulled) / 2);

    paddle_tick(midHall, c->hallRightIdle, PADDLE_DIRECT);
    int32_t span = (int32_t)c->disengagedCounts - c->engagedCounts;
    uint16_t linear = (uint16_t)(c->engagedCounts + (int64_t)span * paddle_position() / PADDLE_FULL);
    TEST_ASSERT_UINT16_WITHIN(1, linear, clutch_commanded());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_shift_times_must_stay_below_the_shift_timeout);
    RUN_TEST(test_a_rejected_edit_changes_nothing);
    RUN_TEST(test_every_named_field_reads_back_what_was_set);
    RUN_TEST(test_save_and_load_round_trip_including_the_shift_times);
    RUN_TEST(test_a_corrupt_or_old_blob_is_refused);
    RUN_TEST(test_clutch_predicates_follow_the_position);
    RUN_TEST(test_clutch_predicates_work_with_the_servo_reversed);
    RUN_TEST(test_stale_feedback_never_reads_as_pulled);
    RUN_TEST(test_commands_are_clamped_to_the_calibrated_travel);
    RUN_TEST(test_paddle_position_from_the_halls);
    RUN_TEST(test_a_disconnected_hall_is_idle_and_counted_never_pulled);
    RUN_TEST(test_a_paddle_switched_off_is_ignored);
    RUN_TEST(test_every_curve_is_monotonic_and_stays_in_travel);
    RUN_TEST(test_every_curve_works_with_the_servo_reversed);
    RUN_TEST(test_curves_hold_at_the_strength_limits);
    RUN_TEST(test_piecewise_maps_the_paddle_zone_onto_the_clutch_zone);
    RUN_TEST(test_the_paddle_only_drives_the_clutch_when_it_is_allowed_to);
    RUN_TEST(test_manual_mode_maps_the_paddle_straight_onto_the_travel);
    return UNITY_END();
}
