/*
 * Hand-computed tests of the D-103 class-B accumulators (derived_ride.c), driven directly:
 * flag seconds (stale cap, the last row adds nothing, invalid rows), the waiting-row rule of the
 * grid end, max / first / last, counts, warm-up and its flags, thermal cycles and the fan-on
 * step (gap too long, not at rest at the first sample, the coolant at the second sample).
 */
#include "features/derived/derived_ride.h"
#include "vehicle_cl250.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <string.h>
#include <unity.h>

static derived_ride_acc_t acc;

void setUp(void)
{
    (void)memset(&acc, 0, sizeof acc);
}
void tearDown(void) {}

static derived_out_t fin(uint32_t sig)
{
    derived_out_t o;
    (void)memset(&o, 0, sizeof o);
    derived_ride_finalize(&acc, sig, &o);
    return o;
}

static void expect_val(uint32_t sig, float v)
{
    const derived_out_t o = fin(sig);
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, v, o.value);
}

static void expect_flag(uint32_t sig, derived_flag_t f)
{
    const derived_out_t o = fin(sig);
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, o.status);
    TEST_ASSERT_EQUAL_INT(f, o.flag);
}

static void row(uint32_t sig, uint32_t t, bool valid, float v)
{
    derived_ride_class_a(&acc, sig, t, valid, v);
}

static void confirm_all(uint32_t sig)
{
    derived_ride_advance(&acc, sig, 0x00FFFFFFu);
}

static void did(uint32_t idx, float v, uint32_t t)
{
    derived_ride_sample(&acc, idx, v, t);
}

/* ---- flag seconds ------------------------------------------------------------------------------ */

static void test_flag_seconds_cap_each_row_at_the_signals_stale_limit(void)
{
    const uint32_t sig = VEHICLE_CL250_DERIVED_OVER_REV;
    const uint32_t limit = vehicle_cl250_derived[sig].stale_after_ms;
    row(sig, 0u, true, 1.0f);                  /* until the next row: 110 ms */
    row(sig, 110u, true, 1.0f);                /* a gap of limit + 5000: capped at the limit */
    row(sig, 110u + limit + 5000u, true, 0.0f); /* value 0: adds nothing */
    row(sig, 220u + limit + 5000u, false, 0.0f);
    row(sig, 330u + limit + 5000u, true, 1.0f); /* the last row adds nothing */
    confirm_all(sig);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_OVER_REV_TIME, (110.0f + (float)limit) / 1000.0f);
}

static void test_flag_seconds_without_a_valid_row_is_invalid_input(void)
{
    const uint32_t sig = VEHICLE_CL250_DERIVED_LUGGING;
    row(sig, 0u, false, 0.0f);
    row(sig, 110u, false, 0.0f);
    confirm_all(sig);
    expect_flag(VEHICLE_CL250_DERIVED_RIDE_LUGGING_TIME, DERIVED_FLAG_INVALID_INPUT);
}

static void test_flag_seconds_zero_when_valid_but_never_set(void)
{
    const uint32_t sig = VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE;
    row(sig, 0u, true, 0.0f);
    row(sig, 110u, true, 0.0f);
    confirm_all(sig);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_COLD_ABUSE_TIME, 0.0f);
}

static void test_rows_after_the_grid_end_are_dropped(void)
{
    const uint32_t sig = VEHICLE_CL250_DERIVED_OVER_REV;
    row(sig, 0u, true, 1.0f);
    row(sig, 100u, true, 1.0f);
    row(sig, 200u, true, 1.0f);
    row(sig, 300u, true, 1.0f);
    derived_ride_advance(&acc, sig, 200u); /* every input has a sample up to 200 ms */
    /* the row at 300 never confirms: 0..100 and 100..200 count, the last confirmed row adds nothing */
    expect_val(VEHICLE_CL250_DERIVED_RIDE_OVER_REV_TIME, 0.2f);
}

static void test_the_waiting_ring_confirms_the_oldest_row_when_full(void)
{
    const uint32_t sig = VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA;
    const uint32_t cap = derived_ride_tail_cap(sig);
    TEST_ASSERT_EQUAL_UINT32(DERIVED_RIDE_TAIL_CAP_HEAT_SOAK, cap);
    for (uint32_t i = 0u; i < (cap + 3u); i++) {
        row(sig, i * 110u, true, (i == 0u) ? 9.0f : 1.0f);
    }
    /* nothing was confirmed by a bound, yet the first three rows left the ring (bounded) */
    expect_val(VEHICLE_CL250_DERIVED_RIDE_HEAT_SOAK_MAX, 9.0f);
    derived_ride_advance(&acc, sig, 0u); /* NULL-safe and wrap-safe paths */
    derived_ride_advance(NULL, sig, 0u);
}

/* ---- max / first / last / counts ------------------------------------------------------------------ */

static void test_max_first_last(void)
{
    expect_flag(VEHICLE_CL250_DERIVED_RIDE_HEAT_SOAK_MAX, DERIVED_FLAG_INVALID_INPUT);
    row(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, 0u, true, 3.0f);
    row(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, 110u, true, 7.0f);
    row(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, 220u, false, 0.0f);
    row(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, 330u, true, 5.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_STD, 0u, true, 20.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_STD, 110u, true, 40.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_STD, 220u, true, 30.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, 0u, true, -4.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, 110u, false, 0.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, 220u, true, 12.0f);
    row(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, 330u, false, 0.0f);
    row(VEHICLE_CL250_DERIVED_CRANKING_DIP, 500u, true, 2.5f);
    row(VEHICLE_CL250_DERIVED_CRANKING_DIP, 900u, true, 3.5f);
    row(VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE, 100u, true, 12.9f);
    row(VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE, 800u, true, 12.4f);
    confirm_all(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA);
    confirm_all(VEHICLE_CL250_DERIVED_IDLE_RPM_STD);
    confirm_all(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_HEAT_SOAK_MAX, 7.0f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_STD, 40.0f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_DRIFT, 12.0f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_CRANKING_DIP, 2.5f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_REST_VOLTAGE, 12.9f);
}

static void test_missing_first_and_last_are_invalid_input(void)
{
    expect_flag(VEHICLE_CL250_DERIVED_RIDE_CRANKING_DIP, DERIVED_FLAG_INVALID_INPUT);
    expect_flag(VEHICLE_CL250_DERIVED_RIDE_REST_VOLTAGE, DERIVED_FLAG_INVALID_INPUT);
    expect_flag(VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_STD, DERIVED_FLAG_INVALID_INPUT);
    expect_flag(VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_DRIFT, DERIVED_FLAG_INVALID_INPUT);
}

static void test_counts_are_zero_without_rows_and_ignore_invalid_rows(void)
{
    expect_val(VEHICLE_CL250_DERIVED_RIDE_SHIFT_COUNT, 0.0f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_LAUNCH_COUNT, 0.0f);
    row(VEHICLE_CL250_DERIVED_SHIFT_EVENT, 100u, true, 0.3f);
    row(VEHICLE_CL250_DERIVED_SHIFT_EVENT, 200u, false, 0.0f);
    row(VEHICLE_CL250_DERIVED_SHIFT_EVENT, 300u, true, 0.3f);
    row(VEHICLE_CL250_DERIVED_LAUNCH_EVENT, 400u, true, 0.8f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_SHIFT_COUNT, 2.0f);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_LAUNCH_COUNT, 1.0f);
}

/* ---- warm-up and thermal cycles ------------------------------------------------------------------- */

#define COOLANT (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP
#define RPM (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED
#define SPEED (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED
#define BATT (uint32_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE
#define WARMUP VEHICLE_CL250_DERIVED_RIDE_WARMUP_TIME

static void test_warmup_time_from_the_first_rpm_to_the_first_operating_coolant(void)
{
    did(RPM, 0.0f, 0u);     /* not running yet */
    did(COOLANT, 40.0f, 500u);
    did(RPM, 1200.0f, 1000u); /* the engine start */
    did(COOLANT, 79.9f, 3000u);
    did(COOLANT, 80.0f, 6000u); /* operating (>= 80) */
    did(COOLANT, 90.0f, 9000u);
    expect_val(WARMUP, 5.0f);
}

static void test_warmup_flags(void)
{
    expect_flag(WARMUP, DERIVED_FLAG_INVALID_INPUT); /* nothing at all */
    did(COOLANT, 40.0f, 0u);
    expect_flag(WARMUP, DERIVED_FLAG_INVALID_INPUT); /* no rpm above 0 */
    did(RPM, 1000.0f, 100u);
    expect_flag(WARMUP, DERIVED_FLAG_NEVER_WARM);
}

static void test_warmup_starts_warm(void)
{
    did(COOLANT, 85.0f, 0u); /* the first coolant sample is already warm */
    did(RPM, 1000.0f, 500u);
    expect_flag(WARMUP, DERIVED_FLAG_STARTS_WARM);
}

static void test_warmup_operating_before_the_engine_start_is_starts_warm(void)
{
    did(COOLANT, 40.0f, 0u);
    did(COOLANT, 82.0f, 1000u); /* operating before the first rpm above 0 */
    did(RPM, 900.0f, 2000u);
    expect_flag(WARMUP, DERIVED_FLAG_STARTS_WARM);
}

static void test_thermal_cycles(void)
{
    expect_val(VEHICLE_CL250_DERIVED_RIDE_THERMAL_CYCLE_COUNT, 0.0f); /* no sample: still a value */
    did(COOLANT, 85.0f, 0u);  /* starts warm: no cycle until it has been cold */
    did(COOLANT, 90.0f, 10u);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_THERMAL_CYCLE_COUNT, 0.0f);
    did(COOLANT, 49.9f, 20u); /* below cold (50) */
    did(COOLANT, 60.0f, 30u); /* between: the state is kept */
    did(COOLANT, 80.0f, 40u); /* cycle 1 */
    did(COOLANT, 95.0f, 50u); /* hot again, not a new cycle */
    did(COOLANT, 30.0f, 60u);
    did(COOLANT, 70.0f, 70u);
    expect_val(VEHICLE_CL250_DERIVED_RIDE_THERMAL_CYCLE_COUNT, 1.0f);
    did(COOLANT, 81.0f, 80u); /* cycle 2 */
    expect_val(VEHICLE_CL250_DERIVED_RIDE_THERMAL_CYCLE_COUNT, 2.0f);
}

/* ---- fan-on temperature ------------------------------------------------------------------------------ */

#define FAN VEHICLE_CL250_DERIVED_RIDE_FAN_ON_TEMPERATURE

/* The machine at rest at idle with a warm coolant, all fresh at `t`. */
static void idle_at(uint32_t t, float coolant)
{
    did(SPEED, 0.0f, t);
    did(RPM, 1300.0f, t);
    did(COOLANT, coolant, t);
}

static void battery(float v, uint32_t t)
{
    did(BATT, v, t);
    derived_ride_settle(&acc);
}

static void test_fan_on_temperature_is_the_coolant_at_the_second_sample(void)
{
    expect_flag(FAN, DERIVED_FLAG_NO_FAN_STEP);
    idle_at(1000u, 93.0f);
    battery(13.80f, 1000u);
    idle_at(1500u, 95.0f); /* the coolant fresh at the second battery sample */
    battery(13.60f, 1500u); /* a 0.20 V drop >= 0.15 */
    expect_val(FAN, 95.0f);
    idle_at(2000u, 97.0f);
    battery(13.20f, 2000u); /* a later step is not the first one */
    expect_val(FAN, 95.0f);
}

static void test_fan_step_below_the_voltage_step_is_no_step(void)
{
    idle_at(1000u, 93.0f);
    battery(13.80f, 1000u);
    idle_at(1500u, 95.0f);
    battery(13.70f, 1500u); /* 0.10 V < 0.15 */
    expect_flag(FAN, DERIVED_FLAG_NO_FAN_STEP);
}

static void test_fan_step_with_a_gap_longer_than_the_battery_limit_is_no_step(void)
{
    const uint32_t limit = vehicle_cl250_dids[VEHICLE_CL250_IDX_BATTERY_VOLTAGE].stale_after_ms;
    idle_at(1000u, 93.0f);
    battery(13.80f, 1000u);
    idle_at(1000u + limit + 1u, 95.0f);
    battery(13.20f, 1000u + limit + 1u);
    expect_flag(FAN, DERIVED_FLAG_NO_FAN_STEP);
}

static void test_fan_step_with_a_gap_exactly_the_battery_limit_counts(void)
{
    const uint32_t limit = vehicle_cl250_dids[VEHICLE_CL250_IDX_BATTERY_VOLTAGE].stale_after_ms;
    idle_at(1000u, 93.0f);
    battery(13.80f, 1000u);
    idle_at(1000u + limit, 95.0f);
    battery(13.20f, 1000u + limit);
    expect_val(FAN, 95.0f);
}

static void test_fan_step_needs_rest_at_both_samples(void)
{
    did(SPEED, 20.0f, 1000u); /* moving at the first sample */
    did(RPM, 1300.0f, 1000u);
    did(COOLANT, 93.0f, 1000u);
    battery(13.80f, 1000u);
    idle_at(1500u, 95.0f);
    battery(13.60f, 1500u);
    expect_flag(FAN, DERIVED_FLAG_NO_FAN_STEP);
}

static void test_fan_step_needs_a_warm_coolant_and_the_idle_band(void)
{
    idle_at(1000u, 89.0f); /* coolant must be above 90 */
    battery(13.80f, 1000u);
    idle_at(1500u, 95.0f);
    battery(13.60f, 1500u);
    expect_flag(FAN, DERIVED_FLAG_NO_FAN_STEP);
    idle_at(3000u, 95.0f);
    battery(13.80f, 3000u);
    did(SPEED, 0.0f, 3500u);
    did(RPM, 2500.0f, 3500u); /* above the idle band */
    did(COOLANT, 95.0f, 3500u);
    battery(13.60f, 3500u);
    expect_flag(FAN, DERIVED_FLAG_NO_FAN_STEP);
}

static void test_fan_step_uses_the_inputs_fresh_at_the_battery_time(void)
{
    idle_at(1000u, 93.0f);
    battery(13.80f, 1000u);
    /* the battery sample is ingested first, a coolant sample of the same step is later in time */
    did(SPEED, 0.0f, 1500u);
    did(RPM, 1300.0f, 1500u);
    did(COOLANT, 95.0f, 1500u);
    did(BATT, 13.60f, 1500u); /* index order: a battery sample with the others already in */
    did(COOLANT, 99.0f, 1600u); /* newer than the pending battery sample: it settles first */
    expect_val(FAN, 95.0f);
}

static void test_waiting_row_capacities_cover_the_gen_table(void)
{
    const struct {
        uint32_t sig;
        uint32_t need;
    } want[] = {
        {VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, 139u}, {VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE, 57u},
        {VEHICLE_CL250_DERIVED_OVER_REV, 5u},          {VEHICLE_CL250_DERIVED_LUGGING, 57u},
        {VEHICLE_CL250_DERIVED_IDLE_RPM_STD, 24u},     {VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, 24u},
    };
    uint32_t total = 0u;
    for (uint32_t i = 0u; i < (sizeof want / sizeof want[0]); i++) {
        TEST_ASSERT_EQUAL_UINT32(want[i].need, derived_ride_tail_need(want[i].sig));
        TEST_ASSERT_TRUE(derived_ride_tail_fits(want[i].sig));
        total += derived_ride_tail_cap(want[i].sig);
    }
    TEST_ASSERT_EQUAL_UINT32(DERIVED_RIDE_TAIL_POOL, total);
    TEST_ASSERT_EQUAL_UINT32(0u, derived_ride_tail_cap(VEHICLE_CL250_DERIVED_SHIFT_EVENT));
    TEST_ASSERT_EQUAL_UINT32(0u, derived_ride_tail_need(VEHICLE_CL250_DERIVED_SHIFT_EVENT));
    TEST_ASSERT_TRUE(derived_ride_tail_fits(VEHICLE_CL250_DERIVED_SHIFT_EVENT));
}

static void test_calls_with_null_are_ignored(void)
{
    derived_ride_sample(NULL, COOLANT, 1.0f, 0u);
    derived_ride_class_a(NULL, VEHICLE_CL250_DERIVED_OVER_REV, 0u, true, 1.0f);
    derived_ride_settle(NULL);
    TEST_ASSERT_TRUE(derived_ride_is_tail_signal(VEHICLE_CL250_DERIVED_LUGGING));
    TEST_ASSERT_FALSE(derived_ride_is_tail_signal(VEHICLE_CL250_DERIVED_SHIFT_EVENT));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_flag_seconds_cap_each_row_at_the_signals_stale_limit);
    RUN_TEST(test_flag_seconds_without_a_valid_row_is_invalid_input);
    RUN_TEST(test_flag_seconds_zero_when_valid_but_never_set);
    RUN_TEST(test_rows_after_the_grid_end_are_dropped);
    RUN_TEST(test_the_waiting_ring_confirms_the_oldest_row_when_full);
    RUN_TEST(test_max_first_last);
    RUN_TEST(test_missing_first_and_last_are_invalid_input);
    RUN_TEST(test_counts_are_zero_without_rows_and_ignore_invalid_rows);
    RUN_TEST(test_warmup_time_from_the_first_rpm_to_the_first_operating_coolant);
    RUN_TEST(test_warmup_flags);
    RUN_TEST(test_warmup_starts_warm);
    RUN_TEST(test_warmup_operating_before_the_engine_start_is_starts_warm);
    RUN_TEST(test_thermal_cycles);
    RUN_TEST(test_fan_on_temperature_is_the_coolant_at_the_second_sample);
    RUN_TEST(test_fan_step_below_the_voltage_step_is_no_step);
    RUN_TEST(test_fan_step_with_a_gap_longer_than_the_battery_limit_is_no_step);
    RUN_TEST(test_fan_step_with_a_gap_exactly_the_battery_limit_counts);
    RUN_TEST(test_fan_step_needs_rest_at_both_samples);
    RUN_TEST(test_fan_step_needs_a_warm_coolant_and_the_idle_band);
    RUN_TEST(test_fan_step_uses_the_inputs_fresh_at_the_battery_time);
    RUN_TEST(test_waiting_row_capacities_cover_the_gen_table);
    RUN_TEST(test_calls_with_null_are_ignored);
    return UNITY_END();
}
