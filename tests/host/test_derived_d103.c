/* Hand-computed tests of the D-103 class-A signals (R21-2/R21-3). The golden replay in
 * test_derived_core.c is the reference evidence; these tests pin the flag paths and the
 * ride-reset of the new state that the replay scenarios do not cover. */
#include "features/derived/derived_core.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static derived_core_t core;
static derived_in_t in[VEHICLE_CL250_DID_COUNT];

static void rig(void)
{
    derived_core_init(&core);
    (void)memset(in, 0, sizeof in);
}

static void put(uint32_t did, float value, uint32_t stamp)
{
    in[did].value = value;
    in[did].stamp_ms = stamp;
    in[did].present = true;
}

static void all_dids(float value, uint32_t stamp)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        put(i, value, stamp);
    }
}

static void touch_all(uint32_t stamp)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        in[i].stamp_ms = stamp;
        in[i].present = true;
    }
}

static void step(uint32_t now)
{
    derived_core_step(&core, now, in);
}

static const derived_out_t* out(uint32_t sig)
{
    return derived_core_out(&core, sig);
}

static void valid(uint32_t sig, float expected, float tol)
{
    const derived_out_t* const o = out(sig);
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o->status);
    TEST_ASSERT_FLOAT_WITHIN(tol, expected, o->value);
}

static void invalid(uint32_t sig, derived_flag_t flag)
{
    const derived_out_t* const o = out(sig);
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, o->status);
    TEST_ASSERT_EQUAL_INT(flag, o->flag);
}

static float rpm_for_ratio(float speed_kmh, float ratio)
{
    const float wheel_rpm =
        ((speed_kmh * DERIVED_KMH_TO_MPS) / (2.0f * DERIVED_PI * VEHICLE_CL250_REAR_WHEEL_RADIUS_DYNAMIC_M)) *
        DERIVED_SECONDS_PER_MINUTE;
    return ratio * wheel_rpm * VEHICLE_CL250_PRIMARY_REDUCTION_RATIO * VEHICLE_CL250_SECONDARY_REDUCTION_RATIO;
}

static void state_rig(float throttle, float fuel_rate)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, rpm_for_ratio(60.0f, VEHICLE_CL250_GEAR_RATIO_3), 0u);
    put(VEHICLE_CL250_IDX_THROTTLE_POS, throttle, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, fuel_rate, 0u);
    put(VEHICLE_CL250_IDX_COOLANT_TEMP, 90.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_LOAD, 30.0f, 0u);
}

static void test_clutch_and_gear_states(void)
{
    state_rig(20.0f, 3.0f);
    step(0u);
    valid(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, 3.0f, 0.01f);
    valid(VEHICLE_CL250_DERIVED_CLUTCH_STATE, 0.0f, 0.0f);
    valid(VEHICLE_CL250_DERIVED_CLUTCH_SLIP_RATIO, 1.0f, 1.0e-4f);
    valid(VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE, 0.0f, 0.0f);

    /* twice the tolerance above 2nd (nearest gear) with throttle above the load minimum:
     * slipping, and outside the gear-match band, so no gear. */
    touch_all(110u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED,
        rpm_for_ratio(60.0f, VEHICLE_CL250_GEAR_RATIO_2 * (1.0f + (2.0f * VEHICLE_CL250_DERIVED_GEAR_RATIO_TOLERANCE))),
        110u);
    step(110u);
    valid(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, 0.0f, 0.0f);
    valid(VEHICLE_CL250_DERIVED_CLUTCH_STATE, 2.0f, 0.0f);
    invalid(VEHICLE_CL250_DERIVED_CLUTCH_SLIP_RATIO, DERIVED_FLAG_NO_GEAR);
    valid(VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE, 1.0f, 0.0f);

    /* twice the tolerance below 1st (nearest gear): disengaged, and no gear. */
    touch_all(220u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED,
        rpm_for_ratio(60.0f, VEHICLE_CL250_GEAR_RATIO_1 * (1.0f - (2.0f * VEHICLE_CL250_DERIVED_GEAR_RATIO_TOLERANCE))),
        220u);
    step(220u);
    valid(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, 0.0f, 0.0f);
    valid(VEHICLE_CL250_DERIVED_CLUTCH_STATE, 1.0f, 0.0f);
    valid(VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE, 1.0f, 0.0f);

    /* a gear and closed throttle: coasting, and above idle with no fuel = engine braking. */
    touch_all(330u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, rpm_for_ratio(60.0f, VEHICLE_CL250_GEAR_RATIO_3), 330u);
    put(VEHICLE_CL250_IDX_THROTTLE_POS, 1.0f, 330u);
    put(VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 0.0f, 330u);
    step(330u);
    valid(VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE, 2.0f, 0.0f);
    valid(VEHICLE_CL250_DERIVED_ENGINE_BRAKING_PHASE, 1.0f, 0.0f);

    /* below the moving threshold is its own flag, not a stale input. */
    touch_all(440u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 2.0f, 440u);
    step(440u);
    invalid(VEHICLE_CL250_DERIVED_CLUTCH_STATE, DERIVED_FLAG_NOT_MOVING);
    invalid(VEHICLE_CL250_DERIVED_CLUTCH_SLIP_RATIO, DERIVED_FLAG_INVALID_INPUT);
    invalid(VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE, DERIVED_FLAG_NOT_MOVING);
}

static void test_engine_flags_and_residuals(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 9600.0f, 0u);
    put(VEHICLE_CL250_IDX_COOLANT_TEMP, 40.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_LOAD, 90.0f, 0u);
    step(0u);
    valid(VEHICLE_CL250_DERIVED_OVER_REV, 1.0f, 0.0f);
    valid(VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE, 1.0f, 0.0f);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 15.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, rpm_for_ratio(15.0f, VEHICLE_CL250_GEAR_RATIO_2), 0u);
    put(VEHICLE_CL250_IDX_ENGINE_LOAD, 90.0f, 0u);
    step(0u);
    valid(VEHICLE_CL250_DERIVED_LUGGING, 1.0f, 0.0f);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 5.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, 40.0f, 0u);
    put(VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP, 25.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_MAP, 36.0f, 0u);
    put(VEHICLE_CL250_IDX_THROTTLE_POS, 1.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1500.0f, 0u);
    step(0u);
    valid(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, 15.0f, 1.0e-5f);
    valid(VEHICLE_CL250_DERIVED_IDLE_MAP_RESIDUAL, 1.0f, 1.0e-5f);

    /* VE and residual at 100 % and 85 % of the same theoretical airflow. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_MAP, 100.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, 25.0f, 0u);
    /* full-cylinder flow, g/s: rho = p / (R T), times displacement and rpm / 120 (four-stroke). */
    const float rho = (100.0f * DERIVED_PA_PER_KPA) / (DERIVED_R_AIR_VE * (25.0f + DERIVED_KELVIN_OFFSET));
    const float theoretical_gps = rho * ((float)VEHICLE_CL250_ENGINE_DISPLACEMENT_CM3 * DERIVED_M3_PER_CM3) *
                                  (3000.0f / (DERIVED_SECONDS_PER_MINUTE * (float)VEHICLE_CL250_ENGINE_REVOLUTIONS_PER_CYCLE)) * DERIVED_G_PER_KG;
    put(VEHICLE_CL250_IDX_MAF_RATE, theoretical_gps, 0u);
    step(0u);
    valid(VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY, 100.0f, 0.1f);
    valid(VEHICLE_CL250_DERIVED_INTAKE_FLOW_RESIDUAL, 17.65f, 0.1f);

    /* a value outside the gen range is OUT_OF_RANGE, never clamped. */
    touch_all(110u);
    put(VEHICLE_CL250_IDX_MAF_RATE, 30.0f, 110u);
    step(110u);
    invalid(VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY, DERIVED_FLAG_OUT_OF_RANGE);
    invalid(VEHICLE_CL250_DERIVED_INTAKE_FLOW_RESIDUAL, DERIVED_FLAG_OUT_OF_RANGE);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_MAP, 200.0f, 0u);
    put(VEHICLE_CL250_IDX_THROTTLE_POS, 1.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1500.0f, 0u);
    step(0u);
    invalid(VEHICLE_CL250_DERIVED_IDLE_MAP_RESIDUAL, DERIVED_FLAG_OUT_OF_RANGE);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, -40.0f, 0u);
    put(VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP, 100.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    step(0u);
    invalid(VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA, DERIVED_FLAG_OUT_OF_RANGE);
}

static void test_battery_and_fuel_windows(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 14.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3500.0f, 0u);
    step(0u);
    valid(VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE, 14.0f, 1.0e-5f);

    touch_all(110u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 20.0f, 110u);
    step(110u);
    invalid(VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE, DERIVED_FLAG_OUT_OF_RANGE);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 50.0f, 0u);
    step(0u);
    invalid(VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED, DERIVED_FLAG_INSUFFICIENT_SAMPLES);
    /* the signal is on the 5 s fuel-level grid; two accel-fresh fuel samples are needed. */
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 4900u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 50.0f, 4900u);
    step(4900u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 5000u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 50.0f, 5000u);
    step(5000u);
    valid(VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED, 50.0f, 1.0e-5f);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 39900u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 130.0f, 39900u);
    step(39900u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 40000u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 130.0f, 40000u);
    step(40000u);
    invalid(VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED, DERIVED_FLAG_OUT_OF_RANGE);
}

static void test_idle_windows_and_drift(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_COOLANT_TEMP, 85.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1500.0f, 0u);
    step(0u);
    invalid(VEHICLE_CL250_DERIVED_IDLE_RPM_STD, DERIVED_FLAG_INSUFFICIENT_SAMPLES);
    touch_all(110u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1500.0f, 110u);
    step(110u);
    valid(VEHICLE_CL250_DERIVED_IDLE_RPM_STD, 0.0f, 0.0f);
    invalid(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, DERIVED_FLAG_INSUFFICIENT_SAMPLES);

    /* a full first window at 1000 rpm, then a full window at 1800 rpm: +800 is outside the
     * -500..500 gen range. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_COOLANT_TEMP, 85.0f, 0u);
    for (uint32_t t = 0u; t <= 120000u; t += 110u) {
        const float rpm = (t < 60000u) ? 1000.0f : 1800.0f;
        put(VEHICLE_CL250_IDX_ENGINE_SPEED, rpm, t);
        put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, t);
        put(VEHICLE_CL250_IDX_COOLANT_TEMP, 85.0f, t);
        step(t);
    }
    invalid(VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT, DERIVED_FLAG_OUT_OF_RANGE);
}

static void test_events(void)
{
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 2000.0f, 0u);
    step(0u);
    touch_all(110u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 110u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 110u);
    step(110u);
    valid(VEHICLE_CL250_DERIVED_SHIFT_EVENT, 110.0f, 0.0f);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    step(0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 5.0f, 110u);
    step(110u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 11.0f, 220u);
    step(220u);
    valid(VEHICLE_CL250_DERIVED_LAUNCH_EVENT, 110.0f, 0.0f);

    /* -3.63 m/s2 for 330 ms, then a flat sample closes the run. */
    rig();
    all_dids(10.0f, 0u);
    float speed = 60.0f;
    for (uint32_t t = 0u; t <= 220u; t += 110u) {
        put(VEHICLE_CL250_IDX_VEHICLE_SPEED, speed, t);
        step(t);
        speed -= 1.44f; /* 0.4 m/s per 110 ms */
    }
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, speed, 550u);
    step(550u);
    valid(VEHICLE_CL250_DERIVED_BRAKING_EVENT, -3.63636f, 0.01f);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 0u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 20.0f, 0u);
    step(0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 110u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 35.0f, 110u);
    step(110u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 220u);
    step(220u);
    valid(VEHICLE_CL250_DERIVED_REFUEL_EVENT, 1.8f, 1.0e-4f);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 12.5f, 0u);
    step(0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 500.0f, 110u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 9.0f, 110u);
    step(110u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1100.0f, 220u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 12.0f, 220u);
    step(220u);
    valid(VEHICLE_CL250_DERIVED_CRANKING_DIP, 9.0f, 1.0e-5f);
    valid(VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE, 12.5f, 1.0e-5f);

    /* out of the 0..16 V range: OUT_OF_RANGE, never clamped. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 30.0f, 0u);
    step(0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 500.0f, 110u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 30.0f, 110u);
    step(110u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1100.0f, 220u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 30.0f, 220u);
    step(220u);
    invalid(VEHICLE_CL250_DERIVED_CRANKING_DIP, DERIVED_FLAG_OUT_OF_RANGE);
    invalid(VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE, DERIVED_FLAG_OUT_OF_RANGE);
}

static void test_a_new_ride_resets_the_new_state(void)
{
    const derived_epoch_t first = {true, true, 1u};
    const derived_epoch_t second = {true, true, 2u};

    /* a window must hold only samples of its own ride. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 12.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3500.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    valid(VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE, 12.0f, 1.0e-5f);
    derived_core_step_epoch(&core, 110u, in, &second);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 14.0f, 220u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3500.0f, 220u);
    derived_core_step_epoch(&core, 220u, in, &second);
    valid(VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE, 14.0f, 1.0e-5f);

    /* an event window is cleared, including a run in progress at the reset. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 12.5f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 0.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    put(VEHICLE_CL250_IDX_BATTERY_VOLTAGE, 9.0f, 110u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 500.0f, 110u);
    derived_core_step_epoch(&core, 110u, in, &first);
    derived_core_step_epoch(&core, 220u, in, &second);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1100.0f, 330u);
    derived_core_step_epoch(&core, 330u, in, &second);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, out(VEHICLE_CL250_DERIVED_CRANKING_DIP)->status);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, out(VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE)->status);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 5.0f, 110u);
    derived_core_step_epoch(&core, 110u, in, &first);
    derived_core_step_epoch(&core, 220u, in, &second);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 11.0f, 330u);
    derived_core_step_epoch(&core, 330u, in, &second);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, out(VEHICLE_CL250_DERIVED_LAUNCH_EVENT)->status);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 58.0f, 110u);
    derived_core_step_epoch(&core, 110u, in, &first);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 56.0f, 220u);
    derived_core_step_epoch(&core, 220u, in, &first);
    derived_core_step_epoch(&core, 330u, in, &second);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 56.0f, 440u);
    derived_core_step_epoch(&core, 440u, in, &second);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, out(VEHICLE_CL250_DERIVED_BRAKING_EVENT)->status);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 0u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 20.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 110u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 35.0f, 110u);
    derived_core_step_epoch(&core, 110u, in, &first);
    derived_core_step_epoch(&core, 220u, in, &second);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 330u);
    derived_core_step_epoch(&core, 330u, in, &second);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, out(VEHICLE_CL250_DERIVED_REFUEL_EVENT)->status);

    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 2000.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    derived_core_step_epoch(&core, 110u, in, &second);
    const uint32_t seq = out(VEHICLE_CL250_DERIVED_SHIFT_EVENT)->seq;
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 50.0f, 220u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 220u);
    derived_core_step_epoch(&core, 220u, in, &second);
    TEST_ASSERT_EQUAL_UINT32(seq, out(VEHICLE_CL250_DERIVED_SHIFT_EVENT)->seq);

    /* the idle statistics must not carry the previous ride's points. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 0.0f, 0u);
    put(VEHICLE_CL250_IDX_COOLANT_TEMP, 85.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1500.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1500.0f, 110u);
    derived_core_step_epoch(&core, 110u, in, &first);
    derived_core_step_epoch(&core, 220u, in, &second);
    touch_all(330u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 1800.0f, 330u);
    derived_core_step_epoch(&core, 330u, in, &second);
    invalid(VEHICLE_CL250_DERIVED_IDLE_RPM_STD, DERIVED_FLAG_INSUFFICIENT_SAMPLES);

    /* the filtered fuel window is cleared as well. */
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 12.0f, 0u);
    derived_core_step_epoch(&core, 0u, in, &first);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 4900u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 12.0f, 4900u);
    derived_core_step_epoch(&core, 4900u, in, &first);
    derived_core_step_epoch(&core, 5000u, in, &second);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 9800u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 14.0f, 9800u);
    derived_core_step_epoch(&core, 9800u, in, &second);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 9900u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 14.0f, 9900u);
    derived_core_step_epoch(&core, 9900u, in, &second);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 10000u);
    put(VEHICLE_CL250_IDX_FUEL_LEVEL, 14.0f, 10000u);
    derived_core_step_epoch(&core, 10000u, in, &second);
    valid(VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED, 14.0f, 1.0e-5f);
}

static void test_every_point_signal_reports_its_stale_did(void)
{
    static const uint32_t sigs[] = {
        VEHICLE_CL250_DERIVED_CLUTCH_STATE,
        VEHICLE_CL250_DERIVED_CLUTCH_SLIP_RATIO,
        VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE,
        VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY,
        VEHICLE_CL250_DERIVED_INTAKE_FLOW_RESIDUAL,
        VEHICLE_CL250_DERIVED_IDLE_MAP_RESIDUAL,
        VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA,
        VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE,
        VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE,
        VEHICLE_CL250_DERIVED_OVER_REV,
        VEHICLE_CL250_DERIVED_LUGGING,
        VEHICLE_CL250_DERIVED_IDLE_RPM_STD,
        VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT,
        VEHICLE_CL250_DERIVED_ENGINE_BRAKING_PHASE,
        VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED,
    };
    rig();
    all_dids(10.0f, 0u);
    put(VEHICLE_CL250_IDX_VEHICLE_SPEED, 60.0f, 0u);
    put(VEHICLE_CL250_IDX_ENGINE_SPEED, 3000.0f, 0u);
    step(0u);
    step(30000u);
    /* the two signals that read a derived output see it INVALID (the reference maps a stale
     * dependency to invalid_input); the rest report the stale DID directly. */
    static const derived_flag_t flags[] = {
        DERIVED_FLAG_STALE_INPUT,  DERIVED_FLAG_INVALID_INPUT, DERIVED_FLAG_STALE_INPUT,
        DERIVED_FLAG_STALE_INPUT,  DERIVED_FLAG_STALE_INPUT,   DERIVED_FLAG_STALE_INPUT,
        DERIVED_FLAG_STALE_INPUT,  DERIVED_FLAG_STALE_INPUT,   DERIVED_FLAG_STALE_INPUT,
        DERIVED_FLAG_STALE_INPUT,  DERIVED_FLAG_INVALID_INPUT, DERIVED_FLAG_STALE_INPUT,
        DERIVED_FLAG_STALE_INPUT,  DERIVED_FLAG_STALE_INPUT,   DERIVED_FLAG_STALE_INPUT,
    };
    TEST_ASSERT_EQUAL_UINT32(sizeof sigs / sizeof sigs[0], sizeof flags / sizeof flags[0]);
    for (uint32_t i = 0u; i < (sizeof sigs / sizeof sigs[0]); i++) {
        invalid(sigs[i], flags[i]);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_clutch_and_gear_states);
    RUN_TEST(test_engine_flags_and_residuals);
    RUN_TEST(test_battery_and_fuel_windows);
    RUN_TEST(test_idle_windows_and_drift);
    RUN_TEST(test_events);
    RUN_TEST(test_a_new_ride_resets_the_new_state);
    RUN_TEST(test_every_point_signal_reports_its_stale_did);
    return UNITY_END();
}
