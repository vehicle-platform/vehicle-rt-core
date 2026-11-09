/*
 * D-079 health windows (features/derived/derived_windows): hand-computed cases of the state
 * machine and the statistics, on the real gen/ windows W1..W5. The golden replay of the
 * reference is test_derived_ride; here every open / close reason, the re-arm rule, the stale
 * close, the max duration clamp, the qualifying count and the first-instance values are
 * checked against numbers worked out by hand (times in ms).
 *
 * The windows (defs v0.24.0): W1 key on: ENGINE_SPEED == 0, 2 s .. 10 s. W2 crank: engine
 * start, 100 ms .. 3 s. W3 warm idle: coolant > 80, speed == 0, rpm 1000..1800, 20 s .. 60 s.
 * W4 cruise: gear >= 4, speed band 4, throttle band 5, 10 s .. 60 s. W5 post stop: engine
 * stop, ENGINE_SPEED == 0, 60 s .. 1800 s. Stale limits: rpm 330, speed 300, coolant 2400.
 */
#include "features/derived/derived_core.h"
#include "features/derived/derived_windows.h"

#include "vehicle_cl250.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#define RPM VEHICLE_CL250_IDX_ENGINE_SPEED
#define SPEED VEHICLE_CL250_IDX_VEHICLE_SPEED
#define THROTTLE VEHICLE_CL250_IDX_THROTTLE_POS
#define COOLANT VEHICLE_CL250_IDX_COOLANT_TEMP
#define BATTERY VEHICLE_CL250_IDX_BATTERY_VOLTAGE
#define INTAKE VEHICLE_CL250_IDX_INTAKE_AIR_TEMP
#define AMBIENT VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP
#define MAP_DID VEHICLE_CL250_IDX_INTAKE_MAP
#define BARO VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE
#define STFT VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1
#define ALL_WINDOWS (0x1Fu)
#define RING_CAP (548u)

static derived_windows_t W;
static derived_sample_t ring_data[RING_CAP];
static derived_ring_t ring;

void setUp(void)
{
    (void)memset(ring_data, 0, sizeof ring_data);
    ring.head = 0u;
    ring.count = 0u;
    derived_windows_begin(&W, ALL_WINDOWS, ring_data, &ring, RING_CAP);
}
void tearDown(void) {}

static void did(uint32_t idx, float v, uint32_t t)
{
    derived_windows_push_did(&W, idx, v, t);
    derived_windows_run(&W);
}

static void expect(uint32_t sig, double value, double tol)
{
    double v = -1.0;
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_NONE, derived_windows_value(&W, sig, &v));
    if (fabs(v - value) > tol) {
        printf("signal %u: expected %.9f got %.9f\n", (unsigned)sig, value, v);
    }
    TEST_ASSERT_TRUE_MESSAGE(fabs(v - value) <= tol, "value differs");
}

static void expect_flag(uint32_t sig, derived_flag_t flag)
{
    double v = 0.0;
    TEST_ASSERT_EQUAL_INT(flag, derived_windows_value(&W, sig, &v));
}

#define KEY_ON_COUNT VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_COUNT
#define CRANK_COUNT VEHICLE_CL250_DERIVED_WINDOW_CRANK_COUNT
#define IDLE_COUNT VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_COUNT
#define CRUISE_COUNT VEHICLE_CL250_DERIVED_WINDOW_CRUISE_COUNT
#define STOP_COUNT VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COUNT

/* ---- W1: key on ------------------------------------------------------------------------------- */

/* Engine off 0..2200 (110 ms cadence), the first instance closes by condition at 2310:
 * duration 2310 >= 2000, so it qualifies. The battery sample at 0 is pushed before the rpm
 * sample of the same time and still counts (window inputs first, then the statistics). */
static void test_key_on_closes_by_condition_and_reports_the_first_instance(void)
{
    derived_windows_push_did(&W, BATTERY, 12.0f, 0u); /* queued before the rpm sample of the same ms */
    derived_windows_push_did(&W, COOLANT, 90.0f, 0u);
    derived_windows_push_did(&W, INTAKE, 30.0f, 0u);
    derived_windows_push_did(&W, AMBIENT, 20.0f, 0u);
    derived_windows_push_did(&W, MAP_DID, 40.0f, 0u);
    derived_windows_push_did(&W, BARO, 100.0f, 0u);
    derived_windows_push_did(&W, RPM, 0.0f, 0u);
    derived_windows_run(&W);
    for (uint32_t t = 110u; t <= 2200u; t += 110u) {
        did(RPM, 0.0f, t);
        if (t == 770u) {
            did(BATTERY, 12.4f, t);
        }
        if (t == 1650u) {
            did(BATTERY, 12.2f, t);
        }
    }
    did(RPM, 900.0f, 2310u); /* the engine runs: the condition fails at 2310 */
    did(BATTERY, 99.0f, 2400u); /* after the close: outside */
    derived_windows_end(&W);
    expect(KEY_ON_COUNT, 1.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, (12.0 + 12.4 + 12.2) / 3.0, 1.0e-5);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_SENSOR_SPREAD, 70.0, 1.0e-9);  /* 90 - 20 */
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_MAP_BARO_DELTA, -60.0, 1.0e-9); /* 40 - 100 */
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_WARM_RESTART, 1.0, 0.0);       /* |90 - 20| > 5 */
}

static void test_key_on_cold_soak_is_no_warm_restart(void)
{
    derived_windows_push_did(&W, COOLANT, 22.0f, 0u);
    derived_windows_push_did(&W, AMBIENT, 20.0f, 0u);
    did(RPM, 0.0f, 0u);
    for (uint32_t t = 110u; t <= 2200u; t += 110u) {
        did(RPM, 0.0f, t);
    }
    derived_windows_end(&W);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_WARM_RESTART, 0.0, 0.0); /* |22 - 20| <= 5 */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, DERIVED_FLAG_NO_DATA); /* no battery sample */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_SENSOR_SPREAD, DERIVED_FLAG_NO_DATA); /* no intake sample */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_MAP_BARO_DELTA, DERIVED_FLAG_NO_DATA);
}

static void test_a_short_instance_does_not_qualify(void)
{
    for (uint32_t t = 0u; t <= 1100u; t += 110u) {
        did(RPM, 0.0f, t);
    }
    did(RPM, 900.0f, 1210u); /* closed by condition after 1210 ms < 2000 */
    for (uint32_t t = 1320u; t <= 4000u; t += 110u) {
        did(RPM, 0.0f, t); /* a key-on window opens at the first point only */
    }
    derived_windows_end(&W);
    expect(KEY_ON_COUNT, 0.0, 0.0);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, DERIVED_FLAG_NO_WINDOW);
}

static void key_on_then_a_gap(bool later_sample)
{
    derived_windows_push_did(&W, BATTERY, 12.0f, 0u);
    did(RPM, 0.0f, 0u);
    for (uint32_t t = 110u; t <= 2200u; t += 110u) {
        did(RPM, 0.0f, t);
    }
    did(BATTERY, 13.0f, 2400u); /* before the expiry 2200 + 330 + 1 = 2531: inside */
    did(BATTERY, 14.0f, 2600u); /* after it: outside */
    if (later_sample) {
        did(RPM, 0.0f, 6000u); /* proves the expiry point 2531 */
    }
    derived_windows_end(&W);
}

static void test_a_lost_input_closes_at_its_stale_instant(void)
{
    key_on_then_a_gap(true);
    /* closed by condition at 2531 (duration 2531 >= 2000): the 2400 sample is inside */
    expect(KEY_ON_COUNT, 1.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, 12.5, 1.0e-6);
}

static void test_an_expiry_without_a_later_sample_is_not_a_point(void)
{
    key_on_then_a_gap(false);
    /* no later window sample: the instance ends with the ride at the last point 2200, and the
     * samples after it are outside */
    expect(KEY_ON_COUNT, 1.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, 12.0, 1.0e-6);
}

static void test_max_duration_clamps_the_close_and_keeps_its_boundary_sample(void)
{
    derived_windows_push_did(&W, BATTERY, 12.0f, 0u);
    did(RPM, 0.0f, 0u);
    for (uint32_t t = 110u; t <= 10500u; t += 110u) {
        did(RPM, 0.0f, t);
        if (t == 9900u) {
            did(BATTERY, 14.0f, 10000u);
        }
        if (t == 10010u) {
            did(BATTERY, 20.0f, 10050u); /* after open + max: outside */
        }
    }
    derived_windows_end(&W);
    /* the first point >= 10000 is 10010: closed at open + max = 10000, qualifying by max duration */
    expect(KEY_ON_COUNT, 1.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, 13.0, 1.0e-6); /* 12 at 0, 14 at 10000 */
}

static void test_the_peek_sees_the_open_instance(void)
{
    derived_windows_push_did(&W, BATTERY, 12.0f, 0u);
    did(RPM, 0.0f, 0u);
    for (uint32_t t = 110u; t <= 2200u; t += 110u) {
        did(RPM, 0.0f, t);
    }
    /* last evaluated point 2090: 2090 >= 2000, as if the ride ended there */
    expect(KEY_ON_COUNT, 1.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE, 12.0, 1.0e-6);
}

static void test_a_summary_without_an_instance_is_no_window_and_counts_are_zero(void)
{
    expect(KEY_ON_COUNT, 0.0, 0.0);
    expect(CRANK_COUNT, 0.0, 0.0);
    expect(STOP_COUNT, 0.0, 0.0);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START, DERIVED_FLAG_NO_WINDOW);
}

/* ---- W2: crank (engine start) ---------------------------------------------------------------- */

static void test_crank_opens_on_a_transition_and_closes_on_engine_zero_and_rearms(void)
{
    did(RPM, 0.0f, 0u);
    did(RPM, 0.0f, 110u);
    did(RPM, 800.0f, 220u); /* 0 -> > 0, consecutive: opens at 220 */
    did(RPM, 800.0f, 330u);
    did(RPM, 800.0f, 440u);
    did(RPM, 0.0f, 520u); /* engine zero: closed, duration 300 >= 100 */
    expect(CRANK_COUNT, 1.0, 0.0);
    did(RPM, 0.0f, 630u);
    did(RPM, 900.0f, 740u); /* the next transition opens again */
    did(RPM, 900.0f, 850u);
    derived_windows_end(&W); /* ride end at 850: duration 110 >= 100 */
    expect(CRANK_COUNT, 2.0, 0.0);
}

static void test_crank_closes_unknown_when_the_engine_speed_is_lost(void)
{
    did(RPM, 0.0f, 0u);
    did(RPM, 700.0f, 110u); /* opens at 110 */
    did(RPM, 700.0f, 220u);
    did(RPM, 700.0f, 330u);
    did(RPM, 700.0f, 2000u); /* the expiry 330 + 331 = 661 is a point: unknown, duration 551 */
    derived_windows_end(&W);
    expect(CRANK_COUNT, 1.0, 0.0);
}

static void test_crank_needs_consecutive_samples_for_a_transition(void)
{
    did(RPM, 0.0f, 0u);
    did(RPM, 800.0f, 1000u); /* 1000 ms apart > 330: not a transition */
    did(RPM, 800.0f, 1110u);
    did(RPM, 800.0f, 1220u);
    derived_windows_end(&W);
    expect(CRANK_COUNT, 0.0, 0.0);
}

static void test_a_crank_longer_than_three_seconds_is_clamped_and_qualifies(void)
{
    did(RPM, 0.0f, 0u);
    did(RPM, 600.0f, 110u); /* opens at 110 */
    for (uint32_t t = 220u; t <= 3500u; t += 110u) {
        did(RPM, 600.0f, t);
    }
    derived_windows_end(&W);
    expect(CRANK_COUNT, 1.0, 0.0); /* closed by max duration at 3110 */
}

/* ---- W3: warm idle (condition, re-arm) ----------------------------------------------------------- */

/* Every 100 ms: coolant 90, speed 0, rpm `rpm_v(t)`; MAP and STFT every 2000 ms. */
static void idle_step(uint32_t t, float rpm_v, float speed)
{
    derived_windows_push_did(&W, COOLANT, 90.0f, t);
    derived_windows_push_did(&W, SPEED, speed, t);
    ring.head = (uint16_t)((ring.head + 1u) % RING_CAP);
    ring_data[ring.head].value = rpm_v;
    ring_data[ring.head].stamp_ms = t;
    if (ring.count < RING_CAP) {
        ring.count++;
    }
    derived_windows_push_did(&W, RPM, rpm_v, t);
    if ((t % 2000u) == 0u) {
        derived_windows_push_did(&W, MAP_DID, 30.0f + (float)(t / 2000u), t);
        derived_windows_push_did(&W, STFT, (((t / 2000u) % 2u) == 0u) ? 1.0f : 3.0f, t);
    }
    derived_windows_run(&W);
}

static void test_warm_idle_statistics_of_the_first_instance(void)
{
    for (uint32_t t = 0u; t <= 24000u; t += 100u) {
        idle_step(t, (t < 12000u) ? 1100.0f : 1300.0f, 0.0f);
    }
    derived_windows_end(&W); /* session end at 24000: 24000 >= 20000 */
    expect(IDLE_COUNT, 1.0, 0.0);
    /* rpm: 1100 before the middle 12000, 1300 from it */
    expect(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_SPREAD, 200.0, 1.0e-9);
    expect(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_DRIFT, 200.0, 1.0e-9);
    /* MAP at 0, 2000 .. 24000: 30 .. 42 */
    expect(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_MAP_SPREAD, 12.0, 1.0e-6);
    /* STFT at the same 13 times: 1, 3, 1, 3 ...: seven 1s and six 3s, mean 25 / 13 */
    expect(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_MEAN, 25.0 / 13.0, 1.0e-6);
    /* sample std (n - 1): m2 = 7 (12/13)^2 + 6 (14/13)^2 = 1680/169 / ... = 12.923077, / 12 */
    expect(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_STD, sqrt(((7.0 * 144.0 + 6.0 * 196.0) / 169.0) / 12.0), 1.0e-6);
}

static void test_a_condition_window_rearms_only_after_the_conditions_failed(void)
{
    uint32_t t = 0u;
    for (; t <= 125000u; t += 100u) { /* conditions hold throughout: one 60 s instance (max), no re-open */
        idle_step(t, 1200.0f, 0.0f);
    }
    expect(IDLE_COUNT, 1.0, 0.0);
    idle_step(125100u, 1200.0f, 5.0f); /* speed != 0: the conditions fail once */
    t = 125200u;
    for (; t <= 150400u; t += 100u) { /* 25.2 s of idle again: the second instance qualifies */
        idle_step(t, 1200.0f, 0.0f);
    }
    derived_windows_end(&W);
    expect(IDLE_COUNT, 2.0, 0.0);
}

static void test_warm_idle_without_samples_is_no_data(void)
{
    /* an instance with no MAP / STFT samples (they are polled every 2 s; use times off the grid) */
    for (uint32_t t = 1u; t <= 24001u; t += 100u) {
        derived_windows_push_did(&W, COOLANT, 90.0f, t);
        derived_windows_push_did(&W, SPEED, 0.0f, t);
        derived_windows_push_did(&W, RPM, 1100.0f, t);
        derived_windows_run(&W);
    }
    derived_windows_end(&W);
    expect(IDLE_COUNT, 1.0, 0.0);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_MAP_SPREAD, DERIVED_FLAG_NO_DATA);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_MEAN, DERIVED_FLAG_NO_DATA);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_STD, DERIVED_FLAG_INSUFFICIENT_SAMPLES);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_SPREAD, DERIVED_FLAG_NO_DATA); /* no history ring samples */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_DRIFT, DERIVED_FLAG_NO_DATA);
}

/* ---- W4: cruise (a derived condition, stable bands) ---------------------------------------------- */

static void cruise_step(uint32_t t, bool gear_valid, float speed, float throttle)
{
    derived_windows_push_derived(&W, VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, gear_valid, 5.0f, t);
    derived_windows_push_did(&W, SPEED, speed, t);
    derived_windows_push_did(&W, THROTTLE, throttle, t);
    derived_windows_run(&W);
}

static void test_cruise_closes_when_a_stable_band_is_exceeded(void)
{
    uint32_t t = 0u;
    for (; t <= 12000u; t += 100u) {
        cruise_step(t, true, 60.0f + (((t / 100u) % 2u) == 0u ? 0.0f : 2.0f), 20.0f); /* speed spread 2 <= 4 */
    }
    cruise_step(12100u, true, 66.0f, 20.0f); /* spread 6 > 4: closed, 12100 ms >= 10000 */
    cruise_step(12200u, true, 66.0f, 20.0f);
    derived_windows_end(&W);
    expect(CRUISE_COUNT, 1.0, 0.0);
}

static void test_cruise_that_ends_too_early_does_not_qualify(void)
{
    uint32_t t = 0u;
    for (; t <= 5000u; t += 100u) {
        cruise_step(t, true, 60.0f, 20.0f);
    }
    cruise_step(5100u, true, 60.0f, 30.0f); /* throttle spread 10 > 5 at 5100 ms < 10000 */
    derived_windows_end(&W);
    expect(CRUISE_COUNT, 0.0, 0.0);
}

static void test_an_unknown_gear_closes_the_cruise_by_condition(void)
{
    uint32_t t = 0u;
    for (; t <= 11000u; t += 100u) {
        cruise_step(t, true, 60.0f, 20.0f);
    }
    cruise_step(11100u, false, 60.0f, 20.0f); /* an invalid row: the condition fails */
    derived_windows_end(&W);
    expect(CRUISE_COUNT, 1.0, 0.0);
}

/* ---- W5: post stop (engine stop) ----------------------------------------------------------------- */

/* The engine stops at 220 (800 rpm samples before, then 0 every 110 ms). Coolant minus ambient
 * (20 degC) is 64, 32, 16, 8 at 0, 60, 120, 180 s after the open: ln is linear, slope
 * -ln 2 / 60, tau = 60 / ln 2. */
static void test_post_stop_statistics_and_tau(void)
{
    /* every 60 s boundary is not a multiple of 110 ms: use the rpm cadence times 220 + 110 m
     * closest from above; the samples are pushed at 220, 60280, 120340, 180400 */
    did(RPM, 800.0f, 0u);
    did(RPM, 800.0f, 110u);
    did(AMBIENT, 20.0f, 150u); /* before the open: outside */
    const uint32_t at[4] = {220u, 60280u, 120340u, 180400u};
    const float cool[4] = {84.0f, 52.0f, 36.0f, 28.0f};
    uint32_t k = 0u;
    for (uint32_t t = 220u; t <= 180400u; t += 110u) {
        derived_windows_push_did(&W, RPM, 0.0f, t);
        if (t == 330u) {
            derived_windows_push_did(&W, AMBIENT, 20.0f, t);
        }
        if ((k < 4u) && (t == at[k])) {
            derived_windows_push_did(&W, COOLANT, cool[k], t);
            k++;
        }
        derived_windows_run(&W);
    }
    derived_windows_end(&W);
    expect(STOP_COUNT, 1.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START, 84.0, 0.0);
    expect(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_END, 28.0, 0.0);
    /* x = 0, 60.06, 120.12, 180.18 s; v - ambient = 64, 32, 16, 8: the slope is the least squares
     * of ln over these x */
    const double x[4] = {0.0, 60.06, 120.12, 180.18};
    const double y[4] = {log(64.0), log(32.0), log(16.0), log(8.0)};
    double mx = 0.0;
    double my = 0.0;
    for (uint32_t i = 0u; i < 4u; i++) {
        mx += x[i] / 4.0;
        my += y[i] / 4.0;
    }
    double var = 0.0;
    double cov = 0.0;
    for (uint32_t i = 0u; i < 4u; i++) {
        var += (x[i] - mx) * (x[i] - mx);
        cov += (x[i] - mx) * (y[i] - my);
    }
    expect(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU, -var / cov, 1.0e-6);
    expect(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU, 60.06 / log(2.0), 1.0e-6); /* exactly linear */
}

static void post_stop_series(const float* coolant, const uint32_t* at, uint32_t n, bool ambient)
{
    did(RPM, 800.0f, 0u);
    did(RPM, 800.0f, 110u);
    uint32_t k = 0u;
    for (uint32_t t = 220u; t <= 61000u; t += 110u) {
        derived_windows_push_did(&W, RPM, 0.0f, t);
        if (ambient && (t == 330u)) {
            derived_windows_push_did(&W, AMBIENT, 20.0f, t);
        }
        if ((k < n) && (t == at[k])) {
            derived_windows_push_did(&W, COOLANT, coolant[k], t);
            k++;
        }
        derived_windows_run(&W);
    }
    derived_windows_end(&W);
}

static void test_tau_failure_flags(void)
{
    const uint32_t at[3] = {330u, 20020u, 40040u};
    const float rising[3] = {30.0f, 40.0f, 50.0f}; /* ln(10), ln(20), ln(30): slope > 0 */
    post_stop_series(rising, at, 3u, true);
    expect(STOP_COUNT, 1.0, 0.0);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU, DERIVED_FLAG_NO_DATA);
}

static void test_tau_needs_three_positive_differences_and_an_ambient(void)
{
    const uint32_t at[3] = {330u, 20020u, 40040u};
    const float below[3] = {15.0f, 18.0f, 50.0f}; /* only one above the ambient 20 */
    post_stop_series(below, at, 3u, true);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU, DERIVED_FLAG_INSUFFICIENT_SAMPLES);
    expect(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START, 15.0, 0.0);
    setUp();
    post_stop_series(below, at, 3u, false); /* no ambient sample at all */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU, DERIVED_FLAG_NO_DATA);
}

static void test_tau_with_equal_times_is_insufficient(void)
{
    /* three coolant samples at one instant (variance 0): INSUFFICIENT_SAMPLES */
    did(RPM, 800.0f, 0u);
    did(RPM, 800.0f, 110u);
    for (uint32_t t = 220u; t <= 61000u; t += 110u) {
        did(RPM, 0.0f, t);
        if (t == 330u) {
            derived_windows_push_did(&W, AMBIENT, 20.0f, t);
            for (uint32_t i = 0u; i < 3u; i++) {
                derived_windows_push_did(&W, COOLANT, 50.0f, t);
            }
            derived_windows_run(&W);
        }
    }
    derived_windows_end(&W);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU, DERIVED_FLAG_INSUFFICIENT_SAMPLES); /* one sample */
}

/* ---- bounds ------------------------------------------------------------------------------------ */

static void test_a_pending_list_overflow_is_a_math_error_not_a_wrong_value(void)
{
    did(RPM, 0.0f, 0u);
    for (uint32_t t = 1u; t <= 20u; t++) { /* 20 battery samples between two engine samples */
        did(BATTERY, 12.0f, t);
    }
    expect_flag(KEY_ON_COUNT, DERIVED_FLAG_MATH_ERROR);
}

static void test_an_event_overflow_is_a_math_error(void)
{
    for (uint32_t i = 0u; i < (DERIVED_WINDOWS_EVENT_CAP + 1u); i++) {
        derived_windows_push_did(&W, RPM, 0.0f, i);
    }
    derived_windows_run(&W);
    expect_flag(KEY_ON_COUNT, DERIVED_FLAG_MATH_ERROR);
}

static void test_the_coolant_buffer_overflow_is_a_math_error(void)
{
    did(RPM, 800.0f, 0u);
    did(RPM, 800.0f, 110u);
    uint32_t t = 220u;
    for (; t <= 2400u; t += 110u) {
        did(RPM, 0.0f, t);
    }
    /* the post-stop window holds 2252 coolant samples; send more, one per ms (stale-safe rpm) */
    uint32_t c = 0u;
    for (t = 2401u; c < (DERIVED_WIN_POST_STOP_COOLANT_CAP + 5u); t++, c++) {
        if (((t - 220u) % 110u) == 0u) {
            derived_windows_push_did(&W, RPM, 0.0f, t);
        }
        derived_windows_push_did(&W, COOLANT, 50.0f, t);
        derived_windows_run(&W);
    }
    derived_windows_end(&W);
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START, DERIVED_FLAG_MATH_ERROR);
}

static void post_stop_with_coolant(float first, float second, uint32_t second_at)
{
    did(RPM, 800.0f, 0u);
    did(RPM, 800.0f, 110u);
    for (uint32_t t = 220u; t <= second_at; t += 110u) {
        did(RPM, 0.0f, t);
        if (t == 330u) {
            did(COOLANT, first, t);
        }
        if (t == second_at) {
            did(COOLANT, second, t);
        }
    }
    derived_windows_end(&W);
}

static void test_a_coolant_value_the_did_cannot_decode_is_a_math_error(void)
{
    post_stop_with_coolant(50.0f, 50.5f, 440u); /* 50.5 is not an integer raw of the 1 degC DID */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START, DERIVED_FLAG_MATH_ERROR);
}

static void test_a_coolant_gap_above_65_s_is_a_math_error(void)
{
    post_stop_with_coolant(50.0f, 51.0f, 70070u); /* 69.7 s between two stored samples */
    expect_flag(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_END, DERIVED_FLAG_MATH_ERROR);
}

static void test_gen_checks(void)
{
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_WINDOW_W1_KEY_ON,
                             derived_windows_of_signal(VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_COUNT));
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_WINDOW_W5_POST_STOP,
                             derived_windows_of_signal(VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COUNT));
    TEST_ASSERT_EQUAL_UINT32(DERIVED_WINDOWS_NONE, derived_windows_of_signal(VEHICLE_CL250_DERIVED_RIDE_DISTANCE));
    TEST_ASSERT_EQUAL_UINT32(DERIVED_WINDOWS_NONE, derived_windows_of_signal(VEHICLE_CL250_DERIVED_COUNT));
    TEST_ASSERT_TRUE(derived_windows_has_summary(VEHICLE_CL250_DERIVED_WINDOW_CRUISE_COUNT));
    TEST_ASSERT_FALSE(derived_windows_has_summary(VEHICLE_CL250_DERIVED_WINDOW_CRUISE_VE_RESIDUAL));
    for (uint32_t w = 0u; w < DERIVED_WINDOWS_COUNT; w++) {
        TEST_ASSERT_TRUE(derived_windows_sizing_ok(w, DERIVED_WIN_RPM_HIST_CAP));
    }
    TEST_ASSERT_FALSE(derived_windows_sizing_ok(DERIVED_WINDOWS_COUNT, DERIVED_WIN_RPM_HIST_CAP));
    /* the warm-idle RPM statistics need an ENGINE_SPEED history that holds the longest instance */
    const uint32_t w3_need = (vehicle_cl250_window[VEHICLE_CL250_WINDOW_W3_WARM_IDLE].max_duration_ms /
                              vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].poll_period_ms) + 2u;
    TEST_ASSERT_TRUE(derived_windows_sizing_ok(VEHICLE_CL250_WINDOW_W3_WARM_IDLE, w3_need));
    TEST_ASSERT_FALSE(derived_windows_sizing_ok(VEHICLE_CL250_WINDOW_W3_WARM_IDLE, w3_need - 1u));
    TEST_ASSERT_TRUE(derived_windows_sizing_ok(VEHICLE_CL250_WINDOW_W1_KEY_ON, 0u)); /* only W3 reads it */
    uint8_t chans[DERIVED_WINDOWS_CH_MAX];
    TEST_ASSERT_EQUAL_UINT32(0u, derived_windows_derived_inputs(VEHICLE_CL250_WINDOW_W3_WARM_IDLE, chans));
    TEST_ASSERT_EQUAL_UINT32(1u, derived_windows_derived_inputs(VEHICLE_CL250_WINDOW_W4_CRUISE, chans));
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, chans[0]);
    TEST_ASSERT_TRUE(derived_windows_derived_inputs(DERIVED_WINDOWS_COUNT, chans) > DERIVED_WINDOWS_CH_MAX);
}

static void test_null_and_disabled_arguments(void)
{
    derived_windows_begin(NULL, ALL_WINDOWS, NULL, NULL, 0u);
    derived_windows_push_did(NULL, RPM, 0.0f, 0u);
    derived_windows_push_derived(NULL, VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, true, 5.0f, 0u);
    derived_windows_push_did(&W, VEHICLE_CL250_DID_COUNT, 0.0f, 0u);
    derived_windows_push_derived(&W, VEHICLE_CL250_DERIVED_COUNT, true, 0.0f, 0u);
    derived_windows_run(NULL);
    derived_windows_end(NULL);
    double v = 0.0;
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_MATH_ERROR, derived_windows_value(NULL, KEY_ON_COUNT, &v));
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_MATH_ERROR, derived_windows_value(&W, KEY_ON_COUNT, NULL));
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_MATH_ERROR, derived_windows_value(&W, VEHICLE_CL250_DERIVED_RIDE_DISTANCE, &v));
    /* only W2 enabled: the others are MATH_ERROR (never computed), a sample nobody reads is ignored */
    derived_windows_begin(&W, 1u << VEHICLE_CL250_WINDOW_W2_CRANK, NULL, NULL, 0u);
    did(BATTERY, 12.0f, 0u);
    did(RPM, 0.0f, 0u);
    did(RPM, 0.0f, 110u);
    did(RPM, 800.0f, 220u);
    did(RPM, 800.0f, 330u);
    derived_windows_end(&W);
    expect(CRANK_COUNT, 1.0, 0.0);
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_MATH_ERROR, derived_windows_value(&W, KEY_ON_COUNT, &v));
}

static void test_a_sample_going_back_in_time_is_held_at_the_last_point(void)
{
    did(RPM, 0.0f, 100u);
    derived_windows_push_did(&W, RPM, 0.0f, 50u); /* earlier than the staged point */
    derived_windows_run(&W);
    for (uint32_t t = 210u; t <= 2300u; t += 110u) {
        did(RPM, 0.0f, t);
    }
    derived_windows_end(&W);
    expect(KEY_ON_COUNT, 1.0, 0.0);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_key_on_closes_by_condition_and_reports_the_first_instance);
    RUN_TEST(test_key_on_cold_soak_is_no_warm_restart);
    RUN_TEST(test_a_short_instance_does_not_qualify);
    RUN_TEST(test_a_lost_input_closes_at_its_stale_instant);
    RUN_TEST(test_an_expiry_without_a_later_sample_is_not_a_point);
    RUN_TEST(test_max_duration_clamps_the_close_and_keeps_its_boundary_sample);
    RUN_TEST(test_the_peek_sees_the_open_instance);
    RUN_TEST(test_a_summary_without_an_instance_is_no_window_and_counts_are_zero);
    RUN_TEST(test_crank_opens_on_a_transition_and_closes_on_engine_zero_and_rearms);
    RUN_TEST(test_crank_closes_unknown_when_the_engine_speed_is_lost);
    RUN_TEST(test_crank_needs_consecutive_samples_for_a_transition);
    RUN_TEST(test_a_crank_longer_than_three_seconds_is_clamped_and_qualifies);
    RUN_TEST(test_warm_idle_statistics_of_the_first_instance);
    RUN_TEST(test_a_condition_window_rearms_only_after_the_conditions_failed);
    RUN_TEST(test_warm_idle_without_samples_is_no_data);
    RUN_TEST(test_cruise_closes_when_a_stable_band_is_exceeded);
    RUN_TEST(test_cruise_that_ends_too_early_does_not_qualify);
    RUN_TEST(test_an_unknown_gear_closes_the_cruise_by_condition);
    RUN_TEST(test_post_stop_statistics_and_tau);
    RUN_TEST(test_tau_failure_flags);
    RUN_TEST(test_tau_needs_three_positive_differences_and_an_ambient);
    RUN_TEST(test_tau_with_equal_times_is_insufficient);
    RUN_TEST(test_a_pending_list_overflow_is_a_math_error_not_a_wrong_value);
    RUN_TEST(test_an_event_overflow_is_a_math_error);
    RUN_TEST(test_the_coolant_buffer_overflow_is_a_math_error);
    RUN_TEST(test_a_coolant_value_the_did_cannot_decode_is_a_math_error);
    RUN_TEST(test_a_coolant_gap_above_65_s_is_a_math_error);
    RUN_TEST(test_gen_checks);
    RUN_TEST(test_null_and_disabled_arguments);
    RUN_TEST(test_a_sample_going_back_in_time_is_held_at_the_last_point);
    return UNITY_END();
}
