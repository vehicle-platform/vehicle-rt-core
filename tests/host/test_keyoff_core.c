/* L0 tests for the key-off power-down state machine (D-138 item 2), host only. */
#include "features/power/keyoff_core.h"

#include <stdint.h>
#include <string.h>
#include <unity.h>

#define ACK_NONE ((uint8_t)PLATFORM_SHUTDOWN_ACK_SHUTDOWN_ACK_STATE_NONE_CHOICE)
#define ACK_HALTING ((uint8_t)PLATFORM_SHUTDOWN_ACK_SHUTDOWN_ACK_STATE_HALTING_CHOICE)
#define ACK_READY ((uint8_t)PLATFORM_SHUTDOWN_ACK_SHUTDOWN_ACK_STATE_READY_TO_CUT_CHOICE)
#define REQ_NONE ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REQUEST_NONE_CHOICE)
#define REQ_SHUTDOWN ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REQUEST_SHUTDOWN_CHOICE)
#define REASON_NONE ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_NONE_CHOICE)
#define REASON_KEY_OFF ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_KEY_OFF_CHOICE)
#define REASON_LOW ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_LOW_VOLTAGE_CHOICE)

static keyoff_t k;
static keyoff_in_t in;
static keyoff_out_t out;

void setUp(void)
{
    keyoff_init(&k);
    memset(&in, 0, sizeof in);
    in.kl15_valid = true;
    in.kl15_on = true;
    in.engine_stopped = true;
    in.battery_valid = true;
    in.battery_mv = 13800u;
}

void tearDown(void) {}

static void step(uint32_t now_ms)
{
    keyoff_step(&k, &in, now_ms, &out);
}

/* KL15 falls at now_ms; the sequence starts one debounce later, which is returned. */
static uint32_t key_off_at(uint32_t now_ms)
{
    in.kl15_on = false;
    step(now_ms);
    step(now_ms + KEYOFF_KEY_DEBOUNCE_MS);
    return now_ms + KEYOFF_KEY_DEBOUNCE_MS;
}

static void test_running_holds_and_requests_nothing(void)
{
    step(0u);
    TEST_ASSERT_TRUE(out.hold);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    TEST_ASSERT_FALSE(out.close_logs);
}

static void test_zeroed_state_holds(void)
{
    keyoff_t z;
    memset(&z, 0, sizeof z);
    keyoff_step(&z, &in, 0u, &out);
    TEST_ASSERT_TRUE(out.hold);
}

static void test_key_off_requests_a_shutdown_with_deadline(void)
{
    const uint32_t t0 = key_off_at(1000u);
    TEST_ASSERT_TRUE(out.hold);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    TEST_ASSERT_EQUAL_UINT8(REASON_KEY_OFF, out.reason);
    TEST_ASSERT_EQUAL_UINT8(1u, out.request_id);
    TEST_ASSERT_EQUAL_UINT8(25u, out.deadline_s);
    TEST_ASSERT_TRUE(out.close_logs);
    step(t0 + 10000u);
    TEST_ASSERT_EQUAL_UINT8(15u, out.deadline_s);
}

static void ack(uint8_t state, uint8_t id)
{
    in.ack_fresh = true;
    in.ack_id = id;
    in.ack_state = state;
}

static void test_ready_ack_with_matching_id_releases_hold(void)
{
    const uint32_t t0 = key_off_at(0u);
    ack(ACK_HALTING, out.request_id);
    step(t0 + 100u);
    TEST_ASSERT_TRUE(out.hold);
    ack(ACK_READY, out.request_id);
    step(t0 + 2000u);
    TEST_ASSERT_FALSE(out.hold);
}

static void test_ack_of_another_sequence_is_ignored(void)
{
    const uint32_t t0 = key_off_at(0u);
    ack(ACK_READY, (uint8_t)(out.request_id + 1u));
    step(t0 + 100u);
    TEST_ASSERT_TRUE(out.hold);
}

static void test_stale_ack_is_ignored(void)
{
    const uint32_t t0 = key_off_at(0u);
    ack(ACK_READY, out.request_id);
    in.ack_fresh = false;
    step(t0 + 100u);
    TEST_ASSERT_TRUE(out.hold);
}

static void test_a_hung_pi_does_not_hold_the_supply_past_the_timeout(void)
{
    const uint32_t t0 = key_off_at(0u);
    step(t0 + KEYOFF_TIMEOUT_MS - 1u);
    TEST_ASSERT_TRUE(out.hold);
    step(t0 + KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_FALSE(out.hold);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    TEST_ASSERT_EQUAL_UINT8(0u, out.deadline_s);
}

static void test_ready_in_the_pass_of_the_timeout_is_done_either_way(void)
{
    const uint32_t t0 = key_off_at(0u);
    ack(ACK_READY, out.request_id);
    step(t0 + KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_FALSE(out.hold);
}

static void test_timeout_is_wrap_safe(void)
{
    const uint32_t t0 = key_off_at(0xFFFFF000u);
    step(t0 + KEYOFF_TIMEOUT_MS - 1u);
    TEST_ASSERT_TRUE(out.hold);
    step(t0 + KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_FALSE(out.hold);
}

static void test_invalid_kl15_during_shutdown_neither_cancels_nor_extends(void)
{
    const uint32_t t0 = key_off_at(0u);
    in.kl15_valid = false;
    step(t0 + 1000u);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    step(t0 + KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_FALSE(out.hold);
}

static void test_a_short_kl15_glitch_starts_nothing(void)
{
    in.kl15_on = false;
    step(0u);
    in.kl15_on = true;
    step(KEYOFF_KEY_DEBOUNCE_MS - 1u);
    step(KEYOFF_KEY_DEBOUNCE_MS * 4u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_key_off_with_the_engine_running_is_vetoed(void)
{
    in.engine_running = true;
    in.kl15_on = false;
    step(0u);
    step(KEYOFF_KEY_DEBOUNCE_MS * 10u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_done_returns_to_running_on_a_debounced_key_on(void)
{
    const uint32_t t0 = key_off_at(0u);
    step(t0 + KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_TRUE(out.close_logs);
    in.kl15_on = true;
    step(t0 + KEYOFF_TIMEOUT_MS + 10u);
    TEST_ASSERT_TRUE(out.hold); /* KL15 valid and on holds regardless of the phase */
    step(t0 + KEYOFF_TIMEOUT_MS + 10u + KEYOFF_KEY_DEBOUNCE_MS);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    TEST_ASSERT_FALSE(out.close_logs);
}

static void test_key_back_before_halting_cancels(void)
{
    const uint32_t t0 = key_off_at(0u);
    in.kl15_on = true;
    step(t0 + 100u);
    step(t0 + 100u + KEYOFF_KEY_DEBOUNCE_MS);
    TEST_ASSERT_TRUE(out.hold);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    TEST_ASSERT_FALSE(out.close_logs);
}

static void test_key_back_after_halting_does_not_cancel(void)
{
    const uint32_t t0 = key_off_at(0u);
    ack(ACK_HALTING, out.request_id);
    step(t0 + 100u);
    in.kl15_on = true;
    step(t0 + 200u);
    step(t0 + 200u + KEYOFF_KEY_DEBOUNCE_MS);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
}

static void test_request_id_increments_per_sequence_and_wraps(void)
{
    uint8_t last = 0u;
    for (uint32_t i = 0u; i < 17u; i++) {
        const uint32_t base = i * 10000u;
        (void)key_off_at(base);
        last = out.request_id;
        in.kl15_on = true;
        step(base + 1000u);
        step(base + 1000u + KEYOFF_KEY_DEBOUNCE_MS); /* cancelled */
    }
    TEST_ASSERT_EQUAL_UINT8(1u, last); /* 17 sequences, 4-bit counter */
}

static void test_invalid_kl15_never_starts_a_shutdown(void)
{
    in.kl15_valid = false;
    in.kl15_on = false;
    step(0u);
    step(KEYOFF_KEY_DEBOUNCE_MS * 10u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    TEST_ASSERT_TRUE(out.hold);
}

static void test_low_voltage_needs_the_debounce(void)
{
    in.battery_mv = KEYOFF_LOW_VOLTAGE_MV - 1u;
    step(0u);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS - 1u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    TEST_ASSERT_EQUAL_UINT8(REASON_LOW, out.reason);
}

static void test_low_voltage_never_trips_while_the_engine_runs_or_is_unknown(void)
{
    in.battery_mv = 11000u;
    in.engine_stopped = false;
    step(0u);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS * 3u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_low_voltage_dip_that_recovers_resets_the_debounce(void)
{
    in.battery_mv = 11000u;
    step(0u);
    step(8000u);
    in.battery_mv = 13500u;
    step(9000u);
    in.battery_mv = 11000u;
    step(10000u);
    step(10000u + KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS - 1u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_stale_low_voltage_debounce_does_not_survive_a_cancelled_key_off(void)
{
    in.battery_mv = 11000u;
    step(0u);
    (void)key_off_at(1000u); /* sequence starts, low timer cleared */
    in.kl15_on = true;
    step(20000u);
    step(20000u + KEYOFF_KEY_DEBOUNCE_MS); /* cancelled, back to RUNNING */
    step(20000u + KEYOFF_KEY_DEBOUNCE_MS + 1u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_exactly_at_the_cutoff_is_not_low(void)
{
    in.battery_mv = KEYOFF_LOW_VOLTAGE_MV;
    step(0u);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS * 2u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_invalid_battery_never_starts_a_shutdown(void)
{
    in.battery_valid = false;
    in.battery_mv = 0u;
    step(0u);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS * 3u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_low_voltage_shutdown_is_bounded_too(void)
{
    in.battery_mv = 11000u;
    step(0u);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_TRUE(out.hold); /* KL15 is on: the diode-OR keeps the supply regardless */
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
}

static void test_key_off_wins_over_low_voltage_reason(void)
{
    in.battery_mv = 11000u;
    step(0u);
    in.kl15_on = false;
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS);
    step(KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + KEYOFF_KEY_DEBOUNCE_MS);
    TEST_ASSERT_EQUAL_UINT8(REASON_KEY_OFF, out.reason);
}

static void test_a_corrupted_phase_is_bounded(void)
{
    k.phase = (keyoff_phase_t)77;
    step(0u);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    in.kl15_on = false;
    in.kl15_valid = false;
    step(KEYOFF_TIMEOUT_MS);
    TEST_ASSERT_FALSE(out.hold);
}

/* Round 2 MAJOR-1: at a realistic 10 ms cadence a low-voltage sequence starts once, keeps its
 * request and id, and is not cancelled by the KL15 level that was on all along. */
static void test_low_voltage_at_a_real_cadence_starts_once_and_holds_its_request(void)
{
    uint32_t t;
    uint32_t starts = 0u;
    uint8_t id = 0u;
    bool was_shutdown = false;
    in.battery_mv = 11800u;
    for (t = 0u; t <= 60000u; t += 10u) {
        step(t);
        const bool shutdown = (out.request == REQ_SHUTDOWN);
        if (shutdown && !was_shutdown) {
            starts++;
            id = out.request_id;
        }
        if (was_shutdown) {
            TEST_ASSERT_TRUE(shutdown);
            TEST_ASSERT_EQUAL_UINT8(id, out.request_id);
            TEST_ASSERT_EQUAL_UINT8(REASON_LOW, out.reason);
        }
        was_shutdown = shutdown;
    }
    TEST_ASSERT_EQUAL_UINT32(1u, starts);
}

static void test_low_voltage_done_is_latched_while_nothing_changes(void)
{
    uint32_t t;
    in.battery_mv = 11800u;
    for (t = 0u; t <= KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + 20u; t += 10u) {
        step(t);
    }
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    in.ack_fresh = true;
    in.ack_id = out.request_id;
    in.ack_state = ACK_READY;
    for (; t <= 60000u; t += 10u) {
        step(t);
        TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request); /* DONE keeps asking, never RUNNING */
        TEST_ASSERT_EQUAL_UINT8(0u, out.deadline_s);
    }
    in.engine_stopped = false;
    in.engine_running = true; /* the rider starts the engine: reopen */
    step(t);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_low_voltage_before_halting_is_cleared_by_engine_or_recovery(void)
{
    uint32_t t;
    in.battery_mv = 11800u;
    for (t = 0u; t <= KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + 20u; t += 10u) {
        step(t);
    }
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    in.battery_mv = KEYOFF_LOW_VOLTAGE_RECOVER_MV - 1u; /* above the cut-off, below hysteresis */
    step(t + 10u);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    in.battery_mv = KEYOFF_LOW_VOLTAGE_RECOVER_MV;
    step(t + 20u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_low_voltage_after_halting_is_not_cleared(void)
{
    uint32_t t;
    in.battery_mv = 11800u;
    for (t = 0u; t <= KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + 20u; t += 10u) {
        step(t);
    }
    in.ack_fresh = true;
    in.ack_id = out.request_id;
    in.ack_state = ACK_HALTING;
    step(t + 10u);
    in.battery_mv = 13800u;
    step(t + 20u);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
}

static void test_key_cycle_during_a_low_voltage_sequence_cancels(void)
{
    uint32_t t;
    in.battery_mv = 11800u;
    for (t = 0u; t <= KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + 20u; t += 10u) {
        step(t);
    }
    in.kl15_on = false; /* off for the debounce: the off half of a real key cycle */
    step(t + 10u);
    step(t + 10u + KEYOFF_KEY_DEBOUNCE_MS);
    in.kl15_on = true;
    step(t + 20u + KEYOFF_KEY_DEBOUNCE_MS);
    step(t + 20u + (2u * KEYOFF_KEY_DEBOUNCE_MS));
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

static void test_key_off_debounce_boundary_at_a_real_cadence(void)
{
    uint32_t t;
    in.kl15_on = false;
    for (t = 0u; t < KEYOFF_KEY_DEBOUNCE_MS; t += 10u) {
        step(t);
        TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    }
    step(KEYOFF_KEY_DEBOUNCE_MS);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
}

static uint32_t low_voltage_sequence(void)
{
    uint32_t t;
    in.battery_mv = 11800u;
    for (t = 0u; t <= KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS + 20u; t += 10u) {
        step(t);
    }
    return t;
}

/* Round 3 MINOR-A: a single 10 ms KL15-off sample is a glitch, not a key cycle. */
static void test_a_short_kl15_off_glitch_is_not_a_key_cycle_in_a_low_voltage_sequence(void)
{
    uint32_t t = low_voltage_sequence();
    const uint8_t id = out.request_id;
    in.kl15_on = false;
    step(t);
    in.kl15_on = true;
    for (t += 10u; t <= 30000u; t += 10u) {
        step(t);
        TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
        TEST_ASSERT_EQUAL_UINT8(id, out.request_id);
    }
}

static void test_key_cycle_off_half_debounce_boundary(void)
{
    uint32_t t = low_voltage_sequence();
    uint32_t end;
    in.kl15_on = false;
    for (end = t + KEYOFF_KEY_DEBOUNCE_MS - 10u; t < end; t += 10u) {
        step(t); /* 490 ms off */
    }
    in.kl15_on = true;
    for (end = t + KEYOFF_KEY_DEBOUNCE_MS + 10u; t <= end; t += 10u) {
        step(t);
    }
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request); /* not a key cycle */
    in.kl15_on = false;
    for (end = t + KEYOFF_KEY_DEBOUNCE_MS; t <= end; t += 10u) {
        step(t); /* 500 ms off */
    }
    in.kl15_on = true;
    for (end = t + KEYOFF_KEY_DEBOUNCE_MS; t <= end; t += 10u) {
        step(t);
    }
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request); /* a real key cycle cancels */
}

static void test_a_corrupted_phase_with_kl15_on_returns_to_running(void)
{
    step(0u);
    step(KEYOFF_KEY_DEBOUNCE_MS);
    const uint8_t id = out.request_id;
    k.phase = (keyoff_phase_t)7;
    step(KEYOFF_KEY_DEBOUNCE_MS + 10u);
    step(KEYOFF_KEY_DEBOUNCE_MS + 20u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)((id + 1u) & 0x0Fu), out.request_id);
}

static void test_low_voltage_recovery_hysteresis_boundary(void)
{
    uint32_t t = low_voltage_sequence();
    in.battery_valid = false;
    in.battery_mv = 14000u; /* an invalid high sample never clears */
    step(t);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    in.battery_valid = true;
    in.battery_mv = KEYOFF_LOW_VOLTAGE_RECOVER_MV - 1u;
    step(t + 10u);
    TEST_ASSERT_EQUAL_UINT8(REQ_SHUTDOWN, out.request);
    in.battery_mv = KEYOFF_LOW_VOLTAGE_RECOVER_MV;
    step(t + 20u);
    TEST_ASSERT_EQUAL_UINT8(REQ_NONE, out.request);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_running_holds_and_requests_nothing);
    RUN_TEST(test_zeroed_state_holds);
    RUN_TEST(test_key_off_requests_a_shutdown_with_deadline);
    RUN_TEST(test_ready_ack_with_matching_id_releases_hold);
    RUN_TEST(test_ack_of_another_sequence_is_ignored);
    RUN_TEST(test_stale_ack_is_ignored);
    RUN_TEST(test_a_hung_pi_does_not_hold_the_supply_past_the_timeout);
    RUN_TEST(test_ready_in_the_pass_of_the_timeout_is_done_either_way);
    RUN_TEST(test_timeout_is_wrap_safe);
    RUN_TEST(test_invalid_kl15_during_shutdown_neither_cancels_nor_extends);
    RUN_TEST(test_a_short_kl15_glitch_starts_nothing);
    RUN_TEST(test_key_off_with_the_engine_running_is_vetoed);
    RUN_TEST(test_done_returns_to_running_on_a_debounced_key_on);
    RUN_TEST(test_key_back_before_halting_cancels);
    RUN_TEST(test_key_back_after_halting_does_not_cancel);
    RUN_TEST(test_request_id_increments_per_sequence_and_wraps);
    RUN_TEST(test_invalid_kl15_never_starts_a_shutdown);
    RUN_TEST(test_low_voltage_needs_the_debounce);
    RUN_TEST(test_low_voltage_never_trips_while_the_engine_runs_or_is_unknown);
    RUN_TEST(test_low_voltage_dip_that_recovers_resets_the_debounce);
    RUN_TEST(test_stale_low_voltage_debounce_does_not_survive_a_cancelled_key_off);
    RUN_TEST(test_exactly_at_the_cutoff_is_not_low);
    RUN_TEST(test_invalid_battery_never_starts_a_shutdown);
    RUN_TEST(test_low_voltage_shutdown_is_bounded_too);
    RUN_TEST(test_key_off_wins_over_low_voltage_reason);
    RUN_TEST(test_a_corrupted_phase_is_bounded);
    RUN_TEST(test_low_voltage_at_a_real_cadence_starts_once_and_holds_its_request);
    RUN_TEST(test_low_voltage_done_is_latched_while_nothing_changes);
    RUN_TEST(test_low_voltage_before_halting_is_cleared_by_engine_or_recovery);
    RUN_TEST(test_low_voltage_after_halting_is_not_cleared);
    RUN_TEST(test_key_cycle_during_a_low_voltage_sequence_cancels);
    RUN_TEST(test_key_off_debounce_boundary_at_a_real_cadence);
    RUN_TEST(test_a_short_kl15_off_glitch_is_not_a_key_cycle_in_a_low_voltage_sequence);
    RUN_TEST(test_key_cycle_off_half_debounce_boundary);
    RUN_TEST(test_a_corrupted_phase_with_kl15_on_returns_to_running);
    RUN_TEST(test_low_voltage_recovery_hysteresis_boundary);
    (void)ACK_NONE;
    return UNITY_END();
}
