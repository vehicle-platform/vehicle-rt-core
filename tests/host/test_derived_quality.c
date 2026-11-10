/*
 * L0 tests of data_quality_mask (D-104) in the derived core: every bit, the exclusions
 * (stopped, skipped), unknown tester quality, bus_off against the previous sample, the stall
 * threshold, the sum and the arming. The rule is the description of the signal in the defs
 * (cl250_derived.yaml); the bit values, the per-DID poll periods, the stale limits and the stall
 * gap all come from gen/. The poll periods below are the test's expectation of the CL250
 * table (the formula itself never writes one).
 */
#include "features/derived/derived_core.h"
#include "vehicle_cl250.h"
#include "vehicle_cl250_constants.h"
#include "vehicle_cl250_derived.h"

#include <stdint.h>
#include <string.h>
#include <unity.h>

#define SIG VEHICLE_CL250_DERIVED_DATA_QUALITY_MASK
#define T0 (5000u)
#define STEP (VEHICLE_CL250_PERIOD_MS_DATA_QUALITY_MASK)

static derived_core_t core;
static derived_in_t in[VEHICLE_CL250_DID_COUNT];
static derived_quality_in_t q;
static uint32_t now;

typedef struct {
    uint16_t period;
    uint32_t bit;
} poll_bit_t;

static const poll_bit_t expect_bits[] = {
    {110u, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_110MS},
    {200u, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_200MS},
    {800u, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_800MS},
    {2000u, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_2000MS},
    {5000u, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_5000MS},
    {10000u, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_10000MS},
};

static uint32_t bit_of_did(uint32_t idx)
{
    for (uint32_t i = 0u; i < (sizeof expect_bits / sizeof expect_bits[0]); i++) {
        if (expect_bits[i].period == vehicle_cl250_dids[idx].poll_period_ms) {
            return expect_bits[i].bit;
        }
    }
    TEST_FAIL_MESSAGE("a DID poll period without an expected bit");
    return 0u;
}

static uint32_t all_poll_bits(void)
{
    uint32_t m = 0u;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        m |= bit_of_did(i);
    }
    return m;
}

#define BUS_OFF VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_BUS_OFF
#define STALL VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_TESTER_STALL

/* Every DID fresh at `now`, quality valid, all OK, no bus-off, no gap. */
static void healthy(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        in[i].value = 1.0f;
        in[i].stamp_ms = now;
        in[i].present = true;
    }
    memset(&q, 0, sizeof q);
    q.valid = true;
    q.stamp_ms = now;
}

static const derived_out_t* step(void)
{
    q.stamp_ms = now; /* the comms task keeps publishing */
    derived_core_set_quality(&core, &q);
    derived_core_step(&core, now, in);
    return derived_core_out(&core, SIG);
}

static uint32_t mask_now(void)
{
    const derived_out_t* o = step();
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o->status);
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_NONE, o->flag);
    TEST_ASSERT_EQUAL_UINT32(now, o->stamp_ms);
    return (uint32_t)o->value;
}

/* Next tick: time moves one period, every DID sample is refreshed unless the test changed it. */
static uint32_t next_tick(void)
{
    now += STEP;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (in[i].present && ((now - in[i].stamp_ms) < 1000000u) && (in[i].value == 1.0f)) {
            in[i].stamp_ms = now;
        }
    }
    return mask_now();
}

void setUp(void)
{
    derived_core_init(&core);
    now = T0;
    healthy();
}

void tearDown(void) {}

static void test_it_is_computed_from_the_gen_table(void)
{
    TEST_ASSERT_TRUE(derived_core_is_computed(&core, SIG));
    TEST_ASSERT_EQUAL_INT(DERIVED_NC_NONE, derived_core_reason(&core, SIG));
    TEST_ASSERT_EQUAL_UINT32(DERIVED_NO_DATA, derived_core_out(&core, SIG)->status);
    /* the gen/ bits are the eight single bits 1..128 and the table has a bit per DID period */
    TEST_ASSERT_EQUAL_UINT32(255u, all_poll_bits() | BUS_OFF | STALL);
}

static void test_without_quality_it_stays_no_data(void)
{
    derived_core_step(&core, now, in);
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, derived_core_out(&core, SIG)->status);
}

static void test_all_good_is_zero(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, mask_now());
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
}

static void test_a_never_received_did_sets_exactly_its_poll_bit(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        setUp();
        in[i].present = false; /* no sample yet */
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(bit_of_did(i), mask_now(), "never received");
    }
}

static void test_a_stale_did_sets_its_bit_one_ms_past_its_stale_after(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        setUp();
        const uint32_t limit = vehicle_cl250_dids[i].stale_after_ms;
        in[i].stamp_ms = now - limit; /* age == limit: still fresh */
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, mask_now(), "age at the limit");
        setUp();
        in[i].stamp_ms = now - limit - 1u;
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(bit_of_did(i), mask_now(), "age past the limit");
    }
}

static void test_a_negative_response_sets_its_bit_even_with_a_fresh_sample(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        setUp();
        q.did[i] = (uint8_t)DERIVED_QUALITY_NRC;
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(bit_of_did(i), mask_now(), "NRC");
    }
}

static void test_one_bad_did_per_group_and_several_groups_sum(void)
{
    /* two DIDs of the same group set one bit; two groups sum */
    uint32_t a = 0u;
    uint32_t b = 0u;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (bit_of_did(i) != bit_of_did(0u)) {
            b = i;
            break;
        }
    }
    in[a].present = false;
    q.did[b] = (uint8_t)DERIVED_QUALITY_NRC;
    TEST_ASSERT_EQUAL_UINT32(bit_of_did(a) | bit_of_did(b), mask_now());
    TEST_ASSERT_NOT_EQUAL_UINT32(bit_of_did(a), bit_of_did(b));
    /* every DID bad: all six poll bits, no health bit */
    setUp();
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        q.did[i] = (uint8_t)DERIVED_QUALITY_NRC;
    }
    TEST_ASSERT_EQUAL_UINT32(all_poll_bits(), mask_now());
}

static void test_stopped_and_skipped_dids_never_count(void)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        setUp();
        in[i].present = false; /* would be never-received ... */
        q.did[i] = (uint8_t)DERIVED_QUALITY_STOPPED;
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, mask_now(), "stopped, no sample");
        in[i].present = true;
        in[i].value = 2.0f; /* keep next_tick from refreshing it */
        in[i].stamp_ms = 0u; /* ... or stale */
        q.did[i] = (uint8_t)DERIVED_QUALITY_SKIPPED;
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, next_tick(), "skipped, stale");
    }
}

static void test_unknown_tester_quality_sets_every_poll_bit_bus_off_and_stall(void)
{
    q.valid = false; /* never set or closed */
    TEST_ASSERT_EQUAL_UINT32(255u, mask_now());
}

static void test_a_quality_record_older_than_the_stale_limit_is_unknown(void)
{
    const uint32_t limit = vehicle_cl250_derived[SIG].stale_after_ms;
    derived_core_set_quality(&core, &q);
    q.stamp_ms = now - limit; /* age == limit: usable */
    derived_core_set_quality(&core, &q);
    derived_core_step(&core, now, in);
    TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)derived_core_out(&core, SIG)->value);
    now += STEP;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        in[i].stamp_ms = now;
    }
    q.stamp_ms = now - limit - 1u; /* one ms too old: unknown */
    derived_core_set_quality(&core, &q);
    derived_core_step(&core, now, in);
    TEST_ASSERT_EQUAL_UINT32(255u, (uint32_t)derived_core_out(&core, SIG)->value);
}

static void test_bus_off_is_a_count_above_the_previous_sample(void)
{
    q.bus_off_count = 0u;
    TEST_ASSERT_EQUAL_UINT32(0u, mask_now());           /* 0 before the first sample: not above */
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
    q.bus_off_count = 1u;
    TEST_ASSERT_EQUAL_UINT32(BUS_OFF, next_tick());     /* greater than at the previous sample */
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());          /* the same count: the bit clears */
    q.bus_off_count = 3u;
    TEST_ASSERT_EQUAL_UINT32(BUS_OFF, next_tick());
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
}

static void test_a_first_sample_with_bus_offs_already_counted_sets_the_bit(void)
{
    q.bus_off_count = 2u; /* 0 before the first sample */
    TEST_ASSERT_EQUAL_UINT32(BUS_OFF, mask_now());
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
}

static void test_an_unknown_sample_keeps_the_remembered_bus_off_count(void)
{
    q.bus_off_count = 1u;
    TEST_ASSERT_EQUAL_UINT32(BUS_OFF, mask_now());
    q.valid = false;
    TEST_ASSERT_EQUAL_UINT32(255u, next_tick());
    q.valid = true;
    q.bus_off_count = 1u; /* unchanged from before the unknown sample: no new bus-off */
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
}

static void test_tester_stall_is_a_gap_above_the_gen_threshold(void)
{
    q.step_gap_ms = VEHICLE_CL250_DATA_QUALITY_TESTER_STALL_GAP_MS;
    TEST_ASSERT_EQUAL_UINT32(0u, mask_now());
    q.step_gap_ms = VEHICLE_CL250_DATA_QUALITY_TESTER_STALL_GAP_MS + 1u;
    TEST_ASSERT_EQUAL_UINT32(STALL, next_tick());
    q.step_gap_ms = 0u;
    TEST_ASSERT_EQUAL_UINT32(0u, next_tick());
}

static void test_the_value_is_the_sum_of_the_set_bits(void)
{
    in[0].present = false;
    q.did[1] = (uint8_t)DERIVED_QUALITY_NRC;
    q.bus_off_count = 1u;
    q.step_gap_ms = VEHICLE_CL250_DATA_QUALITY_TESTER_STALL_GAP_MS + 1u;
    const uint32_t expect = bit_of_did(0u) | bit_of_did(1u) | BUS_OFF | STALL;
    const uint32_t got = mask_now();
    TEST_ASSERT_EQUAL_UINT32(expect, got);
    /* the bits are distinct powers of two, so the sum equals the OR */
    uint32_t sum = 0u;
    for (uint32_t b = 0u; b < 8u; b++) {
        sum += ((got >> b) & 1u) << b;
    }
    TEST_ASSERT_EQUAL_UINT32(got, sum);
}

static void test_the_grid_is_the_gen_period_and_a_late_step_evaluates_the_latest_tick(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, mask_now());
    now += STEP - 1u; /* not due */
    (void)step();
    TEST_ASSERT_EQUAL_UINT32(T0, derived_core_out(&core, SIG)->stamp_ms);
    now += 1u;
    (void)step();
    TEST_ASSERT_EQUAL_UINT32(T0 + STEP, derived_core_out(&core, SIG)->stamp_ms);
    TEST_ASSERT_EQUAL_UINT32(2u, derived_core_out(&core, SIG)->seq);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_it_is_computed_from_the_gen_table);
    RUN_TEST(test_without_quality_it_stays_no_data);
    RUN_TEST(test_all_good_is_zero);
    RUN_TEST(test_a_never_received_did_sets_exactly_its_poll_bit);
    RUN_TEST(test_a_stale_did_sets_its_bit_one_ms_past_its_stale_after);
    RUN_TEST(test_a_negative_response_sets_its_bit_even_with_a_fresh_sample);
    RUN_TEST(test_one_bad_did_per_group_and_several_groups_sum);
    RUN_TEST(test_stopped_and_skipped_dids_never_count);
    RUN_TEST(test_unknown_tester_quality_sets_every_poll_bit_bus_off_and_stall);
    RUN_TEST(test_a_quality_record_older_than_the_stale_limit_is_unknown);
    RUN_TEST(test_bus_off_is_a_count_above_the_previous_sample);
    RUN_TEST(test_a_first_sample_with_bus_offs_already_counted_sets_the_bit);
    RUN_TEST(test_an_unknown_sample_keeps_the_remembered_bus_off_count);
    RUN_TEST(test_tester_stall_is_a_gap_above_the_gen_threshold);
    RUN_TEST(test_the_value_is_the_sum_of_the_set_bits);
    RUN_TEST(test_the_grid_is_the_gen_period_and_a_late_step_evaluates_the_latest_tick);
    return UNITY_END();
}
