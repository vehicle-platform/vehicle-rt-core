/*
 * services/ride_records (D-098 item 6): the derived -> comms hand-over of the ride records.
 * Before ride_records_init() (this test's first case, one process): publish counts and does
 * nothing, take does nothing, read fails. After it: the no-ride records, a publish reaches the
 * server's read only through the comms take, and the read never guesses a length.
 */
#include "services/ride_records.h"
#include "vehicle_cl250_derived.h"

#include <string.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static uint8_t usage[PLATFORM_UDS_DID_RIDE_USAGE_LENGTH];

static bool stop_count_7(uint32_t signal, float* value)
{
    *value = 7.0f;
    return signal == VEHICLE_CL250_DERIVED_RIDE_STOP_COUNT;
}

static uint32_t stop_count_raw(void)
{
    const uint32_t b = PLATFORM_UDS_RIDE_USAGE_RIDE_STOP_COUNT_BYTE;
    uint32_t v = 0u;
    for (uint32_t i = 0u; i < PLATFORM_UDS_RIDE_USAGE_RIDE_STOP_COUNT_LENGTH; i++) {
        v = (v << 8u) | usage[b + i];
    }
    return v;
}

static void test_before_init_nothing_is_published_or_served(void)
{
    TEST_ASSERT_FALSE(ride_records_publish(stop_count_7, true, 3u));
    TEST_ASSERT_FALSE(ride_records_publish(stop_count_7, true, 4u));
    TEST_ASSERT_EQUAL_UINT32(2u, ride_records_skipped_count());
    ride_records_take();
    TEST_ASSERT_FALSE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
}

static void test_a_publish_reaches_the_read_only_through_the_take(void)
{
    ride_records_init();
    TEST_ASSERT_EQUAL_UINT32(0u, ride_records_skipped_count());
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
    TEST_ASSERT_EQUAL_HEX8(0u, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE]);
    TEST_ASSERT_EQUAL_HEX8(0u, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE + 1u]);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_UDS_RIDE_USAGE_RIDE_STOP_COUNT_NA, stop_count_raw());
    ride_records_take(); /* nothing new: the boot records stay */
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
    TEST_ASSERT_EQUAL_HEX8(0u, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE]);

    TEST_ASSERT_TRUE(ride_records_publish(stop_count_7, true, 3u));
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
    TEST_ASSERT_EQUAL_HEX8(0u, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE]); /* not taken yet */
    ride_records_take();
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
    TEST_ASSERT_EQUAL_HEX8(PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_MASK, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE]);
    TEST_ASSERT_EQUAL_HEX8(3u, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE + 1u]);
    TEST_ASSERT_EQUAL_UINT32(7u, stop_count_raw());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_UDS_RIDE_USAGE_RIDE_DISTANCE_NA,
                             (uint32_t)usage[PLATFORM_UDS_RIDE_USAGE_RIDE_DISTANCE_BYTE] << 24u |
                                 (uint32_t)usage[PLATFORM_UDS_RIDE_USAGE_RIDE_DISTANCE_BYTE + 1u] << 16u |
                                 (uint32_t)usage[PLATFORM_UDS_RIDE_USAGE_RIDE_DISTANCE_BYTE + 2u] << 8u |
                                 (uint32_t)usage[PLATFORM_UDS_RIDE_USAGE_RIDE_DISTANCE_BYTE + 3u]);

    /* two publishes before one take: the newest wins, nothing is queued */
    TEST_ASSERT_TRUE(ride_records_publish(stop_count_7, true, 4u));
    TEST_ASSERT_TRUE(ride_records_publish(stop_count_7, true, 5u));
    ride_records_take();
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
    TEST_ASSERT_EQUAL_HEX8(5u, usage[PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE + 1u]);
    TEST_ASSERT_FALSE(ride_records_producer_fault());
    TEST_ASSERT_FALSE(ride_records_consumer_fault());
}

static void test_the_read_refuses_a_wrong_length_or_another_did(void)
{
    ride_records_init();
    uint8_t big[PLATFORM_UDS_DID_RIDE_USAGE_LENGTH + 1u];
    TEST_ASSERT_FALSE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, big, sizeof big));
    TEST_ASSERT_FALSE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, big, PLATFORM_UDS_DID_RIDE_USAGE_LENGTH - 1u));
    TEST_ASSERT_FALSE(ride_records_read(PLATFORM_UDS_IDX_LEAN_HEALTH, big, PLATFORM_UDS_DID_RIDE_USAGE_LENGTH));
    TEST_ASSERT_FALSE(ride_records_read(PLATFORM_UDS_DID_COUNT, big, PLATFORM_UDS_DID_RIDE_USAGE_LENGTH));
    TEST_ASSERT_FALSE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, NULL, PLATFORM_UDS_DID_RIDE_USAGE_LENGTH));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_before_init_nothing_is_published_or_served); /* first: needs the un-initialised module */
    RUN_TEST(test_a_publish_reaches_the_read_only_through_the_take);
    RUN_TEST(test_the_read_refuses_a_wrong_length_or_another_did);
    return UNITY_END();
}
