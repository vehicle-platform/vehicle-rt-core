/* The derived glue (D-070): vehicle_signals -> derived_inputs -> core, as the tasks run. */
#include "features/derived/derived.h"
#include "services/derived_inputs.h"
#include "services/ride_records.h"
#include "services/tester_epoch.h"
#include "services/vehicle_signals.h"

#include <unity.h>

void setUp(void)
{
    vehicle_signals_init();
    tester_epoch_set(false, 0u);
    derived_inputs_init();
    ride_records_init();
    derived_init();
}
void tearDown(void) {}

static void test_nothing_published_means_no_data(void)
{
    derived_step(0u);
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o.status);
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_WHEEL_SLIP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NOT_COMPUTED, o.status);
    TEST_ASSERT_EQUAL_UINT32(28u + 12u, derived_core()->computed_count); /* class A + the class-B scalars */
}

static void test_a_published_sample_reaches_the_outputs(void)
{
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, 32768u, 1.0f, 100u);
    derived_inputs_publish(100u);
    derived_step(100u);
    derived_out_t o;
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 14.7f, o.value);
    TEST_ASSERT_EQUAL_UINT32(100u, o.stamp_ms);
    /* the comms task stops publishing: the sample ages out by its own stamp */
    derived_step(100u + 6000u + 2000u);
    TEST_ASSERT_TRUE(derived_get(VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_INVALID, o.status);
    TEST_ASSERT_EQUAL_INT(DERIVED_FLAG_STALE_INPUT, o.flag);
}

static void test_derived_get_refuses_bad_arguments(void)
{
    derived_out_t o;
    TEST_ASSERT_FALSE(derived_get(VEHICLE_CL250_DERIVED_COUNT, &o));
    TEST_ASSERT_FALSE(derived_get(0u, NULL));
}

static void test_a_ride_follows_the_published_epoch(void)
{
    derived_out_t o;
    tester_epoch_set(true, 1u);
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_COOLANT_TEMP, 100u, 60.0f, 100u);
    derived_inputs_publish(100u);
    derived_step(100u);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_CURRENT, VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 60.0f, o.value);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o.status);
    /* the ECU goes absent: the epoch ends and the ride closes */
    tester_epoch_set(false, 1u);
    derived_inputs_publish(200u);
    derived_step(200u);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_CURRENT, VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_NO_DATA, o.status);
    TEST_ASSERT_TRUE(derived_ride_get(DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP, &o));
    TEST_ASSERT_EQUAL_INT(DERIVED_VALID, o.status);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 60.0f, o.value);
    TEST_ASSERT_FALSE(derived_ride_get(DERIVED_RIDE_PREVIOUS, VEHICLE_CL250_DERIVED_COUNT, &o));
    TEST_ASSERT_FALSE(derived_ride_get(DERIVED_RIDE_PREVIOUS, 0u, NULL));
    tester_epoch_set(false, 0u);
}

/* D-098 through the glue: the derived task publishes the records only at a ride close, the
 * comms side takes them, and the server's read sees the last CLOSED ride. */
static uint8_t usage[PLATFORM_UDS_DID_RIDE_USAGE_LENGTH];
static uint8_t thermal[PLATFORM_UDS_DID_RIDE_THERMAL_LENGTH];

static void take_and_read(void)
{
    ride_records_take(); /* the comms pass */
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage));
    TEST_ASSERT_TRUE(ride_records_read(PLATFORM_UDS_IDX_RIDE_THERMAL, thermal, sizeof thermal));
}

static uint16_t usage_counter(void)
{
    return (uint16_t)((usage[PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE] << 8u) |
                      usage[PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE + 1u]);
}

static bool usage_closed(void)
{
    return (usage[PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE] & PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_MASK) != 0u;
}

static uint32_t start_coolant_raw(void)
{
    const uint32_t b = PLATFORM_UDS_RIDE_THERMAL_RIDE_START_COOLANT_TEMP_BYTE;
    uint32_t v = 0u;
    for (uint32_t i = 0u; i < PLATFORM_UDS_RIDE_THERMAL_RIDE_START_COOLANT_TEMP_LENGTH; i++) {
        v = (v << 8u) | thermal[b + i];
    }
    return v;
}

static void ride_sample(bool open, uint16_t counter, float coolant, uint32_t t)
{
    tester_epoch_set(open, counter);
    (void)vehicle_signals_write(VEHICLE_CL250_IDX_COOLANT_TEMP, 100u, coolant, t);
    derived_inputs_publish(t);
    derived_step(t);
}

static void test_ride_records_follow_the_last_closed_ride(void)
{
    const platform_uds_ride_field_t* f = NULL;
    for (uint32_t i = 0u; i < PLATFORM_UDS_RIDE_FIELD_COUNT; i++) {
        if (platform_uds_ride_fields[i].signal == VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP) {
            f = &platform_uds_ride_fields[i];
        }
    }
    TEST_ASSERT_NOT_NULL(f);
    take_and_read(); /* boot: no ride closed */
    TEST_ASSERT_FALSE(usage_closed());
    TEST_ASSERT_EQUAL_UINT16(0u, usage_counter());
    TEST_ASSERT_EQUAL_UINT32(f->na, start_coolant_raw());

    ride_sample(true, 1u, 60.0f, 100u); /* ride 0 (counter 1) open: nothing published */
    take_and_read();
    TEST_ASSERT_FALSE(usage_closed());

    ride_sample(true, 2u, 80.0f, 200u); /* ride 0 closes, ride 1 (counter 2) opens */
    take_and_read();
    TEST_ASSERT_TRUE(usage_closed());
    TEST_ASSERT_EQUAL_UINT16(1u, usage_counter());
    TEST_ASSERT_EQUAL_UINT32(ride_records_field_raw(f, true, 60.0f), start_coolant_raw());

    ride_sample(true, 2u, 85.0f, 300u); /* ride 1 still open: still ride 0's record */
    take_and_read();
    TEST_ASSERT_EQUAL_UINT16(1u, usage_counter());
    TEST_ASSERT_EQUAL_UINT32(ride_records_field_raw(f, true, 60.0f), start_coolant_raw());

    ride_sample(false, 2u, 85.0f, 400u); /* ride 1 closes */
    take_and_read();
    TEST_ASSERT_TRUE(usage_closed());
    TEST_ASSERT_EQUAL_UINT16(2u, usage_counter());
    TEST_ASSERT_EQUAL_UINT32(ride_records_field_raw(f, true, 80.0f), start_coolant_raw());
    TEST_ASSERT_EQUAL_UINT32(0u, ride_records_skipped_count());
    tester_epoch_set(false, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_published_means_no_data);
    RUN_TEST(test_a_published_sample_reaches_the_outputs);
    RUN_TEST(test_derived_get_refuses_bad_arguments);
    RUN_TEST(test_a_ride_follows_the_published_epoch);
    RUN_TEST(test_ride_records_follow_the_last_closed_ride);
    return UNITY_END();
}
