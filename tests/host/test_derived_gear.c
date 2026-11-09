/* Gear estimate (features/derived/derived_gear): the nearest gear ratio within +-5 %, plus
 * the ratio lookup and the tolerance-free nearest ratio of the D-103 clutch rule. */
#include "features/derived/derived_gear.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static const float ratios[6] = {VEHICLE_CL250_GEAR_RATIO_1, VEHICLE_CL250_GEAR_RATIO_2, VEHICLE_CL250_GEAR_RATIO_3,
                                VEHICLE_CL250_GEAR_RATIO_4, VEHICLE_CL250_GEAR_RATIO_5, VEHICLE_CL250_GEAR_RATIO_6};

static void test_every_gear_ratio_gives_its_gear(void)
{
    for (uint8_t i = 0u; i < 6u; i++) {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(i + 1u), derived_gear_estimate(ratios[i]));
    }
}

static void test_within_the_tolerance_is_the_gear_and_outside_is_none(void)
{
    TEST_ASSERT_EQUAL_UINT8(3u, derived_gear_estimate(ratios[2] * 1.04f));
    TEST_ASSERT_EQUAL_UINT8(3u, derived_gear_estimate(ratios[2] * 0.96f));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(ratios[2] * 1.06f));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(ratios[2] * 0.94f));
    TEST_ASSERT_EQUAL_UINT8(5u, derived_gear_estimate(ratios[4] * 1.04f));
    /* the geometric mean of 5th and 6th is 5.6 % / 6.0 % off both: no gear */
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(sqrtf(ratios[4] * ratios[5])));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(ratios[5] * 0.5f));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(ratios[0] * 2.0f));
}

static void test_a_bad_ratio_is_no_gear(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(NAN));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(INFINITY));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(0.0f));
    TEST_ASSERT_EQUAL_UINT8(0u, derived_gear_estimate(-ratios[2]));
}

static void test_gear_ratio_lookup(void)
{
    for (uint8_t i = 0u; i < 6u; i++) {
        TEST_ASSERT_EQUAL_FLOAT(ratios[i], derived_gear_ratio((uint8_t)(i + 1u)));
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, derived_gear_ratio(0u));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, derived_gear_ratio(7u));
}

static void test_nearest_ratio_has_no_tolerance(void)
{
    /* far outside every gear-match band, the nearest gear ratio is still returned */
    TEST_ASSERT_EQUAL_FLOAT(ratios[0], derived_gear_nearest_ratio(ratios[0] * 2.0f));
    TEST_ASSERT_EQUAL_FLOAT(ratios[5], derived_gear_nearest_ratio(ratios[5] * 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(ratios[2], derived_gear_nearest_ratio(ratios[2] * 1.06f));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_every_gear_ratio_gives_its_gear);
    RUN_TEST(test_within_the_tolerance_is_the_gear_and_outside_is_none);
    RUN_TEST(test_a_bad_ratio_is_no_gear);
    RUN_TEST(test_gear_ratio_lookup);
    RUN_TEST(test_nearest_ratio_has_no_tolerance);
    return UNITY_END();
}
