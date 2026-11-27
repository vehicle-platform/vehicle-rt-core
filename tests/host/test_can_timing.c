/*
 * L0 tests for hal/can_timing (pure FDCAN nominal bit-timing arithmetic): the cubemx/ image
 * values against the generated vehicle bit rate, a wrong prescaler, the FDCAN field ranges,
 * the SJW rule, the sample-point window and the 64-bit product.
 */
#include "hal/can_timing.h"
#include "vehicle_cl250.h"

#include <stdint.h>
#include <unity.h>

/* FDCAN kernel clock and nominal timing of cubemx/rt-core.ioc (T6b, provisional, D-134).
 * KERNEL_HZ is copied from the .ioc (PLL1Q); T13 checks it at run time with
 * HAL_RCCEx_GetPeriphCLKFreq before calling can_timing_ok(). */
#define KERNEL_HZ 80000000u
#define NOM_PRESCALER 10u
#define NOM_SEG1 13u
#define NOM_SEG2 2u
#define NOM_SJW 2u

void setUp(void) {}
void tearDown(void) {}

static void test_image_values_give_vehicle_bitrate(void)
{
    TEST_ASSERT_EQUAL_UINT32(VEHICLE_CL250_BITRATE,
                             can_timing_bitrate(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, NOM_SEG2));
    TEST_ASSERT_TRUE(can_timing_ok(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, NOM_SEG2, NOM_SJW,
                                   VEHICLE_CL250_BITRATE));
}

static void test_cubemx_default_is_rejected(void)
{
    /* The first image: prescaler 16, seg1 = seg2 = 1 gives 80 MHz / 48 = 1666666 bit/s. */
    TEST_ASSERT_EQUAL_UINT32(1666666u, can_timing_bitrate(KERNEL_HZ, 16u, 1u, 1u));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 16u, 1u, 1u, 1u, VEHICLE_CL250_BITRATE));
}

static void test_wrong_prescaler(void)
{
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, 9u, NOM_SEG1, NOM_SEG2, NOM_SJW, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, 11u, NOM_SEG1, NOM_SEG2, NOM_SJW, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, 5u, NOM_SEG1, NOM_SEG2, NOM_SJW, VEHICLE_CL250_BITRATE));
}

static void test_inexact_rate_is_rejected_but_reported_truncated(void)
{
    /* 80 MHz / (3 * 16) = 1666666.67: truncated by bitrate(), refused by ok(). */
    TEST_ASSERT_EQUAL_UINT32(1666666u, can_timing_bitrate(KERNEL_HZ, 3u, NOM_SEG1, NOM_SEG2));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 3u, NOM_SEG1, NOM_SEG2, NOM_SJW, 1666666u));
}

static void test_zero_inputs(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(KERNEL_HZ, 0u, NOM_SEG1, NOM_SEG2));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(KERNEL_HZ, NOM_PRESCALER, 0u, NOM_SEG2));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(0u, NOM_PRESCALER, NOM_SEG1, NOM_SEG2));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, 0u, NOM_SEG1, NOM_SEG2, NOM_SJW, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, NOM_PRESCALER, 0u, NOM_SEG2, NOM_SJW, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, 0u, NOM_SJW, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(can_timing_ok(0u, NOM_PRESCALER, NOM_SEG1, NOM_SEG2, NOM_SJW, 0u));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, NOM_SEG2, NOM_SJW, 0u));
}

static void test_field_range_edges(void)
{
    /* Upper edges accepted by the divisor, one past each edge refused. */
    TEST_ASSERT_EQUAL_UINT32(512u * 3u, can_timing_divisor(512u, 1u, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_divisor(513u, 1u, 1u));
    TEST_ASSERT_EQUAL_UINT32(1u * 258u, can_timing_divisor(1u, 256u, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_divisor(1u, 257u, 1u));
    TEST_ASSERT_EQUAL_UINT32(1u * 130u, can_timing_divisor(1u, 1u, 128u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_divisor(1u, 1u, 129u));
    /* The largest valid divisor. */
    TEST_ASSERT_EQUAL_UINT32(512u * 385u, can_timing_divisor(512u, 256u, 128u));
}

static void test_out_of_range_values_cannot_wrap_into_a_match(void)
{
    /* Before the range checks these values made expected_bps * divisor wrap at 2^64 and
     * returned true (safety review 2026-11-27). Now the divisor is refused first. */
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(16777216u, 257u, 3600000000u, 678255360u));
    TEST_ASSERT_FALSE(can_timing_ok(16777216u, 257u, 3600000000u, 678255360u, 1u, 16777216u));
    TEST_ASSERT_EQUAL_UINT32(0u, can_timing_bitrate(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX));
    TEST_ASSERT_FALSE(can_timing_ok(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, 1u, 1u));
    /* Largest quotient: kernel / (1 * 3 quanta) never exceeds the input. */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX / 3u, can_timing_bitrate(UINT32_MAX, 1u, 1u, 1u));
}

static void test_sjw_rule(void)
{
    /* 1 <= sjw <= seg2 (seg2 = 2 here). */
    TEST_ASSERT_TRUE(
        can_timing_ok(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, NOM_SEG2, 1u, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, NOM_SEG2, 0u, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(
        can_timing_ok(KERNEL_HZ, NOM_PRESCALER, NOM_SEG1, NOM_SEG2, 3u, VEHICLE_CL250_BITRATE));
}

static void test_sample_point_window(void)
{
    /* 16 quanta, prescaler 10: sample point = (1 + seg1) / 16. 12/16 = 75 % and 14/16 =
     * 87.5 % pass; 11/16 = 68.75 % and 15/16 = 93.75 % fail. */
    TEST_ASSERT_TRUE(can_timing_ok(KERNEL_HZ, 10u, 11u, 4u, 1u, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 10u, 10u, 5u, 1u, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 10u, 14u, 1u, 1u, VEHICLE_CL250_BITRATE));
    /* 20 quanta, prescaler 8: 18/20 = 90 % passes (upper edge), 19/20 = 95 % fails. */
    TEST_ASSERT_TRUE(can_timing_ok(KERNEL_HZ, 8u, 17u, 2u, 1u, VEHICLE_CL250_BITRATE));
    TEST_ASSERT_FALSE(can_timing_ok(KERNEL_HZ, 8u, 18u, 1u, 1u, VEHICLE_CL250_BITRATE));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_image_values_give_vehicle_bitrate);
    RUN_TEST(test_cubemx_default_is_rejected);
    RUN_TEST(test_wrong_prescaler);
    RUN_TEST(test_inexact_rate_is_rejected_but_reported_truncated);
    RUN_TEST(test_zero_inputs);
    RUN_TEST(test_field_range_edges);
    RUN_TEST(test_out_of_range_values_cannot_wrap_into_a_match);
    RUN_TEST(test_sjw_rule);
    RUN_TEST(test_sample_point_window);
    return UNITY_END();
}
