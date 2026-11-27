/*
 * L0 tests for hal/reset_cause (pure RCC_RSR decode): each flag alone, the multi-flag priority
 * of real resets (every reset also sets the pin flag, a power-on reset also sets BOR), the
 * ignored RMVF bit and the unknown case.
 */
#include "hal/reset_cause.h"

#include <stdint.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_single_flags(void)
{
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(RESET_CAUSE_RSR_IWDG1RSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(RESET_CAUSE_RSR_WWDG1RSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOCKUP, reset_cause_decode(RESET_CAUSE_RSR_CPURSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(RESET_CAUSE_RSR_LPWRRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(RESET_CAUSE_RSR_D1RSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(RESET_CAUSE_RSR_D2RSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(RESET_CAUSE_RSR_SFTRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON, reset_cause_decode(RESET_CAUSE_RSR_PORRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_BROWN_OUT, reset_cause_decode(RESET_CAUSE_RSR_BORRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(RESET_CAUSE_RSR_PINRSTF));
}

static void test_real_resets_set_the_pin_flag_too(void)
{
    /* Every reset pulls NRST, so the pin flag accompanies the real cause. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(RESET_CAUSE_RSR_IWDG1RSTF | RESET_CAUSE_RSR_PINRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(RESET_CAUSE_RSR_SFTRSTF | RESET_CAUSE_RSR_PINRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON,
                          reset_cause_decode(RESET_CAUSE_RSR_PORRSTF | RESET_CAUSE_RSR_BORRSTF |
                                             RESET_CAUSE_RSR_PINRSTF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_BROWN_OUT,
                          reset_cause_decode(RESET_CAUSE_RSR_BORRSTF | RESET_CAUSE_RSR_PINRSTF));
}

static void test_priority_order(void)
{
    uint32_t rsr = 0xFFFFFFFFu;

    /* All flags set: peel them off from the most specific one. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_IWDG1RSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_WWDG1RSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOCKUP, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_CPURSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(rsr));
    rsr &= ~(RESET_CAUSE_RSR_LPWRRSTF | RESET_CAUSE_RSR_D1RSTF | RESET_CAUSE_RSR_D2RSTF);
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_SFTRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_PORRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_BROWN_OUT, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_BORRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_PINRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(rsr));
}

static void test_unknown_and_ignored_bits(void)
{
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(0u));
    /* RMVF is a write-only clear bit, reserved bits carry no cause. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(RESET_CAUSE_RSR_RMVF));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode(0x0000FFFFu));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_UNKNOWN, reset_cause_decode((1u << 18) | (1u << 25) | (1u << 27) | (1u << 29) | (1u << 31)));
}

static void test_bit_positions_match_rm0433(void)
{
    TEST_ASSERT_EQUAL_HEX32(0x00010000u, RESET_CAUSE_RSR_RMVF);
    TEST_ASSERT_EQUAL_HEX32(0x00020000u, RESET_CAUSE_RSR_CPURSTF);
    TEST_ASSERT_EQUAL_HEX32(0x00200000u, RESET_CAUSE_RSR_BORRSTF);
    TEST_ASSERT_EQUAL_HEX32(0x00400000u, RESET_CAUSE_RSR_PINRSTF);
    TEST_ASSERT_EQUAL_HEX32(0x00800000u, RESET_CAUSE_RSR_PORRSTF);
    TEST_ASSERT_EQUAL_HEX32(0x01000000u, RESET_CAUSE_RSR_SFTRSTF);
    TEST_ASSERT_EQUAL_HEX32(0x04000000u, RESET_CAUSE_RSR_IWDG1RSTF);
    TEST_ASSERT_EQUAL_HEX32(0x10000000u, RESET_CAUSE_RSR_WWDG1RSTF);
    TEST_ASSERT_EQUAL_HEX32(0x40000000u, RESET_CAUSE_RSR_LPWRRSTF);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_single_flags);
    RUN_TEST(test_real_resets_set_the_pin_flag_too);
    RUN_TEST(test_priority_order);
    RUN_TEST(test_unknown_and_ignored_bits);
    RUN_TEST(test_bit_positions_match_rm0433);
    return UNITY_END();
}
