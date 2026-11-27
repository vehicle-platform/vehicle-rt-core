/*
 * L0 tests for hal/reset_cause (pure RCC_RSR decode): the RM0433 reset-source rows with their
 * realistic multi-flag sets, the priority order, the ignored RMVF bit and the unknown case.
 */
#include "hal/reset_cause.h"

#include <stdint.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

#define CPU RESET_CAUSE_RSR_CPURSTF
#define PIN RESET_CAUSE_RSR_PINRSTF
#define D1 RESET_CAUSE_RSR_D1RSTF
#define D2 RESET_CAUSE_RSR_D2RSTF

static void test_rm0433_reset_source_rows(void)
{
    /* Realistic flag sets per reset source (RM0433 reset-source table): every system reset sets
     * CPURSTF, every NRST-driving reset sets PINRSTF, a power-on reset sets all of BOR, D1, D2. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON,
                          reset_cause_decode(RESET_CAUSE_RSR_PORRSTF | RESET_CAUSE_RSR_BORRSTF | PIN | CPU | D1 | D2));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_BROWN_OUT,
                          reset_cause_decode(RESET_CAUSE_RSR_BORRSTF | PIN | CPU));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(PIN | CPU));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(RESET_CAUSE_RSR_SFTRSTF | PIN | CPU));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(RESET_CAUSE_RSR_IWDG1RSTF | PIN | CPU));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(RESET_CAUSE_RSR_WWDG1RSTF | PIN | CPU));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(RESET_CAUSE_RSR_LPWRRSTF | PIN | CPU));
    /* Domain standby exit: only the domain flag (no NRST pull). */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_DOMAIN_STANDBY, reset_cause_decode(D1));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_DOMAIN_STANDBY, reset_cause_decode(D2));
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_DOMAIN_STANDBY, reset_cause_decode(D1 | D2 | CPU));
    /* A bare CPU reset. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_CPU_RESET, reset_cause_decode(CPU));
}

static void test_priority_order(void)
{
    uint32_t rsr = 0xFFFFFFFFu;

    /* All flags set: peel them off from the most specific one. */
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_IWDG, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_IWDG1RSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_WWDG, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_WWDG1RSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_LOW_POWER, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_LPWRRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_SOFTWARE, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_SFTRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_POWER_ON, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_PORRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_BROWN_OUT, reset_cause_decode(rsr));
    rsr &= ~RESET_CAUSE_RSR_BORRSTF;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_PIN, reset_cause_decode(rsr));
    rsr &= ~PIN;
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_DOMAIN_STANDBY, reset_cause_decode(rsr));
    rsr &= ~(D1 | D2);
    TEST_ASSERT_EQUAL_INT(RESET_CAUSE_CPU_RESET, reset_cause_decode(rsr));
    rsr &= ~CPU;
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

static void test_fault_record(void)
{
    uint32_t kind;

    for (kind = 1u; kind <= 4u; kind++) {
        TEST_ASSERT_EQUAL_INT((int)kind, (int)reset_fault_decode(kind, kind ^ RESET_CAUSE_FAULT_MAGIC));
    }
    /* Cleared record, random RAM, wrong check word, kind out of range. */
    TEST_ASSERT_EQUAL_INT(RESET_FAULT_NONE, reset_fault_decode(0u, 0u));
    TEST_ASSERT_EQUAL_INT(RESET_FAULT_NONE, reset_fault_decode(0xFFFFFFFFu, 0xFFFFFFFFu));
    TEST_ASSERT_EQUAL_INT(RESET_FAULT_NONE, reset_fault_decode(1u, 1u));
    TEST_ASSERT_EQUAL_INT(RESET_FAULT_NONE, reset_fault_decode(3u, 4u ^ RESET_CAUSE_FAULT_MAGIC));
    TEST_ASSERT_EQUAL_INT(RESET_FAULT_NONE, reset_fault_decode(5u, 5u ^ RESET_CAUSE_FAULT_MAGIC));
    TEST_ASSERT_EQUAL_INT(RESET_FAULT_NONE, reset_fault_decode(0u, RESET_CAUSE_FAULT_MAGIC));
}

static void test_fault_record_gate(void)
{
    TEST_ASSERT_TRUE(reset_fault_record_trusted(0u));
    TEST_ASSERT_TRUE(reset_fault_record_trusted(PIN | CPU));
    TEST_ASSERT_TRUE(reset_fault_record_trusted(RESET_CAUSE_RSR_SFTRSTF | PIN | CPU));
    TEST_ASSERT_TRUE(reset_fault_record_trusted(RESET_CAUSE_RSR_IWDG1RSTF | PIN | CPU));
    TEST_ASSERT_FALSE(reset_fault_record_trusted(RESET_CAUSE_RSR_PORRSTF));
    TEST_ASSERT_FALSE(reset_fault_record_trusted(RESET_CAUSE_RSR_BORRSTF | PIN | CPU));
    TEST_ASSERT_FALSE(reset_fault_record_trusted(D1));
    TEST_ASSERT_FALSE(reset_fault_record_trusted(D2 | CPU));
    /* A POR sets all of them. */
    TEST_ASSERT_FALSE(reset_fault_record_trusted(RESET_CAUSE_RSR_PORRSTF | RESET_CAUSE_RSR_BORRSTF | PIN | CPU | D1 | D2));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_rm0433_reset_source_rows);
    RUN_TEST(test_priority_order);
    RUN_TEST(test_unknown_and_ignored_bits);
    RUN_TEST(test_bit_positions_match_rm0433);
    RUN_TEST(test_fault_record);
    RUN_TEST(test_fault_record_gate);
    return UNITY_END();
}
