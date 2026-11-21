/*
 * L0 tests for the pure safety-heartbeat monitor (features/safety_hb, D-118 item 6):
 * the lost verdict is the generated E2E timeout (NO_NEW_DATA) and nothing else, a single
 * E2E error is only a link-health count, and WRONG_LENGTH (defs v0.30.0) is an invalid
 * frame that leaves the E2E state alone. Frames are built with the core vehicle_e2e_protect
 * and the generated DataID: rt-core has no 0x082 pack of its own (it never sends it).
 */
#include "features/safety_hb/safety_hb_core.h"
#include "platform.h"
#include "platform_e2e.h"
#include "vehicle_e2e.h"

#include <string.h>
#include <unity.h>

#define T0 1000u
#define CYCLE PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS
#define TIMEOUT PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS

static safety_hb_core_t core;
static vehicle_e2e_tx_state_t tx;

void setUp(void)
{
    safety_hb_core_init(&core);
    (void)safety_hb_core_lost(&core, T0); /* the first pass anchors the grace */
    vehicle_e2e_tx_init(&tx);
}

void tearDown(void) {}

/* A protected 0x082 frame (zero payload: the monitor never decodes it) with the next counter. */
static void frame(uint8_t out[PLATFORM_HEARTBEAT_SAFETY_LENGTH])
{
    (void)memset(out, 0, PLATFORM_HEARTBEAT_SAFETY_LENGTH);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, vehicle_e2e_protect(PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID, out,
                                                      PLATFORM_HEARTBEAT_SAFETY_LENGTH, &tx));
}

static void send(uint32_t now)
{
    uint8_t f[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    frame(f);
    safety_hb_core_rx(&core, f, (uint8_t)sizeof f, now);
}

static void test_the_timeout_is_three_cycles(void)
{
    TEST_ASSERT_EQUAL_UINT32(3u * CYCLE, TIMEOUT);
}

static void test_start_up_grace_then_lost_when_no_frame_ever_arrives(void)
{
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, T0));
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, T0 + TIMEOUT));
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, T0 + TIMEOUT + 1u));
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.lost_events);
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, T0 + TIMEOUT + 2u)); /* still lost, one event */
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.lost_events);
}

static void test_a_healthy_stream_is_never_lost(void)
{
    for (uint32_t t = T0; t < T0 + 5000u; t += CYCLE) {
        send(t);
        TEST_ASSERT_FALSE(safety_hb_core_lost(&core, t));
        TEST_ASSERT_FALSE(safety_hb_core_lost(&core, t + CYCLE - 1u));
    }
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.resyncs); /* only the first frame */
    TEST_ASSERT_EQUAL_UINT32(0u, core.link.lost_events);
}

static void test_one_lost_frame_does_not_set_the_verdict(void)
{
    send(T0);
    send(T0 + CYCLE);
    /* the frame at T0 + 2 CYCLE is lost: the counter advances without reaching rt-core */
    uint8_t skipped[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    frame(skipped);
    for (uint32_t t = T0 + CYCLE; t < T0 + (3u * CYCLE); t++) {
        TEST_ASSERT_FALSE(safety_hb_core_lost(&core, t));
    }
    /* the next frame jumps the counter by 2 (max delta 1): a link error that does not
     * refresh the timeout, and not a verdict either */
    send(T0 + (3u * CYCLE));
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.wrong_sequence);
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, T0 + (3u * CYCLE)));
    /* the stream is back in step one cycle later: OK, and the timeout is refreshed */
    send(T0 + (4u * CYCLE));
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, T0 + (4u * CYCLE)));
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, T0 + (4u * CYCLE) + TIMEOUT));
    TEST_ASSERT_EQUAL_UINT32(0u, core.link.lost_events);
}

static void test_three_silent_cycles_set_the_verdict_and_a_frame_clears_it(void)
{
    send(T0);
    send(T0 + CYCLE);
    const uint32_t last = T0 + CYCLE;
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, last + TIMEOUT));
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, last + TIMEOUT + 1u));
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.lost_events);
    /* the sender is back: the first frame resyncs (INITIAL), the verdict clears */
    send(last + 1000u);
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, last + 1000u));
    TEST_ASSERT_EQUAL_UINT32(2u, core.link.resyncs);
    /* and a second outage counts again */
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, last + 1000u + TIMEOUT + 1u));
    TEST_ASSERT_EQUAL_UINT32(2u, core.link.lost_events);
}

static void test_a_single_error_is_a_link_count_not_a_lost_verdict(void)
{
    send(T0);
    send(T0 + CYCLE);
    uint8_t good[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    frame(good); /* the next valid frame, kept for later */

    uint8_t bad_crc[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    (void)memcpy(bad_crc, good, sizeof bad_crc);
    bad_crc[0] ^= 0x5Au;
    safety_hb_core_rx(&core, bad_crc, (uint8_t)sizeof bad_crc, T0 + CYCLE + 10u);
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.wrong_crc);

    uint8_t wrong_len[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    (void)memcpy(wrong_len, good, sizeof wrong_len);
    safety_hb_core_rx(&core, wrong_len, 7u, T0 + CYCLE + 20u);
    safety_hb_core_rx(&core, wrong_len, 0u, T0 + CYCLE + 30u);
    TEST_ASSERT_EQUAL_UINT32(2u, core.link.wrong_length);

    safety_hb_core_rx(&core, good, (uint8_t)sizeof good, T0 + (2u * CYCLE));
    safety_hb_core_rx(&core, good, (uint8_t)sizeof good, T0 + (2u * CYCLE) + 5u); /* same counter */
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.repeated);

    uint8_t jump[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    for (uint32_t i = 0u; i < 5u; i++) {
        frame(jump); /* counter jumps by 5: beyond max delta 1 */
    }
    safety_hb_core_rx(&core, jump, (uint8_t)sizeof jump, T0 + (2u * CYCLE) + 10u);
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.wrong_sequence);

    TEST_ASSERT_EQUAL_UINT32(8u, core.link.frames);
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, T0 + (2u * CYCLE) + 10u));
    TEST_ASSERT_EQUAL_UINT32(0u, core.link.lost_events);
}

static void test_wrong_length_is_invalid_and_leaves_the_e2e_state_alone(void)
{
    uint8_t f[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    frame(f);
    safety_hb_core_rx(&core, f, 6u, T0 + 50u); /* a valid-CRC payload with a wrong DLC */
    TEST_ASSERT_EQUAL_UINT32(1u, core.link.wrong_length);
    TEST_ASSERT_EQUAL_UINT32(0u, core.link.resyncs);
    TEST_ASSERT_FALSE(core.e2e.synced); /* untouched: never counts as a heartbeat */
    /* so a stream of wrong-length frames is silence: lost after the grace */
    for (uint32_t t = T0; t <= T0 + TIMEOUT + 100u; t += CYCLE) {
        safety_hb_core_rx(&core, f, 6u, t);
    }
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, T0 + TIMEOUT + 101u));
}

static void test_counters_saturate_and_null_is_safe(void)
{
    core.link.frames = UINT32_MAX;
    core.link.lost_events = UINT32_MAX;
    send(T0);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, core.link.frames);
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, T0 + TIMEOUT + 1000u));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, core.link.lost_events);
    safety_hb_core_init(NULL);
    safety_hb_core_rx(NULL, NULL, 0u, 0u);
    safety_hb_core_rx(&core, NULL, 8u, 0u);
    TEST_ASSERT_TRUE(safety_hb_core_lost(NULL, 0u));
}

static void test_the_verdict_is_wrap_safe(void)
{
    const uint32_t t = UINT32_MAX - 50u;
    safety_hb_core_init(&core);
    (void)safety_hb_core_lost(&core, t);
    send(t);
    send(t + CYCLE); /* wraps past 0 */
    TEST_ASSERT_FALSE(safety_hb_core_lost(&core, t + CYCLE + TIMEOUT));
    TEST_ASSERT_TRUE(safety_hb_core_lost(&core, t + CYCLE + TIMEOUT + 1u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_timeout_is_three_cycles);
    RUN_TEST(test_start_up_grace_then_lost_when_no_frame_ever_arrives);
    RUN_TEST(test_a_healthy_stream_is_never_lost);
    RUN_TEST(test_one_lost_frame_does_not_set_the_verdict);
    RUN_TEST(test_three_silent_cycles_set_the_verdict_and_a_frame_clears_it);
    RUN_TEST(test_a_single_error_is_a_link_count_not_a_lost_verdict);
    RUN_TEST(test_wrong_length_is_invalid_and_leaves_the_e2e_state_alone);
    RUN_TEST(test_counters_saturate_and_null_is_safe);
    RUN_TEST(test_the_verdict_is_wrap_safe);
    return UNITY_END();
}
