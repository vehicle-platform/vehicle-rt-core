/*
 * L0 tests for the safety-heartbeat monitor glue (features/safety_hb, D-118 item 6) on the
 * host CAN port bound to the in-process bus, with a manual clock: a peer plays the safety
 * MCU's 0x082, rt-core receives it through can_if and reports the diagnostic-only DTC
 * SAFETY_HEARTBEAT_LOST. Checked: the DTC sets after three silent cycles and not after one
 * lost frame, it leaves rt-core's own 0x081 (NODE_MODE, ERROR_COUNT) alone, and a wrong
 * DLC (WRONG_LENGTH) is a link error only.
 */
#include "app/host/sim_listener.h"
#include "features/heartbeat/heartbeat.h"
#include "features/safety_hb/safety_hb.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "platform.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "vehicle_e2e.h"

#include <string.h>
#include <unity.h>

#define START_MS 5000u
#define CYCLE PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS
#define TIMEOUT PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS
#define IDX PLATFORM_UDS_DTC_IDX_SAFETY_HEARTBEAT_LOST

static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;
static heartbeat_t hb;
static safety_hb_t mon;
static vehicle_e2e_tx_state_t peer_tx;
/* what the peer saw of rt-core's own 0x081 */
static uint32_t hb_frames;
static uint32_t hb_degraded;
static uint32_t hb_error_count_max;
static uint32_t last_send_ms; /* the peer's latest 0x082 send */

void setUp(void)
{
    hal_time_host_use_manual(START_MS);
    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init();
    diag_init(timebase_now_ms());
    TEST_ASSERT_TRUE(heartbeat_open(&hb));
    TEST_ASSERT_TRUE(safety_hb_open(&mon));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_VEHICLE);
    can_sm_step(CAN_PORT_PLATFORM);
    vehicle_e2e_tx_init(&peer_tx);
    hb_frames = 0u;
    hb_degraded = 0u;
    hb_error_count_max = 0u;
    last_send_ms = 0u;
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* The safety MCU's 0x082: zero payload (never decoded by rt-core), the next E2E counter; dlc is the sent length. */
static void peer_send(uint8_t dlc)
{
    can_frame_t f;
    (void)memset(&f, 0, sizeof f);
    f.id = PLATFORM_HEARTBEAT_SAFETY_FRAME_ID;
    f.extended = PLATFORM_HEARTBEAT_SAFETY_IS_EXTENDED != 0;
    f.dlc = dlc;
    /* a frame of the wrong length does not advance the stream's counter */
    vehicle_e2e_tx_state_t scratch = peer_tx;
    vehicle_e2e_tx_state_t* const state = (dlc == PLATFORM_HEARTBEAT_SAFETY_LENGTH) ? &peer_tx : &scratch;
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, vehicle_e2e_protect(PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID, f.data,
                                                      PLATFORM_HEARTBEAT_SAFETY_LENGTH, state));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_p, node_peer, &f));
}

/* The peer drains rt-core's frames (only 0x081 is expected). */
static void peer_drain(void)
{
    can_frame_t f;
    while (vbus_recv(&bus_p, node_peer, &f) == CAN_PORT_OK) {
        if (f.id != PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
            continue;
        }
        struct platform_heartbeat_rt_core_t m;
        sim_listener_heartbeat_decode(f.data, &m);
        hb_frames++;
        if (m.rt_core_node_mode == PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_DEGRADED_CHOICE) {
            hb_degraded++;
        }
        if (m.rt_core_error_count > hb_error_count_max) {
            hb_error_count_max = m.rt_core_error_count;
        }
    }
}

/* One comms-pass subset (app/comms order: dispatch, monitor, heartbeat), then one ms. */
static void pass(void)
{
    const uint32_t now = timebase_now_ms();
    const diag_vehicle_tester_t st = {true, true, false,
                                      (uint8_t)PLATFORM_UDS_VEHICLE_TESTER_STATUS_FAULT_NONE};
    diag_set_vehicle_tester(&st, now);
    can_sm_step(CAN_PORT_PLATFORM);
    (void)can_if_dispatch(CAN_PORT_PLATFORM, 32u);
    safety_hb_step(&mon);
    heartbeat_step(&hb, false);
    peer_drain();
    hal_time_host_advance(1u);
}

/* Runs ms passes; the peer sends a valid 0x082 every CYCLE ms while sending is true, the
 * skip-th send from here on (1-based, 0 = none) being lost: its counter advances but the
 * frame never reaches the bus. */
static void run(uint32_t ms, bool sending, uint32_t skip)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < ms; i++) {
        if (sending && ((timebase_now_ms() % CYCLE) == 0u)) {
            n++;
            last_send_ms = timebase_now_ms();
            if (n == skip) {
                vehicle_e2e_tx_state_t t = peer_tx;
                uint8_t lost[PLATFORM_HEARTBEAT_SAFETY_LENGTH] = {0};
                (void)vehicle_e2e_protect(PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID, lost, sizeof lost, &t);
                peer_tx = t;
            } else {
                peer_send((uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH);
            }
        }
        pass();
    }
}

/* Silent passes up to and including the pass at absolute time t_ms. */
static void silence_to(uint32_t t_ms)
{
    while (timebase_now_ms() <= t_ms) {
        pass();
    }
}

static bool dtc_failed(void)
{
    return (diag_dtc_status(IDX) & 0x01u) != 0u;
}

static void test_the_receiver_is_registered_on_the_platform_bus_only(void)
{
    TEST_ASSERT_TRUE(mon.opened);
    run(10u * CYCLE, true, 0u);
    TEST_ASSERT_TRUE(safety_hb_link(&mon).frames >= 9u);
    TEST_ASSERT_FALSE(dtc_failed());
    TEST_ASSERT_EQUAL_UINT32(0u, can_if_unrouted_count(CAN_PORT_PLATFORM));
}

static void test_open_after_the_filters_are_sealed_is_refused_and_reports_nothing(void)
{
    safety_hb_t late;
    TEST_ASSERT_FALSE(safety_hb_open(&late));
    safety_hb_step(&late);
    TEST_ASSERT_FALSE(safety_hb_open(NULL));
    safety_hb_step(NULL);
    TEST_ASSERT_EQUAL_UINT8(0u, diag_dtc_status(IDX));
    TEST_ASSERT_EQUAL_UINT32(0u, safety_hb_link(NULL).frames);
}

static void test_one_lost_frame_does_not_set_the_dtc(void)
{
    run(3u * CYCLE, true, 0u);
    TEST_ASSERT_FALSE(dtc_failed());
    run(10u * CYCLE, true, 3u); /* the third send from here on is lost */
    TEST_ASSERT_FALSE(dtc_failed());
    TEST_ASSERT_EQUAL_UINT8(0u, diag_dtc_status(IDX));
}

static void test_three_silent_cycles_set_the_dtc_and_a_frame_clears_the_test_result(void)
{
    run(5u * CYCLE, true, 0u);
    silence_to(last_send_ms + TIMEOUT); /* three cycles of silence: not yet */
    TEST_ASSERT_FALSE(dtc_failed());
    silence_to(last_send_ms + TIMEOUT + 1u);
    TEST_ASSERT_TRUE(dtc_failed());
    TEST_ASSERT_NOT_EQUAL(0u, diag_dtc_status(IDX) & 0x08u); /* CONFIRMED_DTC */
    /* the safety MCU returns: the test result clears, the confirmed bit stays until a clear */
    run(3u * CYCLE, true, 0u);
    TEST_ASSERT_FALSE(dtc_failed());
    TEST_ASSERT_NOT_EQUAL(0u, diag_dtc_status(IDX) & 0x08u);
}

static void test_a_safety_mcu_that_never_boots_fails_the_dtc_after_the_start_up_grace(void)
{
    silence_to(START_MS + TIMEOUT);
    TEST_ASSERT_FALSE(dtc_failed());
    silence_to(START_MS + TIMEOUT + 1u);
    TEST_ASSERT_TRUE(dtc_failed());
}

static void test_the_dtc_leaves_rt_core_s_own_heartbeat_alone(void)
{
    run(10u * CYCLE, true, 0u);
    run(10u * CYCLE, false, 0u); /* the safety MCU goes silent: the DTC sets */
    TEST_ASSERT_TRUE(dtc_failed());
    run(10u * CYCLE, true, 0u);
    run(10u * CYCLE, false, 0u); /* and again */
    TEST_ASSERT_EQUAL_UINT32(2u, safety_hb_link(&mon).lost_events);
    peer_drain();
    TEST_ASSERT_TRUE(hb_frames > 20u);
    TEST_ASSERT_EQUAL_UINT32(0u, hb_degraded);         /* NODE_MODE stayed NORMAL */
    TEST_ASSERT_EQUAL_UINT32(0u, hb_error_count_max);  /* ERROR_COUNT unchanged */
    TEST_ASSERT_FALSE(diag_fault_active());
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_a_wrong_dlc_is_a_link_error_not_a_heartbeat(void)
{
    run(3u * CYCLE, true, 0u);
    const uint32_t before = safety_hb_link(&mon).wrong_length;
    for (uint32_t i = 0u; i < 4u; i++) {
        peer_send(7u); /* the E2E check sees WRONG_LENGTH */
        run(CYCLE, true, 0u);
    }
    TEST_ASSERT_EQUAL_UINT32(before + 4u, safety_hb_link(&mon).wrong_length);
    TEST_ASSERT_FALSE(dtc_failed()); /* the valid frames keep coming: only a link count */
    /* wrong-length frames alone do not keep the monitor alive: they are silence to it */
    const uint32_t last_valid = last_send_ms;
    while (timebase_now_ms() <= (last_valid + TIMEOUT)) {
        if ((timebase_now_ms() % CYCLE) == 0u) {
            peer_send(7u);
        }
        pass();
    }
    TEST_ASSERT_FALSE(dtc_failed());
    silence_to(last_valid + TIMEOUT + 1u);
    TEST_ASSERT_TRUE(dtc_failed());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_receiver_is_registered_on_the_platform_bus_only);
    RUN_TEST(test_open_after_the_filters_are_sealed_is_refused_and_reports_nothing);
    RUN_TEST(test_one_lost_frame_does_not_set_the_dtc);
    RUN_TEST(test_three_silent_cycles_set_the_dtc_and_a_frame_clears_the_test_result);
    RUN_TEST(test_a_safety_mcu_that_never_boots_fails_the_dtc_after_the_start_up_grace);
    RUN_TEST(test_the_dtc_leaves_rt_core_s_own_heartbeat_alone);
    RUN_TEST(test_a_wrong_dlc_is_a_link_error_not_a_heartbeat);
    return UNITY_END();
}
