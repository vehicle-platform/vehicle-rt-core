/*
 * SIL test (D-118): the safety MCU is an ordinary node on the platform bus (FDCAN2).
 * The real comms_pass() with the UDS client and the simulated CL250 ECU, the heartbeat
 * (0x081) and the lean sender (0x020 through lean_ekf_step), and a peer node on the
 * platform bus that stands for the safety MCU and the rest of the nodes.
 *  - 0x020 and 0x081 appear on the platform bus at their gen/ cycles, in a gapless E2E
 *    sequence that a receiver check (gen/ DataID, max delta, timeout) accepts.
 *  - 0x023 and 0x082 (the safety MCU's own frames) and a 0x120 that a peer puts on the bus
 *    are received as ordinary frames and never re-sent: rt-core never forwards a safety
 *    frame, and never fills 0x120 from a received frame (D-118; a release 0x120 comes
 *    from rt-core's own state only).
 * 0x022 and 0x120 have no rt-core sender yet; this file asserts what must stay true once
 * they exist (nothing mirrored from the bus).
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/imu_host.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/diag.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "vehicle_e2e.h"

#include <stdint.h>
#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define RUN_MS 2000u

static vbus_t bus_v;
static vbus_t bus_p;
static uint8_t node_vehicle;
static uint8_t node_platform;
static uint8_t node_peer;
static uds_client_t client;
static uds_server_t server;
static vehicle_republish_t republisher;
static heartbeat_t heartbeat;
static comms_ekf_t ekf;
static lean_link_t link;
static lean_ekf_t lean;
static lean_tx_t tx;
static sim_ecu_t ecu;

typedef struct {
    uint32_t frames;
    uint32_t last_ms;
    uint32_t gap_min;
    uint32_t gap_max;
    uint32_t e2e_ok;
    uint32_t e2e_bad; /* anything but OK after the first (INITIAL) frame */
    vehicle_e2e_rx_state_t rx;
} stream_t;

typedef struct {
    stream_t lean;
    stream_t hb;
    uint32_t other_safety; /* 0x022, 0x023, 0x082, 0x120 sent by rt-core */
} seen_t;

void setUp(void) {}
void tearDown(void) {}

static void rig_up(void)
{
    hal_time_host_use_manual(START_MS);
    vbus_init(&bus_v);
    vbus_init(&bus_p);
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_platform));
    TEST_ASSERT_TRUE(sim_ecu_init(&ecu, &bus_v));
    ecu.require_session = true;
    can_if_init();
    diag_init(timebase_now_ms());
    vehicle_signals_init();
    imu_host_reset();
    (void)memset(&client, 0, sizeof client);
    (void)memset(&server, 0, sizeof server);
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_client_open(&client));
    TEST_ASSERT_EQUAL(ISOTP_OK, uds_server_open(&server));
    TEST_ASSERT_TRUE(vehicle_republish_open(&republisher));
    TEST_ASSERT_TRUE(heartbeat_open(&heartbeat));
    comms_ekf_init(&ekf);
    lean_link_init(&link);
    lean_ekf_init(&lean, &link);
    TEST_ASSERT_TRUE(comms_ekf_register(&ekf, lean_link_alive(&link)));
    TEST_ASSERT_TRUE(comms_open_lean(&ekf, &tx, &link));
    TEST_ASSERT_TRUE(comms_apply_filters());
}

static void note(stream_t* s, uint32_t now, uint16_t data_id, uint8_t max_delta, uint32_t timeout_ms,
                 const can_frame_t* f, size_t len)
{
    TEST_ASSERT_EQUAL_UINT8(len, f->dlc);
    const vehicle_e2e_status_t st = vehicle_e2e_check(data_id, max_delta, timeout_ms, f->data, len, &s->rx, now);
    if (st == VEHICLE_E2E_OK) {
        s->e2e_ok++;
    } else if (!((s->frames == 0u) && (st == VEHICLE_E2E_INITIAL))) {
        s->e2e_bad++;
    } else {
        /* the first frame resyncs */
    }
    if (s->frames > 0u) {
        const uint32_t gap = now - s->last_ms;
        if ((s->frames == 1u) || (gap < s->gap_min)) {
            s->gap_min = gap;
        }
        if (gap > s->gap_max) {
            s->gap_max = gap;
        }
    }
    s->last_ms = now;
    s->frames++;
}

/* RUN_MS passes of 1 ms. inject: the peer sends 0x023, 0x082 and 0x120 every 20 ms. */
static seen_t run(bool inject)
{
    seen_t seen;
    (void)memset(&seen, 0, sizeof seen);
    vehicle_e2e_rx_init(&seen.lean.rx);
    vehicle_e2e_rx_init(&seen.hb.rx);
    for (uint32_t i = 0u; i < RUN_MS; i++) {
        const uint32_t now = timebase_now_ms();
        if ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u) {
            lean_ekf_step(&lean, now);
        }
        sim_ecu_step(&ecu, now);
        if (inject && ((now % 20u) == 0u)) {
            static const uint32_t ids[3] = {PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID,
                                            PLATFORM_HEARTBEAT_SAFETY_FRAME_ID, PLATFORM_SAFETY_REQUEST_FRAME_ID};
            for (uint32_t k = 0u; k < 3u; k++) {
                can_frame_t f;
                (void)memset(&f, 0, sizeof f);
                f.id = ids[k];
                f.dlc = 8u;
                f.data[1] = (uint8_t)(now & 0x0Fu);
                (void)vbus_send(&bus_p, node_peer, &f);
            }
        }
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_peer, &got) == CAN_PORT_OK) {
            if (got.extended) {
                continue;
            }
            if (got.id == PLATFORM_EKF_LEAN_FRAME_ID) {
                note(&seen.lean, now, PLATFORM_EKF_LEAN_E2E_DATA_ID, PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                     PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, &got, PLATFORM_EKF_LEAN_LENGTH);
            } else if (got.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                note(&seen.hb, now, PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID,
                     PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER, PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS,
                     &got, PLATFORM_HEARTBEAT_RT_CORE_LENGTH);
            } else if ((got.id == PLATFORM_EKF_FRICTION_MASS_FRAME_ID) ||
                       (got.id == PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID) ||
                       (got.id == PLATFORM_HEARTBEAT_SAFETY_FRAME_ID) || (got.id == PLATFORM_SAFETY_REQUEST_FRAME_ID)) {
                seen.other_safety++;
            } else {
                /* 0x021, 0x110, UDS answers: not counted here */
            }
        }
        hal_time_host_advance(1u);
    }
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
    return seen;
}

/* 0x020 (20 ms) and 0x081 (100 ms) go out on FDCAN2 through the normal com / can_if path. */
static void test_0x020_and_0x081_are_on_the_platform_bus_at_their_cycles_with_valid_e2e(void)
{
    rig_up();
    const seen_t s = run(false);
    TEST_ASSERT_UINT32_WITHIN(2u, RUN_MS / PLATFORM_EKF_LEAN_CYCLE_TIME_MS, s.lean.frames);
    TEST_ASSERT_UINT32_WITHIN(2u, RUN_MS / PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, s.hb.frames);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_EKF_LEAN_CYCLE_TIME_MS, s.lean.gap_min);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_EKF_LEAN_CYCLE_TIME_MS, s.lean.gap_max);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, s.hb.gap_min);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, s.hb.gap_max);
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, s.hb.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(s.lean.frames - 1u, s.lean.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(s.hb.frames - 1u, s.hb.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, s.other_safety);
}

/* The safety MCU's own frames, and a foreign 0x120, change nothing on the bus from rt-core. */
static void test_received_safety_frames_are_never_resent_and_0x120_is_never_mirrored(void)
{
    rig_up();
    const seen_t quiet = run(false);
    rig_up();
    const seen_t loud = run(true);
    TEST_ASSERT_EQUAL_UINT32(0u, loud.other_safety);
    TEST_ASSERT_EQUAL_UINT32(quiet.lean.frames, loud.lean.frames);
    TEST_ASSERT_EQUAL_UINT32(quiet.hb.frames, loud.hb.frames);
    TEST_ASSERT_EQUAL_UINT32(0u, loud.lean.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, loud.hb.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(quiet.lean.gap_max, loud.lean.gap_max);
    TEST_ASSERT_EQUAL_UINT32(quiet.hb.gap_max, loud.hb.gap_max);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x020_and_0x081_are_on_the_platform_bus_at_their_cycles_with_valid_e2e);
    RUN_TEST(test_received_safety_frames_are_never_resent_and_0x120_is_never_mirrored);
    return UNITY_END();
}
