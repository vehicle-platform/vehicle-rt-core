/*
 * SIL test (D-118): the safety MCU is an ordinary node on the platform bus (FDCAN2).
 * The real comms_pass() with the UDS client and the simulated CL250 ECU, the heartbeat
 * (0x081) and the lean sender (0x020 through lean_ekf_step), and a peer node on the
 * platform bus that stands for the safety MCU and the rest of the nodes.
 *  - 0x020 and 0x081 appear on the platform bus at their gen/ cycles, in a gapless E2E
 *    sequence that a receiver check (gen/ DataID, max delta, timeout) accepts.
 *  - 0x081 runs half a 0x020 cycle behind 0x020 after its first (INIT) frame, so the two
 *    never share a pass (the safety MCU's RX FIFOs are 3 deep).
 *  - 0x023 and 0x082 (the safety MCU's own frames) are never received by rt-core (no
 *    receiver, the acceptance filter drops them) and never sent by it. A gen-packed,
 *    E2E-valid 0x120 with IMMOBILIZER_LOCK_REQUEST = LOCK and peer frames on rt-core's own
 *    0x020 / 0x081 IDs (masquerade) change nothing: rt-core's own 0x022 / 0x120 (if it
 *    ever sends them) are the same with and without the injection and never carry an
 *    injected LOCK (D-118; a release 0x120 comes from rt-core's own state only).
 *  - The two retired SAFETY_MAILBOX DTCs stay unset through a run with bus faults.
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
#define LOG_CAP 64u

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
    uint32_t first_gap; /* the gap between the first two frames */
    uint32_t gap_min;   /* of the later gaps */
    uint32_t gap_max;
    uint32_t times[LOG_CAP];
    uint32_t e2e_ok;
    uint32_t e2e_bad; /* anything but OK after the first (INITIAL) frame */
    vehicle_e2e_rx_state_t rx;
} stream_t;

typedef struct {
    stream_t lean;
    stream_t hb;
    uint32_t other_safety;    /* 0x023 or 0x082 sent by rt-core: never */
    uint32_t friction_frames; /* rt-core's own 0x022, whatever it is */
    uint32_t request_n;       /* rt-core's own 0x120 */
    uint8_t request[LOG_CAP][8];
    uint32_t unrouted; /* can_if_unrouted_count(PLATFORM) at the end */
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
    if (s->frames < LOG_CAP) {
        s->times[s->frames] = now;
    }
    if (s->frames == 1u) {
        s->first_gap = now - s->last_ms;
    } else if (s->frames > 1u) {
        const uint32_t gap = now - s->last_ms;
        if ((s->frames == 2u) || (gap < s->gap_min)) {
            s->gap_min = gap;
        }
        if (gap > s->gap_max) {
            s->gap_max = gap;
        }
    } else {
        /* the first frame: no gap yet */
    }
    s->last_ms = now;
    s->frames++;
}

/* What a hostile or faulty peer puts on the platform bus: the safety MCU's IDs, an
 * E2E-valid gen-packed 0x120 with IMMOBILIZER_LOCK_REQUEST = LOCK, and frames on rt-core's
 * own 0x020 / 0x081 / 0x022 IDs (masquerade). */
static void inject_frames(uint32_t now)
{
    static vehicle_e2e_tx_state_t e2e;
    static bool e2e_ready;
    static const uint32_t plain[5] = {PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID, PLATFORM_HEARTBEAT_SAFETY_FRAME_ID,
                                      PLATFORM_EKF_LEAN_FRAME_ID, PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID,
                                      PLATFORM_EKF_FRICTION_MASS_FRAME_ID};
    for (uint32_t k = 0u; k < 5u; k++) {
        can_frame_t f;
        (void)memset(&f, 0, sizeof f);
        f.id = plain[k];
        f.dlc = 8u;
        f.data[1] = (uint8_t)(now & 0x0Fu);
        (void)vbus_send(&bus_p, node_peer, &f);
    }
    struct platform_safety_request_t msg;
    (void)platform_safety_request_init(&msg);
    msg.immobilizer_lock_request = PLATFORM_SAFETY_REQUEST_IMMOBILIZER_LOCK_REQUEST_LOCK_CHOICE;
    can_frame_t r;
    (void)memset(&r, 0, sizeof r);
    r.id = PLATFORM_SAFETY_REQUEST_FRAME_ID;
    r.dlc = PLATFORM_SAFETY_REQUEST_LENGTH;
    TEST_ASSERT_EQUAL_INT(PLATFORM_SAFETY_REQUEST_LENGTH, platform_safety_request_pack(r.data, &msg, sizeof r.data));
    if (!e2e_ready) {
        vehicle_e2e_tx_init(&e2e);
        e2e_ready = true;
    }
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, platform_safety_request_e2e_protect(r.data, r.dlc, &e2e));
    (void)vbus_send(&bus_p, node_peer, &r);
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
            inject_frames(now);
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
            } else if (got.id == PLATFORM_EKF_FRICTION_MASS_FRAME_ID) {
                seen.friction_frames++;
            } else if (got.id == PLATFORM_SAFETY_REQUEST_FRAME_ID) {
                if (seen.request_n < LOG_CAP) {
                    (void)memcpy(seen.request[seen.request_n], got.data, 8u);
                }
                seen.request_n++;
            } else if ((got.id == PLATFORM_SAFETY_WARNING_STATUS_FRAME_ID) ||
                       (got.id == PLATFORM_HEARTBEAT_SAFETY_FRAME_ID)) {
                seen.other_safety++;
            } else {
                /* 0x021, 0x110, UDS answers: not counted here */
            }
        }
        hal_time_host_advance(1u);
    }
    seen.unrouted = can_if_unrouted_count(CAN_PORT_PLATFORM);
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
    /* 0x081: the first (INIT) frame at once, then the cycle runs half a 0x020 cycle behind it */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS + (PLATFORM_EKF_LEAN_CYCLE_TIME_MS / 2u),
                             s.hb.first_gap);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, s.hb.gap_min);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS, s.hb.gap_max);
    for (uint32_t i = 1u; (i < s.hb.frames) && (i < LOG_CAP); i++) {
        TEST_ASSERT_EQUAL_UINT32(PLATFORM_EKF_LEAN_CYCLE_TIME_MS / 2u,
                                 (s.hb.times[i] - s.lean.times[0]) % PLATFORM_EKF_LEAN_CYCLE_TIME_MS);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, s.lean.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, s.hb.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(s.lean.frames - 1u, s.lean.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(s.hb.frames - 1u, s.hb.e2e_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, s.other_safety);
}

/* Frames injected by a peer change nothing in what rt-core sends; the filter drops them. */
static void test_injected_frames_are_filtered_and_rt_core_0x120_never_depends_on_them(void)
{
    rig_up();
    const seen_t quiet = run(false);
    rig_up();
    const seen_t loud = run(true);
    TEST_ASSERT_EQUAL_UINT32(0u, loud.other_safety);
    TEST_ASSERT_EQUAL_UINT32(quiet.unrouted, loud.unrouted); /* dropped by the acceptance filter, not by dispatch */
    TEST_ASSERT_EQUAL_UINT32(quiet.lean.frames, loud.lean.frames);
    TEST_ASSERT_EQUAL_UINT32(quiet.hb.frames, loud.hb.frames);
    TEST_ASSERT_EQUAL_UINT32(0u, loud.lean.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, loud.hb.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(quiet.lean.gap_max, loud.lean.gap_max);
    TEST_ASSERT_EQUAL_UINT32(quiet.hb.gap_max, loud.hb.gap_max);
    /* rt-core's own 0x022 / 0x120, if it ever sends them, are independent of the bus */
    TEST_ASSERT_EQUAL_UINT32(quiet.friction_frames, loud.friction_frames);
    TEST_ASSERT_EQUAL_UINT32(quiet.request_n, loud.request_n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(quiet.request, loud.request, sizeof loud.request);
    for (uint32_t i = 0u; (i < loud.request_n) && (i < LOG_CAP); i++) {
        /* IMMOBILIZER_LOCK_REQUEST: byte 2, bits 0-1 (gen/ pack): never the injected LOCK */
        TEST_ASSERT_NOT_EQUAL(PLATFORM_SAFETY_REQUEST_IMMOBILIZER_LOCK_REQUEST_LOCK_CHOICE,
                              loud.request[i][2] & 0x03u);
    }
}

/* Safety review MINOR-5: nothing reports the two retired mailbox DTCs, whatever the buses do. */
static void test_the_retired_mailbox_dtcs_stay_unset_through_bus_faults(void)
{
    rig_up();
    for (uint32_t i = 0u; i < RUN_MS; i++) {
        const uint32_t now = timebase_now_ms();
        if ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u) {
            lean_ekf_step(&lean, now);
        }
        sim_ecu_step(&ecu, now);
        if (i == 500u) {
            vbus_set_bus_off(&bus_p, node_platform, true);
            vbus_set_bus_off(&bus_v, node_vehicle, true);
        }
        if (i == 900u) {
            vbus_recover(&bus_p, node_platform);
            vbus_recover(&bus_v, node_vehicle);
            vbus_set_tx_stalled(&bus_p, node_platform, true);
        }
        if (i == 1300u) {
            vbus_set_tx_stalled(&bus_p, node_platform, false);
        }
        inject_frames(now);
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx);
        hal_time_host_advance(1u);
    }
    TEST_ASSERT_EQUAL_UINT8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_SAFETY_MAILBOX_LAYOUT_MISMATCH));
    TEST_ASSERT_EQUAL_UINT8(0u, diag_dtc_status(PLATFORM_UDS_DTC_IDX_SAFETY_MAILBOX_SLOT_FROZEN));
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x020_and_0x081_are_on_the_platform_bus_at_their_cycles_with_valid_e2e);
    RUN_TEST(test_injected_frames_are_filtered_and_rt_core_0x120_never_depends_on_them);
    RUN_TEST(test_the_retired_mailbox_dtcs_stay_unset_through_bus_faults);
    return UNITY_END();
}
