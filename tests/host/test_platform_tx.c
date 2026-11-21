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
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "features/safety_hb/safety_hb.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/imu_host.h"
#include "platform.h"
#include "platform_e2e.h"
#include "platform_uds.h"
#include "uds_iso14229.h"
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
/* D-118 item 6: the 0x082 monitor, opened only in the tests that ask for it (sh_on) */
static safety_hb_t sh;
static bool sh_on;
static bool sh_hostile; /* a hostile 0x082 stream and UDS requests share the bus */
static uint32_t sh_peer_until_ms; /* the simulated safety MCU is silent from here on */
static vehicle_e2e_tx_state_t sh_peer_tx;

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
    uint32_t digest; /* over every frame's time and bytes */
} stream_t;

typedef struct {
    stream_t lean;
    stream_t hb;
    uint32_t other_safety;    /* 0x023 or 0x082 sent by rt-core: never */
    uint32_t friction_frames; /* rt-core's own 0x022, whatever it is */
    uint32_t request_n;       /* rt-core's own 0x120 */
    uint8_t request[LOG_CAP][8];
    uint32_t unrouted; /* can_if_unrouted_count(PLATFORM) at the end */
    uint32_t dtc_status; /* DTC SAFETY_HEARTBEAT_LOST at the end */
    uint32_t uds_requests;
    uint32_t uds_answers;
    uint32_t hostile_sent;
    uint32_t dtc_failed_passes; /* passes with its TEST_FAILED bit set */
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
    TEST_ASSERT_TRUE(!sh_on || safety_hb_open(&sh));
    vehicle_e2e_tx_init(&sh_peer_tx);
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
    s->digest = (s->digest * 16777619u) ^ now;
    for (size_t i = 0u; i < len; i++) {
        s->digest = (s->digest * 16777619u) ^ f->data[i];
    }
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
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_p, node_peer, &f)); /* positive control */
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
    TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_p, node_peer, &r));
}

/* A hostile 0x082 stream: every 10 ms a burst of 40 frames (more than the 32 dispatched per
 * pass) with a bad CRC, a wrong DLC or random counters, plus a physical TesterPresent request
 * every 20 ms that must still be answered. The host port has one queue, so this also shows the
 * bounded dispatch never starves the UDS server (on the target the 0x7xx IDs sit in FIFO1,
 * apart from the 0x082 traffic in FIFO0). */
static void hostile_burst(uint32_t now, seen_t* seen)
{
    if ((now % 10u) == 0u) {
        for (uint32_t k = 0u; k < 40u; k++) {
            can_frame_t f;
            (void)memset(&f, 0, sizeof f);
            f.id = PLATFORM_HEARTBEAT_SAFETY_FRAME_ID;
            f.dlc = (uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH;
            f.data[1] = (uint8_t)(k & 0x0Fu);
            if ((k % 3u) == 0u) {
                f.data[0] = 0x5Au; /* bad CRC (almost surely) */
            } else if ((k % 3u) == 1u) {
                f.dlc = 7u; /* WRONG_LENGTH */
            } else {
                f.data[2] = (uint8_t)(k * 7u);
            }
            if (vbus_send(&bus_p, node_peer, &f) == CAN_PORT_OK) {
                seen->hostile_sent++;
            }
        }
    }
    if ((now % 20u) == 5u) {
        can_frame_t r;
        (void)memset(&r, PLATFORM_UDS_PADDING_BYTE, sizeof r.data);
        r.id = PLATFORM_UDS_PHYS_REQUEST_ID;
        r.extended = false;
        r.dlc = (uint8_t)PLATFORM_UDS_FRAME_DLC;
        r.data[0] = 2u;
        r.data[1] = UDS_SID_TESTER_PRESENT;
        r.data[2] = 0x00u;
        TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_p, node_peer, &r));
        seen->uds_requests++;
    }
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
        if (sh_on && (now < sh_peer_until_ms) && ((now % PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS) == 0u)) {
            /* the safety MCU's 0x082: valid E2E, zero payload (rt-core never decodes it) */
            can_frame_t hb82;
            (void)memset(&hb82, 0, sizeof hb82);
            hb82.id = PLATFORM_HEARTBEAT_SAFETY_FRAME_ID;
            hb82.dlc = (uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH;
            TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, vehicle_e2e_protect(PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID, hb82.data,
                                                              PLATFORM_HEARTBEAT_SAFETY_LENGTH, &sh_peer_tx));
            TEST_ASSERT_EQUAL(CAN_PORT_OK, vbus_send(&bus_p, node_peer, &hb82));
        }
        if (sh_hostile) {
            hostile_burst(now, &seen);
        }
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx, sh_on ? &sh : NULL);
        seen.dtc_failed_passes += ((diag_dtc_status(PLATFORM_UDS_DTC_IDX_SAFETY_HEARTBEAT_LOST) & 0x01u) != 0u) ? 1u : 0u;
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
            } else if (got.id == PLATFORM_UDS_PHYS_RESPONSE_ID) {
                seen.uds_answers += (got.data[0] == 2u) && (got.data[1] == (UDS_SID_TESTER_PRESENT + 0x40u)) ? 1u : 0u;
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
    seen.dtc_status = diag_dtc_status(PLATFORM_UDS_DTC_IDX_SAFETY_HEARTBEAT_LOST);
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

/* D-118 item 6: the 0x082 monitor reports its DTC through the real comms_pass() and nothing
 * else changes: 0x081 and 0x020 are byte-for-byte and time-for-time the same with the safety
 * MCU alive, silent from 600 ms on, or not monitored at all (NODE_MODE, ERROR_COUNT, the
 * counters, E2E all follow from the bytes). */
static void test_the_0x082_monitor_sets_its_dtc_and_changes_nothing_rt_core_sends(void)
{
    sh_on = false;
    rig_up();
    const seen_t unmonitored = run(false);

    sh_on = true;
    sh_peer_until_ms = UINT32_MAX;
    rig_up();
    const seen_t alive = run(false);
    TEST_ASSERT_EQUAL_UINT32(0u, alive.dtc_failed_passes);
    TEST_ASSERT_EQUAL_UINT32(0u, alive.dtc_status);
    TEST_ASSERT_TRUE(safety_hb_link(&sh).frames >= (RUN_MS / PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS) - 2u);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_hb_link(&sh).wrong_crc);

    sh_peer_until_ms = START_MS + 600u;
    rig_up();
    const seen_t silent = run(false);
    TEST_ASSERT_TRUE(silent.dtc_failed_passes > 0u);
    TEST_ASSERT_NOT_EQUAL(0u, silent.dtc_status & 0x09u); /* TEST_FAILED, CONFIRMED_DTC */
    TEST_ASSERT_EQUAL_UINT32(1u, safety_hb_link(&sh).lost_events);

    TEST_ASSERT_EQUAL_UINT32(unmonitored.hb.frames, alive.hb.frames);
    TEST_ASSERT_EQUAL_UINT32(unmonitored.hb.frames, silent.hb.frames);
    TEST_ASSERT_NOT_EQUAL(0u, unmonitored.hb.digest);
    TEST_ASSERT_NOT_EQUAL(0u, unmonitored.lean.digest);
    TEST_ASSERT_EQUAL_UINT32(unmonitored.hb.digest, alive.hb.digest);
    TEST_ASSERT_EQUAL_UINT32(unmonitored.hb.digest, silent.hb.digest);
    TEST_ASSERT_EQUAL_UINT32(unmonitored.lean.digest, alive.lean.digest);
    TEST_ASSERT_EQUAL_UINT32(unmonitored.lean.digest, silent.lean.digest);
    TEST_ASSERT_EQUAL_UINT32(0u, silent.hb.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, silent.lean.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, silent.other_safety);
    sh_on = false;
}

/* A hostile 0x082 stream cannot change what rt-core sends, and the UDS server keeps answering. */
static void test_a_hostile_0x082_stream_changes_nothing_and_uds_is_still_served(void)
{
    sh_on = false;
    sh_hostile = false;
    rig_up();
    const seen_t base = run(false);

    sh_on = true;
    sh_peer_until_ms = 0u; /* no genuine 0x082: the hostile frames are all there is */
    sh_hostile = true;
    rig_up();
    const seen_t hostile = run(false);
    sh_hostile = false;
    sh_on = false;

    TEST_ASSERT_TRUE(hostile.hostile_sent > 32u);
    TEST_ASSERT_TRUE(safety_hb_link(&sh).wrong_crc > 0u);
    TEST_ASSERT_TRUE(safety_hb_link(&sh).wrong_length > 0u);
    TEST_ASSERT_EQUAL_UINT32(base.hb.frames, hostile.hb.frames);
    TEST_ASSERT_EQUAL_UINT32(base.lean.frames, hostile.lean.frames);
    TEST_ASSERT_EQUAL_UINT32(base.hb.digest, hostile.hb.digest);
    TEST_ASSERT_EQUAL_UINT32(base.lean.digest, hostile.lean.digest);
    TEST_ASSERT_EQUAL_UINT32(0u, hostile.hb.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, hostile.lean.e2e_bad);
    TEST_ASSERT_EQUAL_UINT32(0u, hostile.other_safety);
    /* the stream is not a heartbeat: the DTC reports the silence, diagnostic only */
    TEST_ASSERT_TRUE(hostile.dtc_failed_passes > 0u);
    TEST_ASSERT_TRUE(hostile.uds_requests >= 90u);
    TEST_ASSERT_EQUAL_UINT32(hostile.uds_requests, hostile.uds_answers);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x020_and_0x081_are_on_the_platform_bus_at_their_cycles_with_valid_e2e);
    RUN_TEST(test_the_0x082_monitor_sets_its_dtc_and_changes_nothing_rt_core_sends);
    RUN_TEST(test_a_hostile_0x082_stream_changes_nothing_and_uds_is_still_served);
    RUN_TEST(test_injected_frames_are_filtered_and_rt_core_0x120_never_depends_on_them);
    return UNITY_END();
}
