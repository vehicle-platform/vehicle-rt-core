/*
 * SIL test (D-093 item 6, architecture-guard): the mailbox step in the comms pass must not
 * disturb the platform cycle. The real comms_pass() with the UDS client and the simulated
 * CL250 ECU, the heartbeat (0x081) and the lean sender (0x020 through lean_ekf_step), with
 * services/mailbox in
 *   A  never initialised (a missing boot call: every step counted as skipped),
 *   B  initialised, with a model of the M4 stepping before every pass: its layout word,
 *      SAFETY_WARNING_STATUS and HEARTBEAT_SAFETY frames at their cycles, and reads of the
 *      EKF_LEAN and HEARTBEAT_RT_CORE slots,
 *   C  initialised, with every mailbox HSEM refused every take (the M4 never lets go).
 * The frame counts and the longest gap of 0x081 and 0x020 on the platform bus must be
 * identical in all three. In B the M4 must also get exactly the frames the M7 committed
 * (the payload of the n-th 0x020 / 0x081 sent on the bus is the n-th frame the M4 accepts:
 * in order, no gap, no duplicate) and the M4's frames must appear on the platform bus
 * verbatim, in order. In C the writes are deferred, the M4->M7 slots go stale, and nothing
 * is forwarded.
 *
 * The comms pass reads the clock from hal_time_host (manual time here). The host HSEM
 * model (hal/host/hsem_host.c) is reset in every rig; the M4 model runs on its core. A
 * must run before B and C in this file: once mailbox_init() ran, the module stays
 * initialised (every call starts over).
 */
#include "app/comms.h"
#include "app/host/sim_ecu.h"
#include "features/cornering/lean.h"
#include "features/cornering/lean_tx.h"
#include "hal/hsem.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/hsem_host.h"
#include "hal/host/imu_host.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/diag.h"
#include "services/mailbox.h"
#include "services/timebase.h"
#include "services/vehicle_signals.h"
#include "vehicle_e2e.h"

#include <stdint.h>
#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define RUN_MS 1500u
#define LOG_CAP 256u

_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_DIRECTION == PLATFORM_MAILBOX_DIR_M7_TO_M4, "EKF_LEAN is M7->M4");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_DIRECTION == PLATFORM_MAILBOX_DIR_M7_TO_M4,
               "HEARTBEAT_RT_CORE is M7->M4");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_DIRECTION == PLATFORM_MAILBOX_DIR_M4_TO_M7,
               "SAFETY_WARNING_STATUS is M4->M7");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_DIRECTION == PLATFORM_MAILBOX_DIR_M4_TO_M7,
               "HEARTBEAT_SAFETY is M4->M7");
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH == CAN_PORT_MAX_DLC, "the logs hold full frames");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH == CAN_PORT_MAX_DLC, "the logs hold full frames");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH == CAN_PORT_MAX_DLC, "the logs hold full frames");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH == CAN_PORT_MAX_DLC, "the logs hold full frames");

/* ------------------------------------------------------------------------- rig */

static vbus_t bus_v;
static vbus_t bus_p;
static uint8_t node_vehicle;
static uint8_t node_platform;
static uint8_t node_ppeer;
static uds_client_t client;
static uds_server_t server;
static vehicle_republish_t republisher;
static heartbeat_t heartbeat;
static comms_ekf_t ekf;
static lean_link_t link;
static lean_ekf_t lean;
static lean_tx_t tx;
static sim_ecu_t ecu;
static mailbox_region_t region;

typedef struct {
    uint32_t hb_frames;
    uint32_t hb_gap_max;
    uint32_t hb_last;
    uint32_t lean_frames;
    uint32_t lean_gap_max;
    uint32_t lean_last;
    mailbox_health_t health; /* at the end of the run */
} counts_t;

typedef enum { MB_OFF = 0, MB_ON, MB_REFUSED } mb_mode_t;

enum { S_LEAN = 0, S_HB_RT, S_WARN, S_HB_SAFETY, S_COUNT };

static const uint32_t bus_ids[S_COUNT] = {
    PLATFORM_MAILBOX_SLOT_EKF_LEAN_FRAME_ID,
    PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_FRAME_ID,
    PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_FRAME_ID,
    PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_FRAME_ID,
};

typedef struct {
    uint8_t data[LOG_CAP][CAN_PORT_MAX_DLC];
    uint32_t n;
} frame_log_t;

static frame_log_t bus_log[S_COUNT]; /* what the peer node saw, per ID, in order */

/* The M4 model. Two slots it writes, two it reads. */
typedef struct {
    uint8_t hsem_id;
    uint32_t word;
    uint32_t cycle_ms; /* passes between two new frames */
} m4_tx_def_t;

typedef struct {
    uint8_t hsem_id;
    uint32_t word;
    uint32_t stream; /* index in bus_log of the frames it must get */
} m4_rx_def_t;

static const m4_tx_def_t m4_tx_def[2] = {
    {(uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
     PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_OFFSET / MAILBOX_WORD_BYTES,
     PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
     PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_OFFSET / MAILBOX_WORD_BYTES, PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS},
};

static const m4_rx_def_t m4_rx_def[2] = {
    {(uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID, PLATFORM_MAILBOX_SLOT_EKF_LEAN_OFFSET / MAILBOX_WORD_BYTES,
     S_LEAN},
    {(uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID,
     PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_OFFSET / MAILBOX_WORD_BYTES, S_HB_RT},
};

static mailbox_pending_t m4_pend[2];
static uint32_t m4_k[2];
static frame_log_t m4_written[2]; /* frames the M4 committed, in order */
static mailbox_reader_t m4_reader[2];
static uint32_t m4_seen[2];
/* The M4's real E2E receiver check on each slot copy (safety-reviewer MINOR-3, D-093 item
 * 6 "zero WRONG_SEQUENCE"): gen/ DataID, max delta and timeout of 0x020 / 0x081. */
static const uint16_t m4_e2e_id[2] = {PLATFORM_EKF_LEAN_E2E_DATA_ID, PLATFORM_HEARTBEAT_RT_CORE_E2E_DATA_ID};
static const uint8_t m4_e2e_delta[2] = {PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                                        PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER};
static const uint32_t m4_e2e_timeout[2] = {PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS};
static vehicle_e2e_rx_state_t m4_e2e[2];
static uint32_t m4_e2e_ok[2];
static uint32_t m4_e2e_bad[2]; /* anything but OK after the first (INITIAL) frame */
static bool inject_faults;    /* refused HSEMs and TX_FULL passes, see m4_turn() */
static uint32_t m4_mismatch; /* an accepted frame that is not the n-th one on the bus */

void setUp(void) {}
void tearDown(void) {}

/* All six slot semaphores of the generated table. */
static const uint8_t all_hsem[PLATFORM_MAILBOX_SLOT_COUNT] = {
    (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID,       (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
    (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
};

/* mailbox_init() on the rig's region (before comms_apply_filters(): it registers the forwarded IDs for dedicated TX buffers); refused: every take of every slot fails. */
static void mailbox_boot(bool refused)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, timebase_now_ms()));
    if (refused) {
        for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
            hsem_host_force_busy(all_hsem[i], UINT32_MAX);
        }
    }
}

/* mb: MB_OFF leaves the mailbox uninitialised; MB_ON / MB_REFUSED boot it before the filters. */
static void rig_up(mb_mode_t mb)
{
    hal_time_host_use_manual(START_MS);
    hsem_host_reset();
    (void)memset(&region, 0, sizeof region);
    (void)memset(bus_log, 0, sizeof bus_log);
    (void)memset(m4_written, 0, sizeof m4_written);
    (void)memset(m4_k, 0, sizeof m4_k);
    (void)memset(m4_seen, 0, sizeof m4_seen);
    (void)memset(m4_e2e_ok, 0, sizeof m4_e2e_ok);
    (void)memset(m4_e2e_bad, 0, sizeof m4_e2e_bad);
    vehicle_e2e_rx_init(&m4_e2e[0]);
    vehicle_e2e_rx_init(&m4_e2e[1]);
    m4_mismatch = 0u;
    for (uint32_t i = 0u; i < 2u; i++) {
        mailbox_pending_init(&m4_pend[i]);
    }
    mailbox_reader_init(&m4_reader[0], (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH);
    mailbox_reader_init(&m4_reader[1], (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH);

    vbus_init(&bus_v);
    vbus_init(&bus_p);
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_platform));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_ppeer));
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
    if (mb != MB_OFF) {
        mailbox_boot(mb == MB_REFUSED);
    }
    TEST_ASSERT_TRUE(comms_apply_filters());
}


/* ------------------------------------------------------------------- the M4 model */

static void encode(uint32_t stream, uint32_t k, uint8_t data[CAN_PORT_MAX_DLC])
{
    for (uint32_t j = 0u; j < CAN_PORT_MAX_DLC; j++) {
        data[j] = (uint8_t)((k * 29u) + (j * 31u) + stream);
    }
}

static void m4_write(uint32_t s)
{
    if (!m4_pend[s].pending) {
        return;
    }
    hal_hsem_key_t key;
    if (hal_hsem_take(m4_tx_def[s].hsem_id, &key)) {
        volatile uint32_t* const slot = &region.word[m4_tx_def[s].word];
        mailbox_slot_store(&m4_pend[s], slot);
        hal_hsem_barrier();
        mailbox_slot_commit(slot);
        hal_hsem_release(m4_tx_def[s].hsem_id, &key);
        mailbox_pending_done(&m4_pend[s]);
        if (m4_written[s].n < LOG_CAP) {
            mailbox_payload_bytes(m4_pend[s].data, m4_written[s].data[m4_written[s].n]);
            m4_written[s].n++;
        }
    } else {
        mailbox_pending_defer(&m4_pend[s]);
    }
}

static void m4_read(uint32_t r)
{
    hal_hsem_key_t key;
    mailbox_raw_t raw;
    if (!hal_hsem_take(m4_rx_def[r].hsem_id, &key)) {
        mailbox_reader_busy(&m4_reader[r]);
        return;
    }
    mailbox_slot_load(&region.word[m4_rx_def[r].word], &raw);
    hal_hsem_release(m4_rx_def[r].hsem_id, &key);
    const bool peer_ok = region.word[MAILBOX_M7_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID;
    if (mailbox_reader_accept(&m4_reader[r], &raw, peer_ok, timebase_now_ms()) == MAILBOX_READ_NEW) {
        uint8_t data[MAILBOX_PAYLOAD_BYTES];
        mailbox_payload_bytes(raw.data, data);
        const frame_log_t* const bus = &bus_log[m4_rx_def[r].stream];
        const uint32_t n = m4_seen[r];
        if ((n >= bus->n) || (memcmp(data, bus->data[n], CAN_PORT_MAX_DLC) != 0)) {
            m4_mismatch++;
        }
        const size_t len = (r == 0u) ? PLATFORM_EKF_LEAN_LENGTH : PLATFORM_HEARTBEAT_RT_CORE_LENGTH;
        const vehicle_e2e_status_t st = vehicle_e2e_check(m4_e2e_id[r], m4_e2e_delta[r], m4_e2e_timeout[r], data,
                                                          len, &m4_e2e[r], timebase_now_ms());
        if (st == VEHICLE_E2E_OK) {
            m4_e2e_ok[r]++;
        } else if (!((n == 0u) && (st == VEHICLE_E2E_INITIAL))) {
            m4_e2e_bad[r]++;
        } else {
            /* the first frame resyncs */
        }
        m4_seen[r]++;
    }
}

/* The M4 before pass i: layout word, its frames at their cycles (never in pass 0: the M7
 * takes its baselines then), the pending writes, the reads. */
static void m4_turn(uint32_t i, bool generate)
{
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    region.word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    for (uint32_t s = 0u; s < 2u; s++) {
        if (generate && (i > 0u) && ((i % m4_tx_def[s].cycle_ms) == 0u)) {
            uint8_t data[CAN_PORT_MAX_DLC];
            m4_k[s]++;
            encode(S_WARN + s, m4_k[s], data);
            (void)mailbox_pending_offer(&m4_pend[s], data, CAN_PORT_MAX_DLC);
        }
        m4_write(s);
    }
    m4_read(0u);
    m4_read(1u);
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    if (inject_faults) {
        /* the next take of 0x020's HSEM is refused (usually the M7's write: deferred one
         * pass), and some passes the platform port refuses every write (TX_FULL: the
         * senders keep their counter and retry, so nothing is offered to the mailbox) */
        if ((i % 7u) == 3u) {
            hsem_host_force_busy((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID, 1u);
        }
        vbus_set_tx_blocked(&bus_p, node_platform, (i % 11u) == 5u);
    }
}

/* ------------------------------------------------------------------------- run */

static void note(uint32_t now, uint32_t* frames, uint32_t* last, uint32_t* gap_max)
{
    if ((*frames > 0u) && ((now - *last) > *gap_max)) {
        *gap_max = now - *last;
    }
    *last = now;
    (*frames)++;
}

/* RUN_MS passes of 1 ms. With the M4 model (mode ON or REFUSED) it steps before each pass
 * and once more after the last one. */
static counts_t run(mb_mode_t mode)
{
    counts_t c;
    (void)memset(&c, 0, sizeof c);
    for (uint32_t i = 0u; i < RUN_MS; i++) {
        const uint32_t now = timebase_now_ms();
        if ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u) {
            lean_ekf_step(&lean, now);
        }
        sim_ecu_step(&ecu, now);
        if (mode != MB_OFF) {
            m4_turn(i, true);
        }
        comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx);
        can_frame_t got;
        while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
            if (got.id == PLATFORM_HEARTBEAT_RT_CORE_FRAME_ID) {
                note(now, &c.hb_frames, &c.hb_last, &c.hb_gap_max);
            } else if (got.id == PLATFORM_EKF_LEAN_FRAME_ID) {
                note(now, &c.lean_frames, &c.lean_last, &c.lean_gap_max);
            } else {
                /* 0x021, 0x110: not counted here */
            }
            for (uint32_t s = 0u; s < S_COUNT; s++) {
                if ((got.id == bus_ids[s]) && !got.extended && (bus_log[s].n < LOG_CAP)) {
                    (void)memcpy(bus_log[s].data[bus_log[s].n], got.data, CAN_PORT_MAX_DLC);
                    bus_log[s].n++;
                }
            }
        }
        hal_time_host_advance(1u);
    }
    if (mode != MB_OFF) {
        m4_turn(RUN_MS, false); /* reads what the last pass committed */
    }
    c.health = mailbox_health(timebase_now_ms());
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
    return c;
}

static bool hold_lean_hsem; /* the M4 keeps 0x020's HSEM through its turn (its own take/release would free it) */

/* One pass of run() without the bookkeeping. */
static void step_pass(uint32_t i)
{
    const uint32_t now = timebase_now_ms();
    if ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u) {
        lean_ekf_step(&lean, now);
    }
    sim_ecu_step(&ecu, now);
    m4_turn(i, true);
    if (hold_lean_hsem) {
        hsem_host_seize((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID, HSEM_HOST_CORE_M4);
    }
    comms_pass(&client, &server, &republisher, &heartbeat, &ekf, &tx);
    can_frame_t got;
    while (vbus_recv(&bus_p, node_ppeer, &got) == CAN_PORT_OK) {
        for (uint32_t s = 0u; s < S_COUNT; s++) {
            if ((got.id == bus_ids[s]) && !got.extended && (bus_log[s].n < LOG_CAP)) {
                (void)memcpy(bus_log[s].data[bus_log[s].n], got.data, CAN_PORT_MAX_DLC);
                bus_log[s].n++;
            }
        }
    }
    hal_time_host_advance(1u);
}

static void assert_same_cycle(const counts_t* a, const counts_t* b)
{
    TEST_ASSERT_EQUAL_UINT32(a->hb_frames, b->hb_frames);
    TEST_ASSERT_EQUAL_UINT32(a->lean_frames, b->lean_frames);
    TEST_ASSERT_EQUAL_UINT32(a->hb_gap_max, b->hb_gap_max);
    TEST_ASSERT_EQUAL_UINT32(a->lean_gap_max, b->lean_gap_max);
}

#define WARN_BIT ((uint32_t)1u << PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX)
#define HB_SAFETY_BIT ((uint32_t)1u << PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX)

/* ------------------------------------------------------------------------ tests */

/* First in the file: A needs a module that was never initialised. */
static void test_0x081_and_0x020_keep_their_cycle_whatever_the_mailbox_does(void)
{
    /* A: mailbox_init() never ran */
    rig_up(MB_OFF);
    const counts_t a = run(MB_OFF);
    TEST_ASSERT_FALSE(a.health.initialised);
    TEST_ASSERT_EQUAL_UINT32(RUN_MS, a.health.skipped_steps);
    TEST_ASSERT_TRUE(a.hb_frames > 10u);
    TEST_ASSERT_TRUE(a.lean_frames > 40u);
    TEST_ASSERT_EQUAL_UINT32(0u, bus_log[S_WARN].n + bus_log[S_HB_SAFETY].n);

    /* B: initialised, the M4 steps before every pass */
    rig_up(MB_ON);
    const counts_t b = run(MB_ON);
    TEST_ASSERT_TRUE(b.health.initialised);
    TEST_ASSERT_EQUAL_UINT32(0u, b.health.skipped_steps);
    TEST_ASSERT_TRUE(b.health.write_done > 0u);
    TEST_ASSERT_TRUE(b.health.forward_done > 0u);

    /* C: initialised, every HSEM refused every take */
    rig_up(MB_REFUSED);
    const counts_t c = run(MB_REFUSED);
    TEST_ASSERT_TRUE(c.health.write_deferred > 0u);

    assert_same_cycle(&a, &b);
    assert_same_cycle(&a, &c);
}

static void test_the_m4_gets_exactly_the_committed_frames_and_its_own_reach_the_bus_verbatim(void)
{
    rig_up(MB_ON);
    const counts_t b = run(MB_ON);

    /* M7 -> M4: the n-th frame the M4 accepts is the n-th one sent on the bus, none missing */
    TEST_ASSERT_TRUE(bus_log[S_LEAN].n > 40u);
    TEST_ASSERT_TRUE(bus_log[S_HB_RT].n > 10u);
    TEST_ASSERT_EQUAL_UINT32(bus_log[S_LEAN].n, m4_seen[0]);
    TEST_ASSERT_EQUAL_UINT32(bus_log[S_HB_RT].n, m4_seen[1]);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_mismatch);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_reader[0].bad_layout + m4_reader[0].bad_len);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_reader[1].bad_layout + m4_reader[1].bad_len);
    TEST_ASSERT_EQUAL_UINT32(bus_log[S_LEAN].n + bus_log[S_HB_RT].n, b.health.write_done);
    TEST_ASSERT_EQUAL_UINT32(0u, b.health.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, b.health.write_overruns);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_e2e_bad[0] + m4_e2e_bad[1]); /* never WRONG_SEQUENCE etc. */
    TEST_ASSERT_EQUAL_UINT32(m4_seen[0] - 1u, m4_e2e_ok[0]);
    TEST_ASSERT_EQUAL_UINT32(m4_seen[1] - 1u, m4_e2e_ok[1]);

    /* M4 -> M7: forwarded verbatim, in order, none missing */
    for (uint32_t s = 0u; s < 2u; s++) {
        const frame_log_t* const fwd = &bus_log[S_WARN + s];
        TEST_ASSERT_TRUE(m4_written[s].n > 5u);
        TEST_ASSERT_EQUAL_UINT32(m4_written[s].n, fwd->n);
        for (uint32_t n = 0u; n < fwd->n; n++) {
            TEST_ASSERT_EQUAL_UINT8_ARRAY(m4_written[s].data[n], fwd->data[n], CAN_PORT_MAX_DLC);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(bus_log[S_WARN].n + bus_log[S_HB_SAFETY].n, b.health.forward_done);
    TEST_ASSERT_EQUAL_UINT32(0u, b.health.forward_overruns + b.health.forward_dropped + b.health.forward_deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, b.health.read_busy + b.health.read_rejected);
    TEST_ASSERT_FALSE(b.health.layout_mismatch);
    TEST_ASSERT_EQUAL_UINT32(0u, b.health.stale_slots);
}

static void test_refused_semaphores_defer_the_writes_and_leave_the_cycle_alone(void)
{
    rig_up(MB_REFUSED);
    const counts_t c = run(MB_REFUSED);

    TEST_ASSERT_TRUE(c.lean_frames > 40u);
    TEST_ASSERT_TRUE(c.hb_frames > 10u);
    TEST_ASSERT_TRUE(c.health.write_deferred > 0u);
    TEST_ASSERT_TRUE(c.health.write_overruns > 0u); /* newer frames replace the waiting ones */
    TEST_ASSERT_EQUAL_UINT32(0u, c.health.write_done);
    TEST_ASSERT_TRUE(c.health.read_busy > 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, c.health.forward_done);
    TEST_ASSERT_EQUAL_UINT32(0u, bus_log[S_WARN].n + bus_log[S_HB_SAFETY].n);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_seen[0] + m4_seen[1]);
    TEST_ASSERT_EQUAL_UINT32(WARN_BIT | HB_SAFETY_BIT, c.health.stale_slots);
}

static void test_the_m4_e2e_check_stays_ok_under_refused_hsems_and_tx_full(void)
{
    rig_up(MB_ON);
    inject_faults = true;
    const counts_t f = run(MB_ON);
    inject_faults = false;
    vbus_set_tx_blocked(&bus_p, node_platform, false);
    TEST_ASSERT_TRUE(f.health.write_deferred > 0u); /* the refusals did hit the M7 writes */
    TEST_ASSERT_EQUAL_UINT32(0u, f.health.write_overruns);
    TEST_ASSERT_TRUE(m4_seen[0] > 40u);
    TEST_ASSERT_TRUE(m4_seen[1] > 10u);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_mismatch);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_e2e_bad[0] + m4_e2e_bad[1]);
    TEST_ASSERT_EQUAL_UINT32(m4_seen[0] - 1u, m4_e2e_ok[0]);
    TEST_ASSERT_EQUAL_UINT32(m4_seen[1] - 1u, m4_e2e_ok[1]);
}

/* Bus-off on the platform bus (nothing newer is offered) while the M4 holds 0x020's HSEM
 * longer than the cycle (D-093 amendment 2 addendum 2): the held frame is dropped, and no
 * frame reaches the M4 after the release, so nothing stale is read as valid. */
static void test_a_bus_off_with_the_hsem_held_never_delivers_a_stale_frame(void)
{
    rig_up(MB_ON);
    hold_lean_hsem = false;
    uint32_t i = 0u;
    const uint32_t hsem = (uint32_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID;
    for (; i < 40u; i++) { /* warm-up: a few good frames reach the M4 */
        step_pass(i);
    }
    TEST_ASSERT_TRUE(m4_seen[0] > 0u);
    hold_lean_hsem = true;
    const uint32_t deferred0 = mailbox_health(timebase_now_ms()).write_deferred;
    for (; (i < 100u) && (mailbox_health(timebase_now_ms()).write_deferred == deferred0); i++) {
        step_pass(i);
    }
    TEST_ASSERT_TRUE(mailbox_health(timebase_now_ms()).write_deferred > deferred0); /* a frame is waiting */
    vbus_set_bus_off(&bus_p, node_platform, true); /* nothing newer is offered from now on */
    const uint32_t end = i + (2u * PLATFORM_EKF_LEAN_CYCLE_TIME_MS);
    for (; i < end; i++) {
        step_pass(i);
    }
    TEST_ASSERT_TRUE(mailbox_health(timebase_now_ms()).write_aged >= 1u);
    const uint32_t seen = m4_seen[0];
    hold_lean_hsem = false;
    hsem_host_release_as((uint8_t)hsem, HSEM_HOST_CORE_M4);
    for (uint32_t k = 0u; k < 10u; k++) {
        step_pass(i + k);
    }
    TEST_ASSERT_EQUAL_UINT32(seen, m4_seen[0]); /* the aged frame was never written late */
    TEST_ASSERT_EQUAL_UINT32(0u, m4_mismatch);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_e2e_bad[0]);
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x081_and_0x020_keep_their_cycle_whatever_the_mailbox_does);
    RUN_TEST(test_the_m4_gets_exactly_the_committed_frames_and_its_own_reach_the_bus_verbatim);
    RUN_TEST(test_refused_semaphores_defer_the_writes_and_leave_the_cycle_alone);
    RUN_TEST(test_the_m4_e2e_check_stays_ok_under_refused_hsems_and_tx_full);
    RUN_TEST(test_a_bus_off_with_the_hsem_held_never_delivers_a_stale_frame);
    return UNITY_END();
}
