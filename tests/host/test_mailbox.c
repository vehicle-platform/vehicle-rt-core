/*
 * L0 tests of services/mailbox, the M7 glue of the M7<->M4 mailbox (D-093 items 5-6), on
 * the host HSEM model (hal/host/hsem_host) and the host platform port: inert before init,
 * init touches only the M7 layout word and frees only this core's locks, the ownership
 * from the gen/ table (offer only M7->M4 messages), deferral on a refused HSEM (next
 * pass, no wait), bounded loss as a counted overrun, the layout word rewritten every
 * pass, the M4->M7 baseline and verbatim forward, layout and len mismatches, stale
 * slots, the forward's TX_FULL retry and bus-off drop, and com_send() offering only
 * committed frames. The M4 side is modelled here with services/mailbox_core.
 */
#include "services/mailbox.h"

#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/hsem_host.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/can_sm.h"
#include "services/com.h"
#include "services/diag.h"
#include "vehicle_e2e.h"
#include "platform_uds.h"

#include <string.h>
#include <unity.h>

#define T0 1000u

#define LEAN_ID PLATFORM_MAILBOX_SLOT_EKF_LEAN_FRAME_ID
#define LEAN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH)
#define LEAN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID)
#define LEAN_WORD (PLATFORM_MAILBOX_SLOT_EKF_LEAN_OFFSET / MAILBOX_WORD_BYTES)
#define LEAN_IDX PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX

#define WARN_ID PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_FRAME_ID
#define WARN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH)
#define WARN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID)
#define WARN_WORD (PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_OFFSET / MAILBOX_WORD_BYTES)
#define WARN_IDX PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX
#define WARN_TIMEOUT PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS

#define HBS_IDX PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX
#define HBS_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH)
#define HBS_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID)
#define HBS_WORD (PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_OFFSET / MAILBOX_WORD_BYTES)
#define HBS_TIMEOUT PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS

#define PATTERN 0x5A5A5A5Au

static mailbox_region_t region;
static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;
static uint32_t now;
static mailbox_pending_t m4_writer;
static mailbox_reader_t m4_reader;

static void frame(uint8_t k, uint8_t len, uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    (void)memset(out, 0, MAILBOX_PAYLOAD_BYTES);
    for (uint8_t j = 0u; j < len; j++) {
        out[j] = (uint8_t)(k + (uint8_t)(j * 31u));
    }
}

/* ------------------------------------------------------------------- M4 model */

static void m4_write_layout(uint32_t id)
{
    region.word[MAILBOX_M4_LAYOUT_WORD] = id;
}

/* The M4 writes frame k into the slot at word under its HSEM; false if it was refused. */
static bool m4_write_slot(uint8_t hsem, uint32_t word, uint8_t k, uint8_t len)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(k, len, d);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&m4_writer, d, len));
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    hal_hsem_key_t key;
    const bool ok = hal_hsem_take(hsem, &key);
    if (ok) {
        mailbox_slot_store(&m4_writer, &region.word[word]);
        hal_hsem_barrier();
        mailbox_slot_commit(&region.word[word]);
        hal_hsem_release(hsem, &key);
        mailbox_pending_done(&m4_writer);
    }
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    return ok;
}

/* The M4 writes frame k of 0x023 under its HSEM; false if the HSEM was refused. */
static bool m4_write_warn(uint8_t k, uint8_t len)
{
    return m4_write_slot(WARN_HSEM, WARN_WORD, k, len);
}

/* The M4 reads 0x020; MAILBOX_READ_NONE when the HSEM was refused. */
static mailbox_read_t m4_read_lean(uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    hal_hsem_key_t key;
    mailbox_read_t r = MAILBOX_READ_NONE;
    if (hal_hsem_take(LEAN_HSEM, &key)) {
        mailbox_raw_t raw;
        mailbox_slot_load(&region.word[LEAN_WORD], &raw);
        hal_hsem_release(LEAN_HSEM, &key);
        r = mailbox_reader_accept(&m4_reader, &raw, region.word[MAILBOX_M7_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID,
                                  now);
        mailbox_payload_bytes(raw.data, out);
    }
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    return r;
}

/* ------------------------------------------------------------------------ rig */

void setUp(void)
{
    now = T0;
    hal_time_host_use_manual(T0);
    hsem_host_reset();
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        region.word[i] = PATTERN;
    }
    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init(); /* filters in boot(), after mailbox_init() */
    mailbox_pending_init(&m4_writer);
    mailbox_reader_init(&m4_reader, LEAN_LEN);
    diag_init(T0);
}

void tearDown(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
    (void)mailbox_init(NULL, 0u); /* inert for the next test */
}

static void filters(void)
{
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_PLATFORM);
}

/* The H7 boot order: mailbox_init() (dedicated TX IDs), then the filters. */
static void boot(void)
{
    TEST_ASSERT_TRUE(mailbox_init(&region, now));
    /* the 0x020 sender's own registration (lean_tx_open in production): an E2E-range ID
     * reaches the platform port only through its dedicated buffer (ISSUES E-14 (5)) */
    TEST_ASSERT_EQUAL(CAN_IF_OK, can_if_register_tx_dedicated(CAN_PORT_PLATFORM, LEAN_ID, false));
    filters();
}

static void step(void)
{
    mailbox_step(now);
    now++;
}

static bool peer_frame(can_frame_t* f)
{
    return vbus_recv(&bus_p, node_peer, f) == CAN_PORT_OK;
}

static void offer_lean(uint8_t k)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(k, LEAN_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(LEAN_ID, false, d, LEAN_LEN));
}

/* M4 layout OK, then the first step takes the M4->M7 baselines. */
static void boot_with_m4(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    step();
}

/* ---------------------------------------------------------------------- tests */

static void test_the_host_hsem_model_retakes_for_the_same_core_only(void)
{
    hal_hsem_key_t key;
    TEST_ASSERT_TRUE(hal_hsem_take(LEAN_HSEM, &key));
    TEST_ASSERT_TRUE(hal_hsem_take(LEAN_HSEM, &key)); /* the H7 one-step re-take */
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    TEST_ASSERT_FALSE(hal_hsem_take(LEAN_HSEM, &key));
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    hal_hsem_release(LEAN_HSEM, &key);
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(LEAN_HSEM));
    TEST_ASSERT_FALSE(hal_hsem_take(0u, &key)); /* ID 0 is not a mailbox HSEM */
    TEST_ASSERT_FALSE(hal_hsem_take(HAL_HSEM_ID_MAX + 1u, &key));
    TEST_ASSERT_FALSE(hal_hsem_take(LEAN_HSEM, NULL));
}

static void test_inert_before_init(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(1u, LEAN_LEN, d);
    TEST_ASSERT_FALSE(mailbox_offer(LEAN_ID, false, d, LEAN_LEN));
    step();
    step();
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_FALSE(h.initialised);
    TEST_ASSERT_EQUAL_UINT32(2u, h.skipped_steps);
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        TEST_ASSERT_EQUAL_HEX32(PATTERN, region.word[i]);
    }
    TEST_ASSERT_FALSE(mailbox_init(NULL, now));
}

static void test_init_writes_only_the_m7_layout_word_and_frees_only_own_locks(void)
{
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M7); /* left from before an M7 reset */
    hsem_host_seize(WARN_HSEM, HSEM_HOST_CORE_M4); /* the M4 is busy with its slot */
    boot();
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(LEAN_HSEM));
    TEST_ASSERT_EQUAL(HSEM_HOST_CORE_M4, hsem_host_owner(WARN_HSEM));
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        if (i == MAILBOX_M7_LAYOUT_WORD) {
            TEST_ASSERT_EQUAL_HEX32(PLATFORM_MAILBOX_LAYOUT_ID, region.word[i]);
        } else {
            TEST_ASSERT_EQUAL_HEX32(PATTERN, region.word[i]);
        }
    }
    /* a pass with nothing pending writes nothing else either (the readers only read) */
    hsem_host_release_as(WARN_HSEM, HSEM_HOST_CORE_M4);
    step();
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        if (i != MAILBOX_M7_LAYOUT_WORD) {
            TEST_ASSERT_EQUAL_HEX32(PATTERN, region.word[i]);
        }
    }
    TEST_ASSERT_TRUE(mailbox_health(now).initialised);
}

static void test_offer_takes_only_m7_to_m4_messages_of_their_gen_length(void)
{
    boot();
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(1u, MAILBOX_PAYLOAD_BYTES, d);
    TEST_ASSERT_FALSE(mailbox_offer(WARN_ID, false, d, WARN_LEN)); /* M4->M7: not ours */
    TEST_ASSERT_FALSE(mailbox_offer(PLATFORM_VEHICLE_SPEED_FRAME_ID, false, d, PLATFORM_VEHICLE_SPEED_LENGTH));
    TEST_ASSERT_FALSE(mailbox_offer(LEAN_ID, true, d, LEAN_LEN));
    TEST_ASSERT_FALSE(mailbox_offer(LEAN_ID, false, d, (uint8_t)(LEAN_LEN - 1u)));
    TEST_ASSERT_FALSE(mailbox_offer(LEAN_ID, false, NULL, LEAN_LEN));
    TEST_ASSERT_TRUE(mailbox_offer(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_FRAME_ID, false, d,
                                   (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH));
    TEST_ASSERT_TRUE(mailbox_offer(PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_FRAME_ID, false, d,
                                   (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_LENGTH));
    TEST_ASSERT_TRUE(mailbox_offer(PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_FRAME_ID, false, d,
                                   (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_LENGTH));
}

static void test_a_pending_frame_is_written_in_the_same_pass_and_read_by_the_m4(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, m4_read_lean(out));
    offer_lean(9u);
    step();
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(9u, LEAN_LEN, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    TEST_ASSERT_EQUAL_UINT8(LEAN_LEN, region.layout.slot[LEAN_IDX].len);
    step(); /* nothing pending: no second write */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).write_done);
}

static void test_a_refused_hsem_defers_to_the_next_pass(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    offer_lean(1u);
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M4); /* the M4 holds the slot */
    step();
    hsem_host_release_as(LEAN_HSEM, HSEM_HOST_CORE_M4);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_done);
    step();
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(1u, LEAN_LEN, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_overruns);
}

static void test_a_frame_replaced_while_pending_is_a_counted_overrun(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    hsem_host_force_busy(LEAN_HSEM, 2u);
    offer_lean(1u);
    step();
    offer_lean(2u); /* frame 1 never reached its slot */
    step();
    step();
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(2u, LEAN_LEN, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_overruns);
    TEST_ASSERT_EQUAL_UINT32(2u, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
}

/* D-093 amendment 2 principle, E-28 follow-up: a frame the HSEM kept waiting is dropped once strictly older than its
 * message's gen/ cycle, never written late. */
#define LEAN_CYCLE PLATFORM_EKF_LEAN_CYCLE_TIME_MS

/* The M4 holds 0x020's HSEM from the pass where `hold_passes` steps have run (including the
 * first, which stamps the frame), releases it and runs one more step. */
static void hold_then_release(uint32_t hold_passes)
{
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M4);
    for (uint32_t i = 0u; i < hold_passes; i++) {
        step();
    }
    hsem_host_release_as(LEAN_HSEM, HSEM_HOST_CORE_M4);
    step();
}

static void test_a_deferred_frame_taken_within_its_cycle_is_still_written(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    offer_lean(1u);
    hold_then_release(LEAN_CYCLE); /* refused at ages 0..cycle-1, taken at age == cycle */
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(1u, LEAN_LEN, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(LEAN_CYCLE, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_aged);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_dropped);
}

static void test_a_deferred_frame_older_than_its_cycle_is_dropped_not_written(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    offer_lean(1u);
    hold_then_release(LEAN_CYCLE + 1u); /* the free HSEM is met at age cycle + 1 */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out)); /* never written later either */
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_aged);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_dropped);
    /* a newer frame is written normally and restarts the age */
    offer_lean(2u);
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).write_done);
}

/* A newer offer replaces and restamps: the replaced frame's age is not inherited. */
static void test_a_newer_offer_restamps_the_age(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    offer_lean(1u);
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M4);
    for (uint32_t i = 0u; i < LEAN_CYCLE; i++) {
        step();
    }
    offer_lean(2u); /* replaces frame 1, which was about to age */
    for (uint32_t i = 0u; i < LEAN_CYCLE; i++) {
        step();
    }
    hsem_host_release_as(LEAN_HSEM, HSEM_HOST_CORE_M4);
    step();
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(2u, LEAN_LEN, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_overruns);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_aged);
}

static void test_the_age_limit_holds_across_the_ms_wrap(void)
{
    now = UINT32_MAX - 8u;
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    offer_lean(1u);
    hold_then_release(LEAN_CYCLE); /* spans the wrap, age == cycle: written */
    TEST_ASSERT_LESS_THAN_UINT32(UINT32_MAX / 2u, now);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    offer_lean(2u);
    hold_then_release(LEAN_CYCLE + 1u); /* age cycle + 1: dropped */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_aged);
}

/* End to end with the M4's real E2E check: com_send() offers nothing newer (FDCAN2 bus-off)
 * while the M4 holds the HSEM past the cycle. The held frame is dropped; the next frame
 * reads as WRONG_SEQUENCE on the M4 (counter step 2), never as OK with a stale payload. */
static void test_an_aged_frame_never_reaches_the_m4_as_a_stale_step_of_one(void)
{
    vehicle_e2e_tx_state_t tx;
    vehicle_e2e_rx_state_t rx;
    vehicle_e2e_tx_init(&tx);
    vehicle_e2e_rx_init(&rx);
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    for (uint32_t k = 0u; k < 4u; k++) {
        uint8_t d[MAILBOX_PAYLOAD_BYTES] = {(uint8_t)(0xA0u + k), 1u, 2u, 3u, 4u, 5u, 6u, 0u};
        TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, vehicle_e2e_protect(PLATFORM_EKF_LEAN_E2E_DATA_ID, d, LEAN_LEN, &tx));
        TEST_ASSERT_TRUE(mailbox_offer(LEAN_ID, false, d, LEAN_LEN));
        if (k == 3u) {
            hold_then_release(LEAN_CYCLE + 1u); /* frame 3 ages out; nothing newer is offered */
            TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
            step();
            TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
        } else {
            step();
            TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
            const vehicle_e2e_status_t st = vehicle_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID,
                                                              PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER, PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS,
                                                              out, LEAN_LEN, &rx, now);
            TEST_ASSERT_EQUAL((k == 0u) ? VEHICLE_E2E_INITIAL : VEHICLE_E2E_OK, st);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).write_aged);
    /* the sender goes on: the M4 sees the gap, within the E2E timeout */
    uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0xB0u, 1u, 2u, 3u, 4u, 5u, 6u, 0u};
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, vehicle_e2e_protect(PLATFORM_EKF_LEAN_E2E_DATA_ID, d, LEAN_LEN, &tx));
    TEST_ASSERT_TRUE(mailbox_offer(LEAN_ID, false, d, LEAN_LEN));
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE,
                      vehicle_e2e_check(PLATFORM_EKF_LEAN_E2E_DATA_ID, PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER,
                                        PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, out, LEAN_LEN, &rx, now));
}

/* A 100 ms slot (0x081): the limit is its own cycle, so a swapped cycle_ms row (0x020's 20 ms
 * on it, or 0x081's 100 ms on 0x020) is caught. */
#define HBR_ID PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_FRAME_ID
#define HBR_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH)
#define HBR_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID)
#define HBR_CYCLE PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS

static void hold_hbr_then_release(uint32_t hold_passes)
{
    hsem_host_seize(HBR_HSEM, HSEM_HOST_CORE_M4);
    for (uint32_t i = 0u; i < hold_passes; i++) {
        step();
    }
    hsem_host_release_as(HBR_HSEM, HSEM_HOST_CORE_M4);
    step();
}

static void offer_hbr(uint8_t k)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(k, HBR_LEN, d);
    TEST_ASSERT_TRUE(mailbox_offer(HBR_ID, false, d, HBR_LEN));
}

static void test_a_100_ms_slot_ages_at_its_own_cycle_not_at_another_slots(void)
{
    TEST_ASSERT_TRUE(HBR_CYCLE > (2u * LEAN_CYCLE)); /* the cases below tell the rows apart */
    boot_with_m4();
    offer_hbr(1u);
    hold_hbr_then_release(HBR_CYCLE); /* age == 100: written (a 20 ms row would drop it) */
    mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_aged);
    offer_hbr(2u);
    hold_hbr_then_release(HBR_CYCLE + 1u); /* age == 101: dropped */
    h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_aged);
    /* and 0x020 in the same run still ages at 20 ms (a 100 ms row would write it) */
    offer_lean(3u);
    hold_then_release(LEAN_CYCLE + 1u);
    h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(2u, h.write_aged);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
}

/* An offer made between passes (after a step, before the next one) is stamped by the next
 * step: the drop comes at most one pass after offer + cycle + 1, never earlier. */
static void test_an_offer_after_a_step_ages_out_within_one_pass_of_the_cycle(void)
{
    boot_with_m4();
    step();
    const uint32_t t_offer = now; /* the pass that stamps it runs one ms later */
    offer_lean(1u);
    now++;
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M4);
    uint32_t t_drop = 0u;
    for (uint32_t i = 0u; (i < (2u * LEAN_CYCLE)) && (t_drop == 0u); i++) {
        const uint32_t t = now;
        step();
        if (mailbox_health(now).write_aged != 0u) {
            t_drop = t;
        }
    }
    hsem_host_release_as(LEAN_HSEM, HSEM_HOST_CORE_M4);
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, t_drop);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(LEAN_CYCLE + 1u, t_drop - t_offer); /* never before the cycle */
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(LEAN_CYCLE + 2u, t_drop - t_offer);    /* one pass of lag at most */
}

/* E-28 (5): nothing goes into an M4 region with another layout, and a frame offered
 * during the mismatch is dropped, never written late once the word matches. */
static void test_a_mismatched_m4_layout_gets_no_write_and_the_frame_is_dropped(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, m4_read_lean(out));
    const uint32_t seq_before = region.layout.slot[LEAN_IDX].write_seq;
    uint32_t slot_before[MAILBOX_SLOT_WORDS];
    for (uint32_t j = 0u; j < MAILBOX_SLOT_WORDS; j++) {
        slot_before[j] = region.word[LEAN_WORD + j];
    }
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u); /* an M4 image built from other defs */
    offer_lean(1u);
    step();
    for (uint32_t j = 0u; j < MAILBOX_SLOT_WORDS; j++) {
        TEST_ASSERT_EQUAL_HEX32(slot_before[j], region.word[LEAN_WORD + j]);
    }
    TEST_ASSERT_EQUAL_UINT32(seq_before, region.layout.slot[LEAN_IDX].write_seq);
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(LEAN_HSEM)); /* not even taken */
    mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_done);
    /* the word matches again: the dropped frame is never written, the next one is */
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    offer_lean(2u);
    step();
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(2u, LEAN_LEN, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_overruns);
}

/* A frame deferred by a refused HSEM and then caught by a mismatch is dropped, never
 * written after the match (safety-reviewer 2026-11-04, test 1). */
static void test_a_deferred_frame_caught_by_a_mismatch_is_dropped(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    const uint32_t seq_before = region.layout.slot[LEAN_IDX].write_seq;
    offer_lean(1u);
    hsem_host_seize(LEAN_HSEM, HSEM_HOST_CORE_M4);
    step(); /* deferred */
    hsem_host_release_as(LEAN_HSEM, HSEM_HOST_CORE_M4);
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u);
    step(); /* dropped */
    TEST_ASSERT_EQUAL(HSEM_HOST_FREE, hsem_host_owner(LEAN_HSEM));
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    step();
    step();
    TEST_ASSERT_EQUAL_UINT32(seq_before, region.layout.slot[LEAN_IDX].write_seq);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_deferred);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_done);
}

/* Each frame offered into a mismatch is one drop (not an overrun), on every M7->M4 slot:
 * here the rt-core heartbeat, the costlier loss on the M4 (safety-reviewer tests 2-3). */
static void test_heartbeat_frames_offered_into_a_mismatch_are_each_dropped(void)
{
    boot_with_m4();
    const uint32_t hb_idx = PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_INDEX;
    const uint8_t hb_len = (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH;
    const uint32_t seq_before = region.layout.slot[hb_idx].write_seq;
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u);
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    for (uint8_t k = 1u; k <= 2u; k++) {
        frame(k, hb_len, d);
        TEST_ASSERT_TRUE(mailbox_offer(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_FRAME_ID, false, d, hb_len));
        step();
    }
    TEST_ASSERT_EQUAL_UINT32(seq_before, region.layout.slot[hb_idx].write_seq);
    mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(2u, h.write_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, h.write_overruns);
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    frame(3u, hb_len, d);
    TEST_ASSERT_TRUE(mailbox_offer(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_FRAME_ID, false, d, hb_len));
    step();
    TEST_ASSERT_EQUAL_UINT32(seq_before + 1u, region.layout.slot[hb_idx].write_seq);
    TEST_ASSERT_EQUAL_UINT8(d[0], (uint8_t)(region.layout.slot[hb_idx].data[0]));
    h = mailbox_health(now);
    TEST_ASSERT_EQUAL_UINT32(1u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(2u, h.write_dropped);
}

static void test_a_wiped_m7_layout_word_recovers_in_one_pass(void)
{
    boot();
    region.word[MAILBOX_M7_LAYOUT_WORD] = 0u;
    step();
    TEST_ASSERT_EQUAL_HEX32(PLATFORM_MAILBOX_LAYOUT_ID, region.word[MAILBOX_M7_LAYOUT_WORD]);
}

static void test_a_new_m4_frame_is_forwarded_verbatim_once(void)
{
    m4_write_warn(1u, WARN_LEN); /* left from before the M7 boot: baseline only */
    boot_with_m4();
    can_frame_t f;
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_TRUE(m4_write_warn(2u, WARN_LEN));
    step();
    TEST_ASSERT_TRUE(peer_frame(&f));
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(2u, WARN_LEN, want);
    TEST_ASSERT_EQUAL_UINT32(WARN_ID, f.id);
    TEST_ASSERT_FALSE(f.extended);
    TEST_ASSERT_EQUAL_UINT8(WARN_LEN, f.dlc);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, f.data, WARN_LEN);
    step(); /* unchanged write_seq: not again */
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).forward_done);
}

static void test_a_busy_m4_slot_is_read_in_the_next_pass(void)
{
    boot_with_m4();
    TEST_ASSERT_TRUE(m4_write_warn(1u, WARN_LEN));
    hsem_host_seize(WARN_HSEM, HSEM_HOST_CORE_M4);
    step();
    can_frame_t f;
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).read_busy);
    hsem_host_release_as(WARN_HSEM, HSEM_HOST_CORE_M4);
    step();
    TEST_ASSERT_TRUE(peer_frame(&f));
}

static void test_a_layout_mismatch_forwards_nothing_and_raises_the_flag_after_the_grace(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u); /* an M4 image built from other defs */
    boot();
    const uint32_t t_first = now;
    step(); /* baseline, mismatch starts */
    TEST_ASSERT_TRUE(m4_write_warn(1u, WARN_LEN));
    step();
    can_frame_t f;
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).read_rejected);
    /* grace = the shortest M4->M7 E2E timeout */
    const uint32_t grace = (WARN_TIMEOUT < HBS_TIMEOUT) ? WARN_TIMEOUT : HBS_TIMEOUT;
    TEST_ASSERT_FALSE(mailbox_health(t_first + grace).layout_mismatch);
    TEST_ASSERT_TRUE(mailbox_health(t_first + grace + 1u).layout_mismatch);
    /* the M4 fixes its word: the flag clears at the next pass, and the frame written
     * under the mismatch is never forwarded */
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    now = t_first + grace + 2u;
    step();
    TEST_ASSERT_FALSE(mailbox_health(now).layout_mismatch);
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_TRUE(m4_write_warn(2u, WARN_LEN));
    step();
    TEST_ASSERT_TRUE(peer_frame(&f));
}

static void test_a_bad_len_from_the_m4_is_not_forwarded(void)
{
    boot_with_m4();
    TEST_ASSERT_TRUE(m4_write_warn(1u, (uint8_t)(WARN_LEN - 1u)));
    step();
    can_frame_t f;
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).read_rejected);
}

static void test_m4_slots_without_new_frames_read_as_stale(void)
{
    boot_with_m4(); /* baseline at T0 */
    const uint32_t warn_bit = (uint32_t)1u << WARN_IDX;
    const uint32_t hbs_bit = (uint32_t)1u << HBS_IDX;
    TEST_ASSERT_EQUAL_HEX32(0u, mailbox_health(T0 + WARN_TIMEOUT).stale_slots & warn_bit);
    TEST_ASSERT_EQUAL_HEX32(warn_bit, mailbox_health(T0 + WARN_TIMEOUT + 1u).stale_slots & warn_bit);
    TEST_ASSERT_EQUAL_HEX32(hbs_bit, mailbox_health(T0 + HBS_TIMEOUT + 1u).stale_slots & hbs_bit);
    /* only M4->M7 slots are supervised */
    TEST_ASSERT_EQUAL_HEX32(warn_bit | hbs_bit, mailbox_health(T0 + 100000u).stale_slots);
    /* a new frame makes 0x023 fresh again */
    now = T0 + WARN_TIMEOUT + 5u;
    TEST_ASSERT_TRUE(m4_write_warn(1u, WARN_LEN));
    step();
    TEST_ASSERT_EQUAL_HEX32(0u, mailbox_health(now).stale_slots & warn_bit);
}

/* -------------------------------------------- DTCs (D-093 amendment 3, E-28 (3)) */

#define DTC_MISMATCH PLATFORM_UDS_DTC_IDX_SAFETY_MAILBOX_LAYOUT_MISMATCH
#define DTC_FROZEN PLATFORM_UDS_DTC_IDX_SAFETY_MAILBOX_SLOT_FROZEN
#define TEST_FAILED_BIT 0x01u

static bool dtc_failed(uint32_t idx)
{
    return (diag_dtc_status(idx) & TEST_FAILED_BIT) != 0u;
}

/* Steps every ms up to (excluding) until; a writing M4 refreshes 0x023 and 0x082 on their
 * gen/ cycles, counted from the call. */
static void run_until(uint32_t until, bool m4_writes)
{
    uint32_t k = 0u;
    const uint32_t from = now;
    while (now < until) {
        const uint32_t t = now - from;
        if (m4_writes && ((t % PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS) == 0u)) {
            TEST_ASSERT_TRUE(m4_write_slot(WARN_HSEM, WARN_WORD, (uint8_t)k, WARN_LEN));
        }
        if (m4_writes && ((t % PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS) == 0u)) {
            TEST_ASSERT_TRUE(m4_write_slot(HBS_HSEM, HBS_WORD, (uint8_t)k, HBS_LEN));
        }
        k++;
        step();
    }
}

/* Wrap-safe: steps ms passes (run_until compares absolute times). */
static void run_for(uint32_t ms, bool m4_writes)
{
    for (uint32_t t = 0u; t < ms; t++) {
        if (m4_writes && ((t % PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS) == 0u)) {
            TEST_ASSERT_TRUE(m4_write_slot(WARN_HSEM, WARN_WORD, (uint8_t)t, WARN_LEN));
        }
        if (m4_writes && ((t % PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS) == 0u)) {
            TEST_ASSERT_TRUE(m4_write_slot(HBS_HSEM, HBS_WORD, (uint8_t)t, HBS_LEN));
        }
        step();
    }
}

static void test_a_healthy_m4_raises_no_mailbox_dtc(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + 3000u, true);
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_an_m4_that_never_matches_fails_the_mismatch_dtc_after_the_boot_budget(void)
{
    boot(); /* the M4 word still holds the power-up pattern */
    run_until(T0 + MAILBOX_PEER_BOOT_BUDGET_MS + 1u, false);
    /* the old flag is up after the 60 ms grace, but a booting M4 is not a mismatch */
    TEST_ASSERT_TRUE(mailbox_health(now).layout_mismatch);
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
    step(); /* T0 + budget + 1 */
    TEST_ASSERT_TRUE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    /* diagnostic only (addendum): no heartbeat DEGRADED, no ERROR_COUNT onset */
    TEST_ASSERT_FALSE(diag_fault_active());
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
    const mailbox_health_t h = mailbox_health(now);
    TEST_ASSERT_TRUE(h.peer_missing);
    TEST_ASSERT_TRUE(h.dtc_layout_mismatch);
    TEST_ASSERT_FALSE(h.dtc_slot_frozen);
    /* a matching M4 that finally writes clears it in the next pass */
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    run_until(now + 1000u, true);
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_NOT_EQUAL(0u, diag_dtc_status(DTC_MISMATCH) & 0x08u); /* confirmed stays */
}

static void test_an_m4_up_within_the_boot_budget_raises_nothing(void)
{
    boot();
    run_until(T0 + MAILBOX_PEER_BOOT_BUDGET_MS - 100u, false);
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    run_until(T0 + (3u * MAILBOX_PEER_BOOT_BUDGET_MS), true);
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_slots_not_yet_written_after_a_match_raise_nothing_within_their_timeout(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    /* the M4 matches at once but writes its first frames late, each inside its timeout */
    run_until(T0 + WARN_TIMEOUT - 10u, false);
    /* 0x023 from here on its cycle; 0x082 only just inside its own timeout */
    uint8_t k = 1u;
    while (now < (T0 + HBS_TIMEOUT - 10u)) {
        TEST_ASSERT_TRUE(m4_write_slot(WARN_HSEM, WARN_WORD, k, WARN_LEN));
        k++;
        run_until(now + PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS, false);
    }
    TEST_ASSERT_TRUE(m4_write_slot(HBS_HSEM, HBS_WORD, 1u, HBS_LEN));
    step();
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
    run_until(T0 + 2000u, true);
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_a_matched_m4_that_stops_writing_fails_only_the_frozen_dtc(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + 1000u, true);
    const uint32_t last = now - 1u - ((now - 1u - T0) % PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS);
    run_until(last + WARN_TIMEOUT + 2u, false); /* 0x023 had no frame for its timeout */
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_EQUAL_HEX32((uint32_t)1u << WARN_IDX,
                            mailbox_health(now).frozen_slots & ((uint32_t)1u << WARN_IDX));
    TEST_ASSERT_FALSE(diag_fault_active());
    run_until(now + 500u, true); /* it writes again */
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_a_mismatch_after_a_match_fails_only_the_mismatch_dtc(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + 1000u, true);
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID ^ 1u); /* an M4 reflashed from other defs */
    const uint32_t from = now;
    run_until(from + WARN_TIMEOUT + 1u, true);
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH)); /* within the grace */
    run_until(from + HBS_TIMEOUT + 100u, true);
    TEST_ASSERT_TRUE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN)); /* the slots are stale too: one root cause */
    TEST_ASSERT_NOT_EQUAL(0u, mailbox_health(now).stale_slots);
    TEST_ASSERT_FALSE(mailbox_health(now).peer_missing); /* it had matched */
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    run_until(now + 1000u, true);
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_a_warm_reset_with_a_matching_word_raises_nothing_inside_the_budget(void)
{
    /* SRAM4 kept the M4 word over the reset; the M4 is still initialising */
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + WARN_TIMEOUT + HBS_TIMEOUT, false); /* first frames far past timeout */
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    run_until(T0 + MAILBOX_PEER_BOOT_BUDGET_MS - 100u, false);
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    run_until(T0 + (2u * MAILBOX_PEER_BOOT_BUDGET_MS), true);
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
}

static void test_a_matched_m4_that_never_writes_fails_frozen_after_the_boot_budget(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + MAILBOX_PEER_BOOT_BUDGET_MS + 1u, false);
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    step();
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_EQUAL_HEX32(((uint32_t)1u << WARN_IDX) | ((uint32_t)1u << HBS_IDX),
                            mailbox_health(now).frozen_slots);
}

static void test_only_a_stuck_0x082_fails_frozen_with_its_bit_alone(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + 1000u, true);
    uint8_t k = 1u;
    const uint32_t until = now + HBS_TIMEOUT + 150u;
    while (now < until) { /* 0x023 healthy on its cycle, 0x082 no longer written */
        TEST_ASSERT_TRUE(m4_write_slot(WARN_HSEM, WARN_WORD, k, WARN_LEN));
        k++;
        run_until(now + PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS, false);
    }
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_EQUAL_HEX32((uint32_t)1u << HBS_IDX, mailbox_health(now).frozen_slots);
}

static void test_a_short_word_wipe_keeps_a_failing_frozen_dtc(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + 1000u, true);
    run_until(now + HBS_TIMEOUT + 10u, false); /* both slots frozen */
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    m4_write_layout(0u); /* wiped for one pass */
    step();
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    step();
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN)); /* still frozen, no passed gap */
    m4_write_layout(0u); /* now a mismatch that outlasts the grace: the mismatch DTC */
    run_until(now + WARN_TIMEOUT + 2u, false);
    TEST_ASSERT_TRUE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
}

static void test_a_clear_while_failing_sets_the_status_again_on_the_next_pass(void)
{
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    boot();
    run_until(T0 + 1000u, true);
    run_until(now + WARN_TIMEOUT + 5u, false);
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    diag_dtc_clear_all();
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    step();
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_the_dtc_latches_hold_across_the_ms_wrap(void)
{
    now = UINT32_MAX - 500u; /* the boot budget and the matched run cross the wrap */
    hal_time_host_use_manual(now);
    diag_init(now);
    boot();
    run_for(MAILBOX_PEER_BOOT_BUDGET_MS + 2u, false);
    TEST_ASSERT_TRUE(now < 5000u); /* wrapped */
    TEST_ASSERT_TRUE(dtc_failed(DTC_MISMATCH)); /* never matched, budget over */
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    run_for(1000u, true);
    TEST_ASSERT_FALSE(dtc_failed(DTC_MISMATCH));
    TEST_ASSERT_FALSE(dtc_failed(DTC_FROZEN));
    run_for(HBS_TIMEOUT + 10u, false);
    TEST_ASSERT_TRUE(dtc_failed(DTC_FROZEN));
}

static void test_dropped_writes_alone_raise_no_dtc(void)
{
    boot(); /* M7 first: its heartbeat frames are dropped until the M4 matches */
    offer_lean(1u);
    step();
    TEST_ASSERT_NOT_EQUAL(0u, mailbox_health(now).write_dropped);
    m4_write_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    run_until(T0 + 1000u, true);
    TEST_ASSERT_EQUAL_UINT32(0u, diag_fault_onsets());
}

static void test_an_inert_mailbox_reports_no_dtc(void)
{
    mailbox_step(T0 + 10000u);
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(DTC_MISMATCH));
    TEST_ASSERT_EQUAL_HEX8(0u, diag_dtc_status(DTC_FROZEN));
}

static void test_the_forward_retries_on_tx_full_and_drops_on_bus_off(void)
{
    boot_with_m4();
    TEST_ASSERT_TRUE(m4_write_warn(1u, WARN_LEN));
    vbus_set_tx_blocked(&bus_p, node_dut, true);
    step();
    can_frame_t f;
    TEST_ASSERT_FALSE(peer_frame(&f));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).forward_deferred);
    vbus_set_tx_blocked(&bus_p, node_dut, false);
    step();
    TEST_ASSERT_TRUE(peer_frame(&f));
    /* a newer M4 frame replaces a forward the bus did not take yet */
    vbus_set_tx_blocked(&bus_p, node_dut, true);
    TEST_ASSERT_TRUE(m4_write_warn(2u, WARN_LEN));
    step();
    TEST_ASSERT_TRUE(m4_write_warn(3u, WARN_LEN));
    step();
    vbus_set_tx_blocked(&bus_p, node_dut, false);
    step();
    TEST_ASSERT_TRUE(peer_frame(&f));
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    frame(3u, WARN_LEN, want);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, f.data, WARN_LEN);
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).forward_overruns);
    /* bus-off: given up, not retried */
    vbus_set_bus_off(&bus_p, node_dut, true);
    can_sm_step(CAN_PORT_PLATFORM);
    TEST_ASSERT_TRUE(m4_write_warn(4u, WARN_LEN));
    step();
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).forward_dropped);
    TEST_ASSERT_EQUAL_UINT32(2u, mailbox_health(now).forward_done);
}

static void test_the_forwarded_ids_get_dedicated_tx_buffers(void)
{
    boot();
    can_frame_t f;
    (void)memset(&f, 0, sizeof f);
    f.id = WARN_ID;
    f.dlc = WARN_LEN;
    TEST_ASSERT_TRUE(vbus_tx_is_dedicated(&bus_p, node_dut, &f));
    f.id = PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_FRAME_ID;
    f.dlc = (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH;
    TEST_ASSERT_TRUE(vbus_tx_is_dedicated(&bus_p, node_dut, &f));
    /* a re-init after the filters is refused (the port is sealed) and leaves it inert */
    TEST_ASSERT_FALSE(mailbox_init(&region, now));
    TEST_ASSERT_FALSE(mailbox_health(now).initialised);
}

static void test_init_after_the_filters_is_refused_and_inert(void)
{
    filters(); /* the port is sealed: no dedicated buffer can be added */
    TEST_ASSERT_FALSE(mailbox_init(&region, now));
    TEST_ASSERT_FALSE(mailbox_health(now).initialised);
    for (uint32_t i = 0u; i < MAILBOX_REGION_WORDS; i++) {
        TEST_ASSERT_EQUAL_HEX32(PATTERN, region.word[i]); /* nothing written */
    }
    step();
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).skipped_steps);
}

static void test_com_send_offers_only_committed_frames(void)
{
    boot_with_m4();
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    (void)m4_read_lean(out);
    com_msg_stats_t stats;
    (void)memset(&stats, 0, sizeof stats);
    can_port_status_t st = CAN_PORT_ERR_ARG;
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    /* TX_FULL: the sender keeps its counter, so the mailbox gets nothing yet */
    frame(1u, LEAN_LEN, d);
    vbus_set_tx_blocked(&bus_p, node_dut, true);
    TEST_ASSERT_FALSE(com_send(LEAN_ID, false, d, LEAN_LEN, &stats, &st));
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    /* accepted by the port: the same frame goes to the M4 */
    vbus_set_tx_blocked(&bus_p, node_dut, false);
    TEST_ASSERT_TRUE(com_send(LEAN_ID, false, d, LEAN_LEN, &stats, &st));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, st);
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(d, out, MAILBOX_PAYLOAD_BYTES);
    /* another platform message is not a mailbox slot */
    TEST_ASSERT_TRUE(com_send(PLATFORM_VEHICLE_ENGINE_FRAME_ID, false, d, PLATFORM_VEHICLE_ENGINE_LENGTH, &stats, &st));
    step();
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, m4_read_lean(out));
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).write_done);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_host_hsem_model_retakes_for_the_same_core_only);
    RUN_TEST(test_inert_before_init);
    RUN_TEST(test_init_writes_only_the_m7_layout_word_and_frees_only_own_locks);
    RUN_TEST(test_offer_takes_only_m7_to_m4_messages_of_their_gen_length);
    RUN_TEST(test_a_pending_frame_is_written_in_the_same_pass_and_read_by_the_m4);
    RUN_TEST(test_a_refused_hsem_defers_to_the_next_pass);
    RUN_TEST(test_a_frame_replaced_while_pending_is_a_counted_overrun);
    RUN_TEST(test_a_deferred_frame_taken_within_its_cycle_is_still_written);
    RUN_TEST(test_a_deferred_frame_older_than_its_cycle_is_dropped_not_written);
    RUN_TEST(test_a_newer_offer_restamps_the_age);
    RUN_TEST(test_a_100_ms_slot_ages_at_its_own_cycle_not_at_another_slots);
    RUN_TEST(test_an_offer_after_a_step_ages_out_within_one_pass_of_the_cycle);
    RUN_TEST(test_the_age_limit_holds_across_the_ms_wrap);
    RUN_TEST(test_an_aged_frame_never_reaches_the_m4_as_a_stale_step_of_one);
    RUN_TEST(test_a_mismatched_m4_layout_gets_no_write_and_the_frame_is_dropped);
    RUN_TEST(test_a_deferred_frame_caught_by_a_mismatch_is_dropped);
    RUN_TEST(test_heartbeat_frames_offered_into_a_mismatch_are_each_dropped);
    RUN_TEST(test_a_wiped_m7_layout_word_recovers_in_one_pass);
    RUN_TEST(test_a_new_m4_frame_is_forwarded_verbatim_once);
    RUN_TEST(test_a_busy_m4_slot_is_read_in_the_next_pass);
    RUN_TEST(test_a_layout_mismatch_forwards_nothing_and_raises_the_flag_after_the_grace);
    RUN_TEST(test_a_bad_len_from_the_m4_is_not_forwarded);
    RUN_TEST(test_m4_slots_without_new_frames_read_as_stale);
    RUN_TEST(test_the_forward_retries_on_tx_full_and_drops_on_bus_off);
    RUN_TEST(test_the_forwarded_ids_get_dedicated_tx_buffers);
    RUN_TEST(test_init_after_the_filters_is_refused_and_inert);
    RUN_TEST(test_com_send_offers_only_committed_frames);
    RUN_TEST(test_a_healthy_m4_raises_no_mailbox_dtc);
    RUN_TEST(test_an_m4_that_never_matches_fails_the_mismatch_dtc_after_the_boot_budget);
    RUN_TEST(test_an_m4_up_within_the_boot_budget_raises_nothing);
    RUN_TEST(test_slots_not_yet_written_after_a_match_raise_nothing_within_their_timeout);
    RUN_TEST(test_a_matched_m4_that_stops_writing_fails_only_the_frozen_dtc);
    RUN_TEST(test_a_mismatch_after_a_match_fails_only_the_mismatch_dtc);
    RUN_TEST(test_a_warm_reset_with_a_matching_word_raises_nothing_inside_the_budget);
    RUN_TEST(test_a_matched_m4_that_never_writes_fails_frozen_after_the_boot_budget);
    RUN_TEST(test_only_a_stuck_0x082_fails_frozen_with_its_bit_alone);
    RUN_TEST(test_a_short_word_wipe_keeps_a_failing_frozen_dtc);
    RUN_TEST(test_a_clear_while_failing_sets_the_status_again_on_the_next_pass);
    RUN_TEST(test_the_dtc_latches_hold_across_the_ms_wrap);
    RUN_TEST(test_dropped_writes_alone_raise_no_dtc);
    RUN_TEST(test_an_inert_mailbox_reports_no_dtc);
    return UNITY_END();
}
