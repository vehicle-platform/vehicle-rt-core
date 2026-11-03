/*
 * Two-thread stress test of the M7<->M4 mailbox (D-093 items 5 and 6): the real M7 side
 * (services/mailbox.c on services/mailbox_core.c) in one thread, a model of the M4 in
 * another, sharing only the mailbox region and the six slot semaphores of the host HSEM
 * model (hal/host/hsem_host.c). Run under ASan + UBSan (host-tests) and ThreadSanitizer
 * (host-tsan preset): TSan reports any region word the two sides touch without the slot's
 * HSEM, the checks below report a torn, repeated or out-of-order frame.
 *
 * The M4 model uses mailbox_core exactly as D-093 item 5 prescribes: the writer of the
 * M4->M7 slots (SAFETY_WARNING_STATUS and HEARTBEAT_SAFETY) takes the slot's HSEM, stores
 * the payload, barriers, commits write_seq, releases; a refused take defers the frame to
 * its next iteration, and a newer frame replaces a deferred one (an overrun, counted). It
 * is also the reader of the M7->M4 slot EKF_LEAN (baseline, accept).
 * Layout words: they are the one part of the region that is shared without a lock (one
 * aligned 32-bit word, written by its owner only; single-copy atomic on the Cortex-M7/M4,
 * D-093 item 5). The glue accesses them as plain volatile words, which the host's memory
 * model (and TSan) cannot express without atomics in target code, so here they stay out of
 * the concurrent phase: the M4 word is written before the threads start, the glue's own
 * word is read by the M4 model once before them as well. Layout mismatch, wipe and
 * recovery are covered deterministically by test_mailbox and test_mailbox_interleave.
 *
 * Frame k carries k in data[0..3] and k ^ CHECK_MASK in data[4..7], so a payload mixed from
 * two frames never decodes. The checks, for every frame either side receives: untorn,
 * strictly increasing k (no repeat, no older frame), and the gaps counted.
 *
 * Loss accounting (the key property: a lost frame is always counted, none torn, none
 * repeated, none out of order). Frames are lost in exactly two ways, each counted:
 *   - a pending frame replaced by a newer one before its slot write (the writer's overrun);
 *   - a committed frame overwritten in the slot before the reader took it. This is not
 *     counted by either side (the receiver's E2E check sees it, D-093 item 5), and on the
 *     real chip it needs the reader to miss a whole message cycle, so the test does not let
 *     it happen: a writer starts a new frame only when every frame it committed before
 *     has reached the reader (M4 acknowledges what it accepted, the M7 side compares it
 *     with write_done; the M4 compares the forwards the M7 thread has seen with its own
 *     commits). Both directions still contend freely for the HSEMs (deferrals, overruns).
 * Hence, once both sides are drained: offered == accepted + overruns on the M7->M4 slot,
 * generated == forwarded + overruns on each M4->M7 slot, and every observed gap is covered
 * by a counted overrun.
 *
 * Unity asserts only in the main thread; the workers count what they saw. All loops are
 * bounded. Every cross-thread value goes through the HSEMs or __atomic builtins.
 */
#define _POSIX_C_SOURCE 200809L
#include "services/mailbox.h"

#include "hal/hsem.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/hsem_host.h"
#include "hal/host/vbus.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/can_sm.h"

#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define M7_STEPS 200000u        /* comms passes of the M7 thread */
#define OFFER_EVERY 8u          /* an EKF_LEAN frame offered every 8th step */
#define M4_GEN_EVERY 3u         /* the M4 starts a new frame every 3rd iteration */
#define HOOK_YIELD_EVERY 7u     /* every 7th held take yields: widens the windows */
#define LOOP_YIELD_EVERY 64u    /* both loops yield now and then (few cores) */
#define M4_MAX_ITERATIONS 400000000u
#define DRAIN_ROUNDS 6u
#define CHECK_MASK 0xA5A5A5A5u
#define BYTE_BITS 8u
#define M4_STREAMS 2u

_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_DIRECTION == PLATFORM_MAILBOX_DIR_M7_TO_M4, "EKF_LEAN is M7->M4");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_DIRECTION == PLATFORM_MAILBOX_DIR_M4_TO_M7,
               "SAFETY_WARNING_STATUS is M4->M7");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_DIRECTION == PLATFORM_MAILBOX_DIR_M4_TO_M7,
               "HEARTBEAT_SAFETY is M4->M7");
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH == MAILBOX_PAYLOAD_BYTES, "the frame encoding fills 8 bytes");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH == MAILBOX_PAYLOAD_BYTES, "8 bytes");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH == MAILBOX_PAYLOAD_BYTES, "8 bytes");

/* ------------------------------------------------------------------ frame coding */

static void put_le32(uint8_t* p, uint32_t v)
{
    for (uint32_t i = 0u; i < 4u; i++) {
        p[i] = (uint8_t)(v >> (BYTE_BITS * i));
    }
}

static uint32_t get_le32(const uint8_t* p)
{
    uint32_t v = 0u;
    for (uint32_t i = 0u; i < 4u; i++) {
        v |= ((uint32_t)p[i]) << (BYTE_BITS * i);
    }
    return v;
}

static void frame_encode(uint32_t k, uint8_t data[MAILBOX_PAYLOAD_BYTES])
{
    put_le32(&data[0], k);
    put_le32(&data[4], k ^ CHECK_MASK);
}

static bool frame_decode(const uint8_t* data, uint32_t* k)
{
    *k = get_le32(&data[0]);
    return get_le32(&data[4]) == (*k ^ CHECK_MASK);
}

typedef struct {
    uint32_t seen;           /* frames received (torn ones included) */
    uint32_t torn;           /* a payload that does not decode */
    uint32_t not_increasing; /* a repeated or older k */
    uint32_t gaps;           /* k values skipped between received frames */
    uint32_t last_k;
} stream_check_t;

static void check_frame(stream_check_t* c, const uint8_t* data)
{
    uint32_t k = 0u;
    c->seen++;
    if (!frame_decode(data, &k)) {
        c->torn++;
        return;
    }
    if (k <= c->last_k) {
        c->not_increasing++;
        return;
    }
    c->gaps += k - c->last_k - 1u;
    c->last_k = k;
}

/* ------------------------------------------------------------------------- rig */

typedef struct {
    uint32_t frame_id;
    uint8_t hsem_id;
    uint32_t word; /* first region word of the slot */
} slot_ref_t;

static const slot_ref_t m4_streams[M4_STREAMS] = {
    {PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_FRAME_ID, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
     PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_OFFSET / MAILBOX_WORD_BYTES},
    {PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_FRAME_ID, (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
     PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_OFFSET / MAILBOX_WORD_BYTES},
};

static const slot_ref_t lean_slot = {PLATFORM_MAILBOX_SLOT_EKF_LEAN_FRAME_ID,
                                     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID,
                                     PLATFORM_MAILBOX_SLOT_EKF_LEAN_OFFSET / MAILBOX_WORD_BYTES};

static mailbox_region_t region;
static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;

/* M7 side (thread A, then the main thread once A is joined). */
static uint32_t m7_now;
static uint32_t m7_offered;
static uint32_t fwd_unexpected;
static stream_check_t fwd[M4_STREAMS];

/* M4 side (thread B, then the main thread once B is joined). */
static mailbox_pending_t m4_pend[M4_STREAMS];
static uint32_t m4_gen_k[M4_STREAMS];
static mailbox_reader_t m4_reader;
static stream_check_t m4_chk;
static uint32_t m4_rejected;
static uint32_t m4_now;

/* Cross-thread (atomics only). */
static uint32_t m4_acks;             /* EKF_LEAN frames the M4 accepted */
static uint32_t fwd_seen[M4_STREAMS]; /* forwards the M7 thread took off the bus */
static uint32_t m7_done;
static uint32_t hook_counter;

void setUp(void) {}
void tearDown(void) {}

static void yield_hook(uint8_t id)
{
    (void)id;
    if ((__atomic_fetch_add(&hook_counter, 1u, __ATOMIC_RELAXED) % HOOK_YIELD_EVERY) == 0u) {
        (void)sched_yield();
    }
}

static void rig_up(void)
{
    hal_time_host_use_manual(START_MS);
    hsem_host_reset();
    (void)memset(&region, 0, sizeof region);
    (void)memset(fwd, 0, sizeof fwd);
    (void)memset(&m4_chk, 0, sizeof m4_chk);
    (void)memset(m4_gen_k, 0, sizeof m4_gen_k);
    m7_now = START_MS;
    m7_offered = 0u;
    fwd_unexpected = 0u;
    m4_rejected = 0u;
    m4_now = START_MS;
    __atomic_store_n(&m4_acks, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&m7_done, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&hook_counter, 0u, __ATOMIC_RELAXED);
    for (uint32_t s = 0u; s < M4_STREAMS; s++) {
        __atomic_store_n(&fwd_seen[s], 0u, __ATOMIC_RELAXED);
        mailbox_pending_init(&m4_pend[s]);
    }
    mailbox_reader_init(&m4_reader, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH);

    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init(); /* filters after mailbox_init() (dedicated TX IDs) */
}

/* ------------------------------------------------------------------- the M7 side */

/* Forwarded M4 frames on the peer node of the platform bus; the bus is touched by the
 * M7 thread only (vbus is not thread-safe). */
static void m7_drain_bus(void)
{
    can_frame_t f;
    while (vbus_recv(&bus_p, node_peer, &f) == CAN_PORT_OK) {
        uint32_t s = M4_STREAMS;
        for (uint32_t i = 0u; i < M4_STREAMS; i++) {
            if ((f.id == m4_streams[i].frame_id) && !f.extended) {
                s = i;
            }
        }
        if ((s == M4_STREAMS) || (f.dlc != MAILBOX_PAYLOAD_BYTES)) {
            fwd_unexpected++;
            continue;
        }
        check_frame(&fwd[s], f.data);
        __atomic_store_n(&fwd_seen[s], fwd[s].seen, __ATOMIC_RELEASE);
    }
}

/* One comms pass of the mailbox. offer: a new EKF_LEAN frame first, but only when every
 * frame committed so far has been accepted by the M4 (no slot overwrite, see the header). */
static void m7_pass(bool offer)
{
    if (offer && (mailbox_health(m7_now).write_done == __atomic_load_n(&m4_acks, __ATOMIC_ACQUIRE))) {
        uint8_t data[MAILBOX_PAYLOAD_BYTES];
        frame_encode(m7_offered + 1u, data);
        if (mailbox_offer(lean_slot.frame_id, false, data, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH)) {
            m7_offered++;
        }
    }
    mailbox_step(m7_now);
    m7_now++;
    m7_drain_bus();
}

static void* m7_thread(void* arg)
{
    (void)arg;
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    for (uint32_t step = 0u; step < M7_STEPS; step++) {
        m7_pass((step % OFFER_EVERY) == 0u);
        if ((step % LOOP_YIELD_EVERY) == 0u) {
            (void)sched_yield();
        }
    }
    __atomic_store_n(&m7_done, 1u, __ATOMIC_RELEASE);
    return NULL;
}

/* ------------------------------------------------------------------- the M4 model */

/* Writer, D-093 item 5: a refused take defers the pending frame to the next iteration. */
static void m4_write(uint32_t s)
{
    if (!m4_pend[s].pending) {
        return;
    }
    hal_hsem_key_t key;
    if (hal_hsem_take(m4_streams[s].hsem_id, &key)) {
        volatile uint32_t* const slot = &region.word[m4_streams[s].word];
        mailbox_slot_store(&m4_pend[s], slot);
        hal_hsem_barrier();
        mailbox_slot_commit(slot);
        hal_hsem_release(m4_streams[s].hsem_id, &key);
        mailbox_pending_done(&m4_pend[s]);
    } else {
        mailbox_pending_defer(&m4_pend[s]);
    }
}

/* Reader of EKF_LEAN: baseline, then accept; a taken frame is acknowledged to the M7. */
/* The M7 layout word as the M4 model read it before the threads started (see the top). */
static bool m4_peer_ok;

static void m4_read_lean(void)
{
    hal_hsem_key_t key;
    mailbox_raw_t raw;
    if (!hal_hsem_take(lean_slot.hsem_id, &key)) {
        mailbox_reader_busy(&m4_reader);
        return;
    }
    mailbox_slot_load(&region.word[lean_slot.word], &raw);
    hal_hsem_release(lean_slot.hsem_id, &key);
    const mailbox_read_t r = mailbox_reader_accept(&m4_reader, &raw, m4_peer_ok, m4_now);
    if (r == MAILBOX_READ_NEW) {
        uint8_t data[MAILBOX_PAYLOAD_BYTES];
        mailbox_payload_bytes(raw.data, data);
        check_frame(&m4_chk, data);
        __atomic_store_n(&m4_acks, m4_reader.frames, __ATOMIC_RELEASE);
    } else if ((r == MAILBOX_READ_BAD_LAYOUT) || (r == MAILBOX_READ_BAD_LEN)) {
        m4_rejected++;
    } else {
        /* NONE or BASELINE: no frame this iteration */
    }
}

/* One M4 iteration: layout word, new frames (only when all committed ones were forwarded),
 * the pending writes, the EKF_LEAN read. */
static void m4_iteration(bool generate, uint32_t it)
{
    for (uint32_t s = 0u; s < M4_STREAMS; s++) {
        if (generate && ((it % M4_GEN_EVERY) == 0u) &&
            (__atomic_load_n(&fwd_seen[s], __ATOMIC_ACQUIRE) == m4_pend[s].done)) {
            uint8_t data[MAILBOX_PAYLOAD_BYTES];
            m4_gen_k[s]++;
            frame_encode(m4_gen_k[s], data);
            (void)mailbox_pending_offer(&m4_pend[s], data, (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH);
        }
        m4_write(s);
    }
    m4_read_lean();
    m4_now++;
}

static void* m4_thread(void* arg)
{
    (void)arg;
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    uint32_t it = 0u;
    while ((__atomic_load_n(&m7_done, __ATOMIC_ACQUIRE) == 0u) && (it < M4_MAX_ITERATIONS)) {
        m4_iteration(true, it);
        it++;
        if ((it % LOOP_YIELD_EVERY) == 0u) {
            (void)sched_yield();
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------ test */

static void test_two_cores_never_tear_repeat_or_lose_an_uncounted_frame(void)
{
    rig_up();
    TEST_ASSERT_TRUE(mailbox_init(&region, START_MS));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_PLATFORM);
    /* The M4 booted: its layout word is up and its reader has its baseline before the M7
     * thread offers or reads anything (a frame before the baseline is never used, D-093). */
    region.word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    mailbox_step(m7_now); /* the M7 readers take their baselines (write_seq 0) */
    m4_peer_ok = region.word[MAILBOX_M7_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID;
    m7_now++;
    hsem_host_set_core(HSEM_HOST_CORE_M4);
    m4_read_lean();
    hsem_host_set_core(HSEM_HOST_CORE_M7);
    TEST_ASSERT_TRUE(m4_reader.has_baseline);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_reader.frames);

    hsem_host_set_held_hook(yield_hook);
    pthread_t a;
    pthread_t b;
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&b, NULL, m4_thread, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&a, NULL, m7_thread, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(a, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(b, NULL));
    hsem_host_set_held_hook(NULL);

    /* Quiesce on the main thread: alternate M7 passes (commit, read, forward) and M4
     * iterations (pending writes, EKF_LEAN read) without new frames, until both are idle. */
    for (uint32_t round = 0u; round < DRAIN_ROUNDS; round++) {
        m7_pass(false);
        hsem_host_set_core(HSEM_HOST_CORE_M4);
        m4_iteration(false, 1u);
        hsem_host_set_core(HSEM_HOST_CORE_M7);
    }
    const mailbox_health_t h = mailbox_health(m7_now);

    printf("mailbox stress: M7->M4 %u offered, %u accepted, %u overruns, %u deferred M7 writes, %u busy M4 reads; "
           "M4->M7 %u+%u generated, %u+%u forwarded\n",
           (unsigned)m7_offered, (unsigned)m4_chk.seen, (unsigned)h.write_overruns, (unsigned)h.write_deferred,
           (unsigned)m4_reader.busy, (unsigned)m4_gen_k[0], (unsigned)m4_gen_k[1], (unsigned)fwd[0].seen,
           (unsigned)fwd[1].seen);

    /* M7 -> M4 (EKF_LEAN): untorn, in order, every loss counted. */
    TEST_ASSERT_EQUAL_UINT32(0u, m4_chk.torn);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_chk.not_increasing);
    TEST_ASSERT_EQUAL_UINT32(0u, m4_rejected);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, h.write_done);
    TEST_ASSERT_EQUAL_UINT32(h.write_done, m4_chk.seen);
    TEST_ASSERT_EQUAL_UINT32(m7_offered, m4_chk.seen + h.write_overruns);
    TEST_ASSERT_TRUE(m4_chk.gaps <= h.write_overruns);

    /* M4 -> M7 (forwarded verbatim): untorn, in order, every loss counted. */
    uint32_t forwards = 0u;
    uint32_t gaps = 0u;
    uint32_t m4_overruns = 0u;
    for (uint32_t s = 0u; s < M4_STREAMS; s++) {
        TEST_ASSERT_EQUAL_UINT32(0u, fwd[s].torn);
        TEST_ASSERT_EQUAL_UINT32(0u, fwd[s].not_increasing);
        TEST_ASSERT_GREATER_THAN_UINT32(0u, fwd[s].seen);
        TEST_ASSERT_FALSE(m4_pend[s].pending);
        TEST_ASSERT_EQUAL_UINT32(m4_pend[s].done, fwd[s].seen);
        TEST_ASSERT_EQUAL_UINT32(m4_gen_k[s], fwd[s].seen + m4_pend[s].overruns);
        forwards += fwd[s].seen;
        gaps += fwd[s].gaps;
        m4_overruns += m4_pend[s].overruns;
    }
    TEST_ASSERT_EQUAL_UINT32(0u, fwd_unexpected);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, h.forward_done);
    TEST_ASSERT_EQUAL_UINT32(forwards, h.forward_done);
    TEST_ASSERT_TRUE(gaps <= (m4_overruns + h.forward_overruns));
    TEST_ASSERT_EQUAL_UINT32(0u, h.forward_overruns);
    TEST_ASSERT_EQUAL_UINT32(0u, h.forward_dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, h.read_rejected);
    TEST_ASSERT_FALSE(h.layout_mismatch);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, hsem_host_refused_count()); /* the cores did contend */

    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_two_cores_never_tear_repeat_or_lose_an_uncounted_frame);
    return UNITY_END();
}
