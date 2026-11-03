/*
 * Deterministic interleaving tests of the M7 side of the M7<->M4 mailbox (D-093 items 5
 * and 6, ISO 26262-6 section 9): the real services/mailbox.c and mailbox_core.c, linked
 * against a single-threaded test double of hal/hsem.h that calls a hook after every
 * operation of the M7 (take, release, barrier, release_own), like the hal_atomic double of
 * test_snapshot_interleave.c. The double models one owner per semaphore with the core M7 or
 * M4 and the one-step same-core re-take of the real HSEM; the hook runs ONE action of the
 * M4 at exactly the point where the real M4, running in parallel, can change the outcome.
 * The double replaces hsem_host, so this test calls no hsem_host_* function.
 *
 * Scenario: up to STEPS M7 passes, an EKF_LEAN frame offered before the passes of a mask
 * (offers at least two passes apart, so a deferral of one pass loses nothing), and before
 * every pass the regular turn of the M4: its layout word, a new SAFETY_WARNING_STATUS frame
 * and a read of EKF_LEAN. For every mask, every action and every operation index k of the
 * M7's pass sequence (counted in a dry run) the action is injected right after operation k:
 *   - the M4 reads EKF_LEAN (refused while the M7 holds the semaphore: no frame);
 *   - the M4 writes one more SAFETY_WARNING_STATUS frame;
 *   - the M4 wipes its layout word (rewritten by its next turn);
 *   - the same, but its next turn does not rewrite it (it stays wiped for a whole pass);
 *   - the M4 starts wiped and restores its layout word at k;
 *   - the M4 dies mid-write: takes the SAFETY_WARNING_STATUS semaphore, stores the payload
 *     words of the next frame, and never commits or releases.
 * The oracle follows what the M7 does, never what the code under test decides: a frame is
 * read when the M7 takes the slot's semaphore (the M4 cannot commit while the M7 holds it),
 * so the frame it gets is the last one the M4 committed before that take; it is forwarded
 * when its write_seq is new and the M4 layout word matched at the start of the pass, and
 * absorbed (never delivered, not even later) when the word differed. After each pass:
 *   - the forwarded frames are exactly the expected ones, untorn, strictly increasing,
 *     none repeated, none absorbed (the M4->M7 side);
 *   - the M7 layout word is back (the test wipes it before one pass);
 * and at the end: the M4 saw every EKF_LEAN frame offered before a pass whose M4 layout
 * word matched at its start, untorn and in order, and every frame offered before a pass
 * with a differing word was dropped and counted, never written (E-28 (5)); no overrun or
 * deferral counted (the M7->M4 side); after the M4 died, every later take is refused
 * (read_busy accounts for them), nothing torn is forwarded, and the slot is stale in
 * mailbox_health() after its E2E timeout.
 *
 * Also here: the phase-locked contention case (the M4 holds EKF_LEAN exactly when the M7
 * tries to write, for one to three passes per frame: deferred, never lost, consecutive),
 * and the layout-word recovery and mismatch grace (D-093 item 5).
 */
#include "services/mailbox.h"

#include "hal/hsem.h"
#include "hal/host/can_port_host.h"
#include "hal/host/hal_time_host.h"
#include "hal/host/vbus.h"
#include "platform.h"
#include "platform_e2e.h"
#include "services/can_if.h"
#include "services/can_sm.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#define START_MS 1000u
#define STEPS 4u
#define MAX_FRAMES 16u
#define CHECK_MASK 0xA5A5A5A5u
#define BYTE_BITS 8u
#define WIPE_M7_STEP 2u /* the test wipes the M7 layout word before this pass */

#define WARN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID)
#define HB_SAFETY_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID)
#define LEAN_HSEM ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID)
#define WARN_WORD (PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_OFFSET / MAILBOX_WORD_BYTES)
#define LEAN_WORD (PLATFORM_MAILBOX_SLOT_EKF_LEAN_OFFSET / MAILBOX_WORD_BYTES)
#define WARN_STALE_BIT ((uint32_t)1u << PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX)
#define LEAN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH)
#define WARN_LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH)

_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_DIRECTION == PLATFORM_MAILBOX_DIR_M7_TO_M4, "EKF_LEAN is M7->M4");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_DIRECTION == PLATFORM_MAILBOX_DIR_M4_TO_M7,
               "SAFETY_WARNING_STATUS is M4->M7");
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH == MAILBOX_PAYLOAD_BYTES, "the frame encoding fills 8 bytes");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH == MAILBOX_PAYLOAD_BYTES, "8 bytes");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS > (STEPS + 2u), "a few passes are never stale");
_Static_assert(PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS > (STEPS + 2u), "a few passes are never stale");

/* ------------------------------------------------------------ hal/hsem.h test double */

typedef enum { CORE_NONE = 0, CORE_M7 = 1, CORE_M4 = 2 } core_t;

#define HSEM_SLOTS (HAL_HSEM_ID_MAX + 1u)

static uint8_t owner[HSEM_SLOTS];
static core_t cur_core;
static uint32_t suppress;     /* > 0: the M4 (or the test) is running: nothing counted or hooked */
static bool counting;         /* the scenario's passes are running */
static uint32_t op_count;     /* M7 operations so far in the scenario */
static int32_t inject_at;     /* operation index after which inject_fn runs; < 0: never */
static void (*inject_fn)(void);
static bool injected;
static void (*observe_fn)(uint8_t id);
static uint32_t m7_read_refusals; /* M7 takes of an M4->M7 slot refused */

static void m4_enter(void)
{
    suppress++;
    cur_core = CORE_M4;
}

static void m4_leave(void)
{
    cur_core = CORE_M7;
    suppress--;
}

/* Called after every operation: the injection point (the M7 has just done operation k). */
static void op_done(void)
{
    if ((suppress != 0u) || !counting) {
        return;
    }
    const uint32_t idx = op_count++;
    if (!injected && (inject_at >= 0) && (idx == (uint32_t)inject_at) && (inject_fn != NULL)) {
        injected = true;
        m4_enter();
        inject_fn();
        m4_leave();
    }
}

bool hal_hsem_take(uint8_t id, hal_hsem_key_t* key)
{
    if ((id < HAL_HSEM_ID_MIN) || (id > HAL_HSEM_ID_MAX) || (key == NULL)) {
        return false;
    }
    key->irq_state = 0u;
    const bool ok = (owner[id] == (uint8_t)CORE_NONE) || (owner[id] == (uint8_t)cur_core);
    const bool m7_run = (suppress == 0u) && counting && (cur_core == CORE_M7);
    if (ok) {
        owner[id] = (uint8_t)cur_core;
        if (m7_run && (observe_fn != NULL)) {
            observe_fn(id);
        }
    } else if (m7_run && ((id == WARN_HSEM) || (id == HB_SAFETY_HSEM))) {
        m7_read_refusals++;
    } else {
        /* a refused M7 write take: counted by the module as a deferral */
    }
    op_done();
    return ok;
}

void hal_hsem_release(uint8_t id, const hal_hsem_key_t* key)
{
    (void)key;
    if ((id >= HAL_HSEM_ID_MIN) && (id <= HAL_HSEM_ID_MAX) && (owner[id] == (uint8_t)cur_core)) {
        owner[id] = (uint8_t)CORE_NONE;
    }
    op_done();
}

void hal_hsem_barrier(void)
{
    op_done();
}

void hal_hsem_release_own(uint8_t id)
{
    if ((id >= HAL_HSEM_ID_MIN) && (id <= HAL_HSEM_ID_MAX) && (owner[id] == (uint8_t)cur_core)) {
        owner[id] = (uint8_t)CORE_NONE;
    }
    op_done();
}

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

/* ------------------------------------------------------------------------- rig */

static mailbox_region_t region;
static vbus_t bus_p;
static vbus_t bus_v;
static uint8_t node_dut;
static uint8_t node_peer;
static uint8_t node_vehicle;
static char ctx[96];

#define CHECK_EQ(expected, actual) TEST_ASSERT_EQUAL_UINT32_MESSAGE((expected), (actual), ctx)
#define CHECK_TRUE(cond) TEST_ASSERT_TRUE_MESSAGE((cond), ctx)

typedef enum {
    ACT_NONE = 0,
    ACT_M4_READ_LEAN,
    ACT_M4_WRITE_WARN,
    ACT_M4_WIPE,
    ACT_M4_WIPE_HELD,
    ACT_M4_RESTORE,
    ACT_M4_DIE,
    ACT_COUNT
} action_t;

typedef enum { LAYOUT_EVERY_TURN = 0, LAYOUT_SKIP_NEXT, LAYOUT_NEVER } layout_mode_t;

/* The M4 model. */
static mailbox_pending_t m4_pend;
static uint32_t m4_pend_k;
static uint32_t m4_k;         /* warning frames generated */
static uint32_t m4_committed; /* k of the frame in the slot (0: none) */
static layout_mode_t m4_layout_mode;
static bool m4_dead;
static mailbox_reader_t m4_reader;
static uint32_t lean_seen[MAX_FRAMES];
static uint32_t lean_seen_n;
static uint32_t lean_torn;
static uint32_t lean_not_increasing;
static uint32_t lean_rejected;
static uint32_t lean_expect[MAX_FRAMES]; /* EKF_LEAN frames the M7 must write, in order */
static uint32_t lean_expect_n;
static uint32_t lean_drop_expect;       /* EKF_LEAN frames the M7 must drop (M4 word differs) */
static uint32_t lean_drops_total;       /* over all scenarios: the drop path really ran */

/* The M7 side: offers and the forwards seen on the bus. */
static uint32_t m7_offered;
static uint32_t expect[MAX_FRAMES];
static uint32_t expect_n;
static uint32_t got[MAX_FRAMES];
static uint32_t got_n;
static uint32_t got_torn;
static uint32_t got_unexpected;
static bool absorbed[MAX_FRAMES + 1u];
static uint32_t m7_last_read; /* committed k at the M7's last take of the warning slot */
static bool step_peer_ok;     /* the M4 layout word matched when the current pass started */

static void rig_up(void)
{
    hal_time_host_use_manual(START_MS);
    (void)memset(&region, 0, sizeof region);
    (void)memset(owner, 0, sizeof owner);
    cur_core = CORE_M7;
    suppress = 0u;
    counting = false;
    op_count = 0u;
    inject_at = -1;
    inject_fn = NULL;
    observe_fn = NULL;
    injected = false;
    m7_read_refusals = 0u;

    mailbox_pending_init(&m4_pend);
    mailbox_reader_init(&m4_reader, LEAN_LEN);
    m4_pend_k = 0u;
    m4_k = 0u;
    m4_committed = 0u;
    m4_layout_mode = LAYOUT_EVERY_TURN;
    m4_dead = false;
    lean_seen_n = 0u;
    lean_torn = 0u;
    lean_not_increasing = 0u;
    lean_rejected = 0u;
    lean_expect_n = 0u;
    lean_drop_expect = 0u;
    m7_offered = 0u;
    expect_n = 0u;
    got_n = 0u;
    got_torn = 0u;
    got_unexpected = 0u;
    (void)memset(absorbed, 0, sizeof absorbed);
    m7_last_read = 0u;
    step_peer_ok = true;

    vbus_init(&bus_p);
    vbus_init(&bus_v);
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_dut));
    TEST_ASSERT_TRUE(vbus_attach(&bus_p, &node_peer));
    TEST_ASSERT_TRUE(vbus_attach(&bus_v, &node_vehicle));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_PLATFORM, &bus_p, node_dut));
    TEST_ASSERT_TRUE(can_port_host_bind_vbus(CAN_PORT_VEHICLE, &bus_v, node_vehicle));
    can_if_init(); /* filters after mailbox_init() (dedicated TX IDs) */
}

static void rig_down(void)
{
    can_port_host_unbind_all();
    hal_time_host_use_monotonic();
}

/* ------------------------------------------------------------------- the M4 model */

static void m4_commit_pending(void)
{
    if (!m4_pend.pending) {
        return;
    }
    hal_hsem_key_t key;
    if (hal_hsem_take(WARN_HSEM, &key)) {
        volatile uint32_t* const slot = &region.word[WARN_WORD];
        mailbox_slot_store(&m4_pend, slot);
        hal_hsem_barrier();
        mailbox_slot_commit(slot);
        hal_hsem_release(WARN_HSEM, &key);
        mailbox_pending_done(&m4_pend);
        m4_committed = m4_pend_k;
    } else {
        mailbox_pending_defer(&m4_pend); /* kept for the next turn */
    }
}

static void m4_new_warn(void)
{
    uint8_t data[MAILBOX_PAYLOAD_BYTES];
    m4_k++;
    frame_encode(m4_k, data);
    (void)mailbox_pending_offer(&m4_pend, data, WARN_LEN);
    m4_pend_k = m4_k;
    m4_commit_pending();
}

static void m4_read_lean(void)
{
    hal_hsem_key_t key;
    mailbox_raw_t raw;
    if (!hal_hsem_take(LEAN_HSEM, &key)) {
        mailbox_reader_busy(&m4_reader); /* "no frame" */
        return;
    }
    mailbox_slot_load(&region.word[LEAN_WORD], &raw);
    hal_hsem_release(LEAN_HSEM, &key);
    const bool peer_ok = region.word[MAILBOX_M7_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID;
    const mailbox_read_t r = mailbox_reader_accept(&m4_reader, &raw, peer_ok, START_MS);
    if (r == MAILBOX_READ_NEW) {
        uint8_t data[MAILBOX_PAYLOAD_BYTES];
        uint32_t k = 0u;
        mailbox_payload_bytes(raw.data, data);
        if (!frame_decode(data, &k)) {
            lean_torn++;
        } else if ((lean_seen_n > 0u) && (k <= lean_seen[lean_seen_n - 1u])) {
            lean_not_increasing++;
        } else if (lean_seen_n < MAX_FRAMES) {
            lean_seen[lean_seen_n++] = k;
        } else {
            lean_not_increasing++; /* more frames than were ever offered */
        }
    } else if ((r == MAILBOX_READ_BAD_LAYOUT) || (r == MAILBOX_READ_BAD_LEN)) {
        lean_rejected++;
    } else {
        /* NONE or BASELINE */
    }
}

/* The M4's regular turn, before each M7 pass. A dead M4 does nothing. */
static void m4_turn(void)
{
    if (m4_dead) {
        return;
    }
    m4_enter();
    if (m4_layout_mode == LAYOUT_EVERY_TURN) {
        region.word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    } else if (m4_layout_mode == LAYOUT_SKIP_NEXT) {
        m4_layout_mode = LAYOUT_EVERY_TURN; /* this turn leaves the word as it is */
    } else {
        /* LAYOUT_NEVER: wiped until the test restores it */
    }
    m4_commit_pending();
    m4_new_warn();
    m4_read_lean();
    m4_leave();
}

/* ------------------------------------------------------------- injected M4 actions */

static void act_read_lean(void)
{
    m4_read_lean();
}

static void act_write_warn(void)
{
    m4_new_warn();
}

static void act_wipe(void)
{
    region.word[MAILBOX_M4_LAYOUT_WORD] = 0u;
}

static void act_wipe_held(void)
{
    region.word[MAILBOX_M4_LAYOUT_WORD] = 0u;
    m4_layout_mode = LAYOUT_SKIP_NEXT;
}

static void act_restore(void)
{
    region.word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    m4_layout_mode = LAYOUT_EVERY_TURN;
}

/* Takes the warning slot's semaphore and stores the next frame's payload words, then the
 * core is gone: no commit, no release. A refused take (the M7 holds it) is no death. */
static void act_die(void)
{
    hal_hsem_key_t key;
    if (hal_hsem_take(WARN_HSEM, &key)) {
        mailbox_pending_t half;
        uint8_t data[MAILBOX_PAYLOAD_BYTES];
        mailbox_pending_init(&half);
        frame_encode(m4_k + 1u, data);
        (void)mailbox_pending_offer(&half, data, WARN_LEN);
        mailbox_slot_store(&half, &region.word[WARN_WORD]);
        m4_dead = true;
    }
}

static void (*action_fn(action_t a))(void)
{
    switch (a) {
    case ACT_M4_READ_LEAN:
        return act_read_lean;
    case ACT_M4_WRITE_WARN:
        return act_write_warn;
    case ACT_M4_WIPE:
        return act_wipe;
    case ACT_M4_WIPE_HELD:
        return act_wipe_held;
    case ACT_M4_RESTORE:
        return act_restore;
    case ACT_M4_DIE:
        return act_die;
    default:
        return NULL;
    }
}

/* ------------------------------------------------------------------------ oracle */

/* The M7 took the warning slot: it now holds the last committed frame. */
static void observe_take(uint8_t id)
{
    if (id != WARN_HSEM) {
        return;
    }
    const uint32_t k = m4_committed;
    if (k != m7_last_read) {
        m7_last_read = k;
        if (step_peer_ok) {
            if (expect_n < MAX_FRAMES) {
                expect[expect_n++] = k;
            }
        } else if (k <= MAX_FRAMES) {
            absorbed[k] = true;
        } else {
            /* not reached: at most MAX_FRAMES frames per scenario */
        }
    }
}

static void drain_bus(void)
{
    can_frame_t f;
    while (vbus_recv(&bus_p, node_peer, &f) == CAN_PORT_OK) {
        uint32_t k = 0u;
        if ((f.id != PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_FRAME_ID) || f.extended || (f.dlc != WARN_LEN)) {
            got_unexpected++;
        } else if (!frame_decode(f.data, &k)) {
            got_torn++;
        } else if (got_n < MAX_FRAMES) {
            got[got_n++] = k;
        } else {
            got_unexpected++;
        }
    }
}

static void offer_lean(void)
{
    uint8_t data[MAILBOX_PAYLOAD_BYTES];
    m7_offered++;
    frame_encode(m7_offered, data);
    CHECK_TRUE(mailbox_offer(PLATFORM_MAILBOX_SLOT_EKF_LEAN_FRAME_ID, false, data, LEAN_LEN));
}

/* Everything before the first counted M7 operation: boot, baselines (write_seq 0 on both
 * sides). m4_wiped: the M4's layout word is 0 and stays so until the test restores it. */
static void begin_run(bool m4_wiped)
{
    rig_up();
    TEST_ASSERT_TRUE(mailbox_init(&region, START_MS));
    TEST_ASSERT_EQUAL(CAN_PORT_OK, can_if_apply_filters(CAN_PORT_PLATFORM));
    can_sm_step(CAN_PORT_PLATFORM);
    region.word[MAILBOX_M4_LAYOUT_WORD] = m4_wiped ? 0u : PLATFORM_MAILBOX_LAYOUT_ID;
    m4_layout_mode = m4_wiped ? LAYOUT_NEVER : LAYOUT_EVERY_TURN;
    mailbox_step(START_MS); /* the M7 readers take their baselines */
    m4_enter();
    m4_read_lean(); /* the M4 reader takes its baseline */
    m4_leave();
    drain_bus();
    TEST_ASSERT_EQUAL_UINT32(0u, got_n + got_torn + got_unexpected);
    step_peer_ok = !m4_wiped;
}

/* One scenario; returns the number of M7 operations it ran. */
static uint32_t scenario(action_t act, int32_t at, uint32_t offer_mask)
{
    (void)snprintf(ctx, sizeof ctx, "action %d after M7 operation %d, offer mask 0x%x", (int)act, (int)at,
                   (unsigned)offer_mask);
    begin_run(act == ACT_M4_RESTORE);
    observe_fn = observe_take;
    inject_fn = action_fn(act);
    inject_at = (inject_fn != NULL) ? at : -1;
    counting = true;
    op_count = 0u;

    uint32_t now = START_MS;
    for (uint32_t s = 0u; s < STEPS; s++) {
        now++;
        m4_turn();
        const bool offered = (offer_mask & ((uint32_t)1u << s)) != 0u;
        if (offered) {
            offer_lean();
        }
        if (s == WIPE_M7_STEP) {
            region.word[MAILBOX_M7_LAYOUT_WORD] = 0u; /* the M7 must restore it in this pass */
        }
        step_peer_ok = region.word[MAILBOX_M4_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID;
        if (offered) {
            /* the M7 reads the M4 word before its first operation of the pass */
            if (!step_peer_ok) {
                lean_drop_expect++;
            } else if (lean_expect_n < MAX_FRAMES) {
                lean_expect[lean_expect_n++] = m7_offered;
            } else {
                /* not reached: at most MAX_FRAMES offers per scenario */
            }
        }
        mailbox_step(now);
        drain_bus();

        CHECK_EQ(PLATFORM_MAILBOX_LAYOUT_ID, region.word[MAILBOX_M7_LAYOUT_WORD]);
        CHECK_EQ(expect_n, got_n); /* forwarded in the pass it was read */
        CHECK_EQ(0u, got_torn);
        CHECK_EQ(0u, got_unexpected);
    }
    counting = false;
    if (!m4_dead) {
        m4_enter();
        m4_read_lean(); /* the M4's last turn: picks up the last committed frame */
        m4_leave();
    }
    const uint32_t ops = op_count;

    /* M4 -> M7: the forwards are exactly the frames read while the layout matched. */
    for (uint32_t i = 0u; i < expect_n; i++) {
        CHECK_EQ(expect[i], got[i]);
        CHECK_TRUE(!absorbed[got[i]]);
        if (i > 0u) {
            CHECK_TRUE(got[i] > got[i - 1u]);
        }
    }
    if (act != ACT_M4_DIE) {
        CHECK_TRUE(injected || (inject_fn == NULL) || (at >= (int32_t)ops));
    }

    /* M7 -> M4: every frame offered into a matching M4, untorn and in order; every frame
     * offered while the M4 word differed dropped and counted; no overrun or deferral. */
    const mailbox_health_t h = mailbox_health(now);
    CHECK_EQ(0u, lean_torn);
    CHECK_EQ(0u, lean_not_increasing);
    CHECK_EQ(0u, lean_rejected);
    CHECK_EQ(0u, h.write_overruns);
    CHECK_EQ(0u, h.write_deferred);
    CHECK_EQ(0u, h.forward_overruns + h.forward_dropped);
    CHECK_TRUE(!h.layout_mismatch);
    CHECK_EQ(lean_drop_expect, h.write_dropped);
    lean_drops_total += h.write_dropped;
    for (uint32_t i = 0u; (i < lean_seen_n) && (i < lean_expect_n); i++) {
        CHECK_EQ(lean_expect[i], lean_seen[i]); /* in order, no dropped frame written late */
    }
    if (m4_dead) {
        CHECK_TRUE(lean_seen_n <= lean_expect_n);
    } else {
        CHECK_EQ(lean_expect_n, lean_seen_n);
        CHECK_EQ(lean_expect_n, h.write_done);
    }

    /* The M7 accounts for every refused take; a dead M4 leaves its semaphore held. */
    CHECK_EQ(m7_read_refusals, h.read_busy);
    CHECK_EQ(0u, h.stale_slots & WARN_STALE_BIT);
    if (m4_dead) {
        CHECK_EQ((uint32_t)CORE_M4, (uint32_t)owner[WARN_HSEM]);
        const uint32_t far_ms = now + PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS + 1u;
        const uint32_t before = got_n;
        mailbox_step(far_ms);
        drain_bus();
        CHECK_EQ(before, got_n); /* the busy semaphore forwards nothing, torn or not */
        CHECK_EQ(0u, got_torn);
        CHECK_TRUE((mailbox_health(far_ms).stale_slots & WARN_STALE_BIT) != 0u);
    }
    rig_down();
    return ops;
}

/* ------------------------------------------------------------------------- tests */

void setUp(void) {}
void tearDown(void) {}

static void test_every_m4_action_after_every_m7_operation_keeps_the_mailbox_whole(void)
{
    static const uint32_t masks[] = {0x5u, 0xAu, 0x9u}; /* offers at least two passes apart */
    uint32_t runs = 0u;
    for (uint32_t m = 0u; m < (sizeof masks / sizeof masks[0]); m++) {
        const uint32_t total = scenario(ACT_NONE, -1, masks[m]);
        TEST_ASSERT_GREATER_THAN_UINT32(STEPS, total);
        for (uint32_t a = (uint32_t)ACT_M4_READ_LEAN; a < (uint32_t)ACT_COUNT; a++) {
            for (uint32_t k = 0u; k < total; k++) {
                (void)scenario((action_t)a, (int32_t)k, masks[m]);
                runs++;
            }
        }
    }
    printf("mailbox interleave: %u scenarios, %u EKF_LEAN frames dropped on a mismatch\n", (unsigned)runs,
           (unsigned)lean_drops_total);
    TEST_ASSERT_GREATER_THAN_UINT32(100u, runs);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, lean_drops_total); /* the wiped-word actions reach the drop path */
}

static void test_a_frame_read_while_the_m4_layout_is_wiped_is_never_forwarded(void)
{
    begin_run(false);
    observe_fn = observe_take;
    counting = true;
    uint32_t now = START_MS;

    /* pass 1: the M4 wipes its word and commits a frame before the pass starts */
    now++;
    m4_enter();
    region.word[MAILBOX_M4_LAYOUT_WORD] = 0u;
    m4_new_warn();
    m4_leave();
    step_peer_ok = false;
    mailbox_step(now);
    drain_bus();
    TEST_ASSERT_EQUAL_UINT32(0u, got_n);
    TEST_ASSERT_EQUAL_UINT32(1u, mailbox_health(now).read_rejected);

    /* pass 2: the word is back, no new frame: the absorbed one does not come later */
    now++;
    m4_enter();
    region.word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    m4_leave();
    step_peer_ok = true;
    mailbox_step(now);
    drain_bus();
    TEST_ASSERT_EQUAL_UINT32(0u, got_n);

    /* pass 3: a frame written after the restore is forwarded */
    now++;
    m4_enter();
    m4_new_warn();
    m4_leave();
    mailbox_step(now);
    drain_bus();
    TEST_ASSERT_EQUAL_UINT32(1u, got_n);
    TEST_ASSERT_EQUAL_UINT32(2u, got[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, got_torn);
    counting = false;
    rig_down();
}

static void test_a_wiped_m7_layout_word_is_restored_by_the_next_pass(void)
{
    begin_run(false);
    region.word[MAILBOX_M7_LAYOUT_WORD] = 0u;
    mailbox_step(START_MS + 1u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_MAILBOX_LAYOUT_ID, region.word[MAILBOX_M7_LAYOUT_WORD]);
    region.word[MAILBOX_M7_LAYOUT_WORD] = 0xDEADBEEFu;
    mailbox_step(START_MS + 2u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_MAILBOX_LAYOUT_ID, region.word[MAILBOX_M7_LAYOUT_WORD]);
    rig_down();
}

static void test_a_wiped_m4_layout_word_raises_the_flag_only_after_the_grace_time(void)
{
    const uint32_t grace = (PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS < PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS)
                               ? PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS
                               : PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS;
    begin_run(true); /* its baseline pass at START_MS already sees the word differ */
    mailbox_step(START_MS + 1u);
    TEST_ASSERT_FALSE(mailbox_health(START_MS + grace).layout_mismatch);
    TEST_ASSERT_TRUE(mailbox_health(START_MS + grace + 1u).layout_mismatch);
    region.word[MAILBOX_M4_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    mailbox_step(START_MS + 2u + grace);
    TEST_ASSERT_FALSE(mailbox_health(START_MS + 2u + grace).layout_mismatch);
    rig_down();
}

/* Phase-locked contention: the M4 holds EKF_LEAN exactly when the M7 tries to write, for
 * one to three consecutive passes per frame, a frame every 20 passes. */
static void test_a_peer_holding_the_slot_for_one_to_three_passes_loses_no_frame(void)
{
    enum { CYCLES = 12, CYCLE_STEPS = 20 };
    (void)snprintf(ctx, sizeof ctx, "phase-locked contention");
    begin_run(false);
    uint32_t now = START_MS;
    uint32_t held_total = 0u;
    for (uint32_t cycle = 0u; cycle < (uint32_t)CYCLES; cycle++) {
        const uint32_t hold = 1u + (cycle % 3u);
        for (uint32_t s = 0u; s < (uint32_t)CYCLE_STEPS; s++) {
            now++;
            if (s == 0u) {
                offer_lean();
            }
            const bool held = s < hold;
            if (held) {
                owner[LEAN_HSEM] = (uint8_t)CORE_M4; /* the M4 is inside its own read of the slot */
                held_total++;
            }
            mailbox_step(now);
            if (held) {
                owner[LEAN_HSEM] = (uint8_t)CORE_NONE;
            }
            m4_enter();
            m4_read_lean();
            m4_leave();
            const uint32_t expected_seen = (s < hold) ? cycle : (cycle + 1u);
            CHECK_EQ(expected_seen, lean_seen_n); /* deferred while held, then at once */
        }
    }
    const mailbox_health_t h = mailbox_health(now);
    CHECK_EQ((uint32_t)CYCLES, h.write_done);
    CHECK_EQ(held_total, h.write_deferred);
    CHECK_EQ(0u, h.write_overruns);
    CHECK_EQ((uint32_t)CYCLES, lean_seen_n);
    CHECK_EQ(0u, lean_torn);
    CHECK_EQ(0u, lean_not_increasing);
    CHECK_EQ(0u, lean_rejected);
    for (uint32_t i = 0u; i < lean_seen_n; i++) {
        CHECK_EQ(i + 1u, lean_seen[i]); /* consecutive: the receiver's E2E counter never jumps */
    }
    rig_down();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_every_m4_action_after_every_m7_operation_keeps_the_mailbox_whole);
    RUN_TEST(test_a_frame_read_while_the_m4_layout_is_wiped_is_never_forwarded);
    RUN_TEST(test_a_wiped_m7_layout_word_is_restored_by_the_next_pass);
    RUN_TEST(test_a_wiped_m4_layout_word_raises_the_flag_only_after_the_grace_time);
    RUN_TEST(test_a_peer_holding_the_slot_for_one_to_three_passes_loses_no_frame);
    return UNITY_END();
}
