#include "services/mailbox.h"

#include "hal/hsem.h"
#include "platform.h"
#include "platform_e2e.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/diag.h"

#include <stddef.h>
#include <string.h>

/* D-093 item 3: a bike/release image must not run on an unverified region address. The
 * image build (red zone, with the board) defines VEHICLE_RTCORE_BIKE_IMAGE; the host and
 * the logic cross builds do not (safety-reviewer MINOR-2). */
#if defined(VEHICLE_RTCORE_BIKE_IMAGE) && (PLATFORM_MAILBOX_ADDR_VERIFIED == 0)
#error "PLATFORM_MAILBOX_ADDR_VERIFIED is 0: the SRAM4 mailbox address is not confirmed on the board"
#endif

/* One row per generated slot, in slot order (D-093 item 1: ID, length, HSEM, direction
 * from the header; the supervision timeout is the slot message's gen/ E2E timeout). */
typedef struct {
    uint32_t frame_id;
    bool extended;
    uint8_t length;
    uint8_t hsem_id;
    uint8_t direction;
    uint32_t offset;     /* bytes from the region base */
    uint32_t timeout_ms; /* E2E timeout of the slot message */
    uint32_t cycle_ms;   /* cycle time of the slot message (the M7->M4 age limit) */
} mailbox_slot_def_t;

static const mailbox_slot_def_t slot_defs[PLATFORM_MAILBOX_SLOT_COUNT] = {
    {PLATFORM_MAILBOX_SLOT_EKF_LEAN_FRAME_ID, PLATFORM_EKF_LEAN_IS_EXTENDED != 0,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH, (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_DIRECTION, PLATFORM_MAILBOX_SLOT_EKF_LEAN_OFFSET,
     PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS, PLATFORM_EKF_LEAN_CYCLE_TIME_MS},
    {PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_FRAME_ID, PLATFORM_EKF_FRICTION_MASS_IS_EXTENDED != 0,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_DIRECTION,
     PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_OFFSET,
     PLATFORM_EKF_FRICTION_MASS_E2E_TIMEOUT_MS, PLATFORM_EKF_FRICTION_MASS_CYCLE_TIME_MS},
    {PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_FRAME_ID, PLATFORM_HEARTBEAT_RT_CORE_IS_EXTENDED != 0,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_DIRECTION,
     PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_OFFSET,
     PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS, PLATFORM_HEARTBEAT_RT_CORE_CYCLE_TIME_MS},
    {PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_FRAME_ID, PLATFORM_SAFETY_REQUEST_IS_EXTENDED != 0,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_DIRECTION,
     PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_OFFSET,
     PLATFORM_SAFETY_REQUEST_E2E_TIMEOUT_MS, PLATFORM_SAFETY_REQUEST_CYCLE_TIME_MS},
    {PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_FRAME_ID, PLATFORM_SAFETY_WARNING_STATUS_IS_EXTENDED != 0,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_DIRECTION,
     PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_OFFSET,
     PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS, PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS},
    {PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_FRAME_ID, PLATFORM_HEARTBEAT_SAFETY_IS_EXTENDED != 0,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID,
     (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_DIRECTION,
     PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_OFFSET,
     PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS, PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS},
};

/* The rows above are in slot order: a reordered or extended gen/ table fails here. */
_Static_assert(PLATFORM_MAILBOX_SLOT_COUNT == 6u, "one row per generated slot");
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX == 0u, "slot order");
_Static_assert(PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_INDEX == 1u, "slot order");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_INDEX == 2u, "slot order");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_INDEX == 3u, "slot order");
_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX == 4u, "slot order");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX == 5u, "slot order");
_Static_assert(PLATFORM_MAILBOX_SLOT_COUNT <= 32u, "stale_slots is a 32-bit mask");
/* A dropped or replaced M7->M4 frame must never read as valid on the M4 (OK_SOME_LOST):
 * the bounded-loss rules of D-093 amendments 1 and 2 need E2E max delta 1 on every slot
 * message (safety-reviewer MINOR-1, 2026-11-04). */
_Static_assert(PLATFORM_EKF_LEAN_E2E_MAX_DELTA_COUNTER == 1u, "D-093 amendments 1-2 need max delta 1");
_Static_assert(PLATFORM_EKF_FRICTION_MASS_E2E_MAX_DELTA_COUNTER == 1u, "D-093 amendments 1-2 need max delta 1");
_Static_assert(PLATFORM_HEARTBEAT_RT_CORE_E2E_MAX_DELTA_COUNTER == 1u, "D-093 amendments 1-2 need max delta 1");
_Static_assert(PLATFORM_SAFETY_REQUEST_E2E_MAX_DELTA_COUNTER == 1u, "D-093 amendments 1-2 need max delta 1");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_E2E_MAX_DELTA_COUNTER == 1u, "D-093 amendments 1-2 need max delta 1");
_Static_assert(PLATFORM_HEARTBEAT_SAFETY_E2E_MAX_DELTA_COUNTER == 1u, "D-093 amendments 1-2 need max delta 1");
/* Mailbox HSEM IDs stay in 1..31 (ID 0 is vendor boot code's, D-093 item 5). */
_Static_assert((PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID >= HAL_HSEM_ID_MIN) &&
                   (PLATFORM_MAILBOX_SLOT_EKF_LEAN_HSEM_ID <= HAL_HSEM_ID_MAX), "HSEM ID range");
_Static_assert((PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID >= HAL_HSEM_ID_MIN) &&
                   (PLATFORM_MAILBOX_SLOT_EKF_FRICTION_MASS_HSEM_ID <= HAL_HSEM_ID_MAX), "HSEM ID range");
_Static_assert((PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID >= HAL_HSEM_ID_MIN) &&
                   (PLATFORM_MAILBOX_SLOT_HEARTBEAT_RT_CORE_HSEM_ID <= HAL_HSEM_ID_MAX), "HSEM ID range");
_Static_assert((PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID >= HAL_HSEM_ID_MIN) &&
                   (PLATFORM_MAILBOX_SLOT_SAFETY_REQUEST_HSEM_ID <= HAL_HSEM_ID_MAX), "HSEM ID range");
_Static_assert((PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID >= HAL_HSEM_ID_MIN) &&
                   (PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_HSEM_ID <= HAL_HSEM_ID_MAX), "HSEM ID range");
_Static_assert((PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID >= HAL_HSEM_ID_MIN) &&
                   (PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_HSEM_ID <= HAL_HSEM_ID_MAX), "HSEM ID range");

static volatile mailbox_region_t* mb_region; /* NULL: inert */
static mailbox_pending_t mb_pending[PLATFORM_MAILBOX_SLOT_COUNT]; /* M7->M4 writes, M4->M7 forwards */
static mailbox_reader_t mb_reader[PLATFORM_MAILBOX_SLOT_COUNT];   /* M4->M7 slots */
static bool mb_mismatch;            /* the M4 layout word differed at the last step */
static uint32_t mb_mismatch_since_ms;
static uint32_t mb_grace_ms;        /* the shortest M4->M7 E2E timeout */
/* DTC state (D-093 amendment 3 + addendum). The flags are latched in mailbox_step() only
 * and sticky against the ms wrap; mailbox_health() reads them and changes nothing. */
static bool mb_matched_once;        /* the M4 layout word matched at a step since init */
static uint32_t mb_init_ms;
static bool mb_boot_budget_over;    /* MAILBOX_PEER_BOOT_BUDGET_MS passed since init */
static bool mb_mismatch_over_grace; /* the current mismatch run outlasted mb_grace_ms */
static uint32_t mb_match_since_ms;  /* start of the current matched run */
static bool mb_armed[PLATFORM_MAILBOX_SLOT_COUNT];    /* matched run > the slot's timeout */
static bool mb_seen_new[PLATFORM_MAILBOX_SLOT_COUNT]; /* a new M4 frame read since init */
static uint32_t mb_frozen_held;     /* the last reported frozen mask (short mismatch) */
static uint32_t mb_skipped;

static uint32_t mb_sat_add(uint32_t a, uint32_t b)
{
    return (b > (UINT32_MAX - a)) ? UINT32_MAX : (a + b);
}

static bool m7_writes(uint32_t i)
{
    return slot_defs[i].direction == (uint8_t)PLATFORM_MAILBOX_DIR_M7_TO_M4;
}

static bool m7_reads(uint32_t i)
{
    return slot_defs[i].direction == (uint8_t)PLATFORM_MAILBOX_DIR_M4_TO_M7;
}

static uint32_t slot_word(uint32_t i)
{
    return slot_defs[i].offset / MAILBOX_WORD_BYTES;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the H7 setup (with the board) and the tests
bool mailbox_init(volatile mailbox_region_t* region, uint32_t now_ms)
{
    mb_region = NULL;
    mb_mismatch = false;
    mb_mismatch_since_ms = now_ms;
    mb_matched_once = false;
    mb_init_ms = now_ms;
    mb_boot_budget_over = false;
    mb_mismatch_over_grace = false;
    mb_match_since_ms = now_ms;
    mb_frozen_held = 0u;
    mb_skipped = 0u;
    mb_grace_ms = UINT32_MAX;
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        mailbox_pending_init(&mb_pending[i]);
        mailbox_reader_init(&mb_reader[i], slot_defs[i].length);
        mb_armed[i] = false;
        mb_seen_new[i] = false;
        if (m7_reads(i) && (slot_defs[i].timeout_ms < mb_grace_ms)) {
            mb_grace_ms = slot_defs[i].timeout_ms;
        }
    }
    if (region == NULL) {
        return false;
    }
    /* The forwarded M4 frames are safety-range / heartbeat frames: each gets its own
     * replace-on-new TX buffer, never the Tx FIFO behind 0x110 or ISO-TP (D-054 item 6,
     * D-056 item 6; safety-reviewer MAJOR-1). Boot order: before comms_apply_filters(); a
     * repeated init before it finds them listed (CAN_IF_ERR_DUP), after it is refused. */
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        if (m7_reads(i)) {
            const can_if_status_t st =
                can_if_register_tx_dedicated(CAN_PORT_PLATFORM, slot_defs[i].frame_id, slot_defs[i].extended);
            if ((st != CAN_IF_OK) && (st != CAN_IF_ERR_DUP)) {
                /* sealed or full: stay inert, the M4 frames never use the FIFO. An ID listed
                 * before the failure keeps its buffer, unused (safety-reviewer NIT-5). */
                return false;
            }
        }
    }
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        hal_hsem_release_own(slot_defs[i].hsem_id); /* a lock left from before a reset */
    }
    region->word[MAILBOX_M7_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    mb_region = region;
    return true;
}

bool mailbox_offer(uint32_t frame_id, bool extended, const uint8_t* data, uint8_t dlc)
{
    if ((mb_region == NULL) || (data == NULL)) {
        return false;
    }
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        if (m7_writes(i) && (slot_defs[i].frame_id == frame_id) && (slot_defs[i].extended == extended)) {
            if (dlc != slot_defs[i].length) {
                return false;
            }
            return mailbox_pending_offer(&mb_pending[i], data, dlc);
        }
    }
    return false;
}

/* M7->M4: the pending frame under the slot's HSEM, or deferred to the next pass. While
 * the M4 layout word differs, no slot of that peer is written (an image built from other
 * defs may place its slots differently) and the pending frame is dropped, never kept: a
 * frame held through a mismatch shorter than its E2E timeout would reach the M4 late with
 * a counter step of 1 and read as OK with a stale payload (D-093 amendment 2, ISSUES E-28
 * (5), user 2026-11-04). The drop relies on max delta 1 (asserted above): the M4 then
 * reads the gap as WRONG_SEQUENCE, or INITIAL after its timeout, never as valid. A one-pass
 * wipe of a matching M4's word also drops that pass's frames (fails towards UNAVAILABLE). */
static void write_slot(uint32_t i, bool peer_ok, uint32_t now_ms)
{
    if (!mb_pending[i].pending) {
        return;
    }
    if (!peer_ok) {
        mailbox_pending_drop(&mb_pending[i]);
        return;
    }
    /* Same principle for a frame the HSEM kept waiting: strictly older than its cycle at
     * this attempt means drop, never hold (D-093 amendment 2 principle, E-28 follow-up). The M4 then sees a gap. */
    mailbox_pending_stamp(&mb_pending[i], now_ms);
    if (mailbox_pending_aged(&mb_pending[i], now_ms, slot_defs[i].cycle_ms)) {
        mailbox_pending_drop_aged(&mb_pending[i]);
        return;
    }
    hal_hsem_key_t key;
    if (hal_hsem_take(slot_defs[i].hsem_id, &key)) {
        volatile uint32_t* const slot = &mb_region->word[slot_word(i)];
        mailbox_slot_store(&mb_pending[i], slot);
        hal_hsem_barrier(); /* payload before write_seq (D-093 item 5) */
        mailbox_slot_commit(slot);
        hal_hsem_release(slot_defs[i].hsem_id, &key);
        mailbox_pending_done(&mb_pending[i]);
    } else {
        mailbox_pending_defer(&mb_pending[i]);
    }
}

/* M4->M7: read under the slot's HSEM; a new frame becomes the slot's pending forward. */
static void read_slot(uint32_t i, bool peer_ok, uint32_t now_ms)
{
    hal_hsem_key_t key;
    mailbox_raw_t raw;
    if (!hal_hsem_take(slot_defs[i].hsem_id, &key)) {
        mailbox_reader_busy(&mb_reader[i]);
        return;
    }
    mailbox_slot_load(&mb_region->word[slot_word(i)], &raw);
    hal_hsem_release(slot_defs[i].hsem_id, &key);
    if (mailbox_reader_accept(&mb_reader[i], &raw, peer_ok, now_ms) == MAILBOX_READ_NEW) {
        mb_seen_new[i] = true;
        uint8_t data[MAILBOX_PAYLOAD_BYTES];
        mailbox_payload_bytes(raw.data, data);
        (void)mailbox_pending_offer(&mb_pending[i], data, slot_defs[i].length);
    }
}

/* The pending forward to the platform bus, verbatim (never re-signed, D-093 item 5). */
static void forward_slot(uint32_t i)
{
    if (!mb_pending[i].pending) {
        return;
    }
    can_frame_t frame;
    (void)memset(&frame, 0, sizeof frame);
    frame.id = slot_defs[i].frame_id;
    frame.extended = slot_defs[i].extended;
    frame.dlc = slot_defs[i].length;
    uint8_t data[MAILBOX_PAYLOAD_BYTES];
    mailbox_payload_bytes(mb_pending[i].data, data);
    (void)memcpy(frame.data, data, slot_defs[i].length);
    const can_port_status_t st = can_if_write(CAN_PORT_PLATFORM, &frame);
    if (st == CAN_PORT_OK) {
        mailbox_pending_done(&mb_pending[i]);
    } else if (st == CAN_PORT_TX_FULL) {
        mailbox_pending_defer(&mb_pending[i]); /* again next pass */
    } else {
        mailbox_pending_drop(&mb_pending[i]); /* bus-off or latched: given up */
    }
}

static bool mismatch_past_grace(uint32_t now_ms)
{
    return mb_mismatch && ((uint32_t)(now_ms - mb_mismatch_since_ms) > mb_grace_ms);
}

/* Once per initialised step, after the slots were read: the sticky latches. */
static void update_latches(uint32_t now_ms)
{
    if (!mb_boot_budget_over && ((uint32_t)(now_ms - mb_init_ms) > MAILBOX_PEER_BOOT_BUDGET_MS)) {
        mb_boot_budget_over = true;
    }
    if (mismatch_past_grace(now_ms)) {
        mb_mismatch_over_grace = true;
    }
    if (mb_matched_once && !mb_mismatch) {
        const uint32_t matched_ms = (uint32_t)(now_ms - mb_match_since_ms);
        for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
            if (m7_reads(i) && (matched_ms > slot_defs[i].timeout_ms)) {
                mb_armed[i] = true;
            }
        }
    }
}

/* The DTC levels, one definition for mailbox_health() and the report; reads the latches.
 * - Mismatch: before the first match the M4 is booting, not mismatched, so only the boot
 *   budget arms it (peer missing or other defs); after a match, a mismatch past the grace.
 * - Frozen: only while the layouts match; a slot counts once the matched run outlasted its
 *   E2E timeout (armed) and it delivered a new frame since init, or the boot budget is over
 *   (after a warm reset SRAM4 still holds a matching M4 word while the M4 initialises).
 *   During a mismatch still inside the grace the last frozen level holds, so a one-pass
 *   wipe opens no passed gap. */
static void dtc_levels(uint32_t now_ms, bool* mismatch_dtc, uint32_t* frozen)
{
    *mismatch_dtc = mb_matched_once ? mb_mismatch_over_grace : mb_boot_budget_over;
    *frozen = 0u;
    if (mb_matched_once && !mb_mismatch) {
        for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
            if (m7_reads(i) && mb_armed[i] && (mb_seen_new[i] || mb_boot_budget_over) &&
                mailbox_reader_stale(&mb_reader[i], now_ms, slot_defs[i].timeout_ms)) {
                *frozen |= ((uint32_t)1u << i);
            }
        }
    } else if (mb_matched_once && !mb_mismatch_over_grace) {
        *frozen = mb_frozen_held;
    } else {
        /* never matched, or a mismatch past the grace: the mismatch DTC covers it */
    }
}

/* Level-triggered every initialised pass; nothing while inert (the monitors do not run).
 * Diagnostic only: neither DTC sets the heartbeat's DEGRADED or ERROR_COUNT (diag.c). */
static void report_dtcs(uint32_t now_ms)
{
    bool mismatch_dtc;
    uint32_t frozen;
    update_latches(now_ms);
    dtc_levels(now_ms, &mismatch_dtc, &frozen);
    mb_frozen_held = frozen;
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_SAFETY_MAILBOX_LAYOUT_MISMATCH, mismatch_dtc);
    diag_dtc_report(PLATFORM_UDS_DTC_IDX_SAFETY_MAILBOX_SLOT_FROZEN, !mismatch_dtc && (frozen != 0u));
}

void mailbox_step(uint32_t now_ms)
{
    if (mb_region == NULL) {
        if (mb_skipped < UINT32_MAX) {
            mb_skipped++;
        }
        return;
    }
    /* Every pass, so a wiped word recovers within one pass (D-093 item 5). */
    mb_region->word[MAILBOX_M7_LAYOUT_WORD] = PLATFORM_MAILBOX_LAYOUT_ID;
    const bool peer_ok = mb_region->word[MAILBOX_M4_LAYOUT_WORD] == PLATFORM_MAILBOX_LAYOUT_ID;
    if (peer_ok) {
        if (!mb_matched_once || mb_mismatch_over_grace) {
            /* a new matched run starts; a mismatch inside the grace does not restart it */
            mb_match_since_ms = now_ms;
            for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
                mb_armed[i] = false;
            }
        }
        mb_matched_once = true;
        mb_mismatch = false;
        mb_mismatch_over_grace = false;
    } else if (!mb_mismatch) {
        mb_mismatch = true;
        mb_mismatch_since_ms = now_ms;
    } else {
        /* still mismatched since mb_mismatch_since_ms */
    }
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        if (m7_writes(i)) {
            write_slot(i, peer_ok, now_ms);
        } else if (m7_reads(i)) {
            read_slot(i, peer_ok, now_ms);
            forward_slot(i);
        } else {
            /* no other direction exists (gen/ check) */
        }
    }
    report_dtcs(now_ms);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the tests and the future health DID
mailbox_health_t mailbox_health(uint32_t now_ms)
{
    mailbox_health_t h;
    (void)memset(&h, 0, sizeof h);
    h.skipped_steps = mb_skipped;
    if (mb_region == NULL) {
        return h;
    }
    h.initialised = true;
    h.layout_mismatch = mismatch_past_grace(now_ms);
    dtc_levels(now_ms, &h.dtc_layout_mismatch, &h.frozen_slots);
    h.peer_missing = !mb_matched_once && mb_boot_budget_over;
    h.dtc_slot_frozen = !h.dtc_layout_mismatch && (h.frozen_slots != 0u);
    for (uint32_t i = 0u; i < PLATFORM_MAILBOX_SLOT_COUNT; i++) {
        const mailbox_pending_t* const p = &mb_pending[i];
        if (m7_writes(i)) {
            h.write_done = mb_sat_add(h.write_done, p->done);
            h.write_deferred = mb_sat_add(h.write_deferred, p->deferred);
            h.write_overruns = mb_sat_add(h.write_overruns, p->overruns);
            h.write_dropped = mb_sat_add(h.write_dropped, p->dropped);
            h.write_aged = mb_sat_add(h.write_aged, p->aged);
        } else if (m7_reads(i)) {
            const mailbox_reader_t* const r = &mb_reader[i];
            if (mailbox_reader_stale(r, now_ms, slot_defs[i].timeout_ms)) {
                h.stale_slots |= ((uint32_t)1u << i);
            }
            h.read_busy = mb_sat_add(h.read_busy, r->busy);
            h.read_rejected = mb_sat_add(h.read_rejected, mb_sat_add(r->bad_layout, r->bad_len));
            h.forward_done = mb_sat_add(h.forward_done, p->done);
            h.forward_deferred = mb_sat_add(h.forward_deferred, p->deferred);
            h.forward_overruns = mb_sat_add(h.forward_overruns, p->overruns);
            h.forward_dropped = mb_sat_add(h.forward_dropped, p->dropped);
        } else {
            /* no other direction exists */
        }
    }
    return h;
}
