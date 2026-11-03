#ifndef SERVICES_MAILBOX_H
#define SERVICES_MAILBOX_H

/*
 * The M7 side of the M7<->M4 mailbox (D-072 item 4c, D-093 items 3-6). Ownership comes
 * from the generated slot table (gen/ platform_mailbox.h): the M7 writes its layout word
 * and the DIR_M7_TO_M4 slots, and reads the DIR_M4_TO_M7 slots, which it forwards
 * verbatim to the platform bus (FDCAN2), never re-signed.
 *
 *   mailbox_offer()   services/com.c only (com_send(), after CAN_PORT_OK): the frame whose
 *                     E2E counter the sender just committed (D-056 item 5) becomes the
 *                     slot's pending frame. No lock, no region access.
 *   mailbox_step()    app/comms.c only, once per comms pass after 0x081/0x020: rewrites the
 *                     M7 layout word, writes each pending M7->M4 frame under its HSEM or
 *                     defers it to the next pass (no wait, no retry loop), reads each
 *                     M4->M7 slot under its HSEM and forwards a new frame.
 *
 * Both run in the comms task: one accessing context per slot on the M7 (D-093 item 5),
 * so the slot state here needs no lock. CI greps the two call sites.
 *
 * Bounded loss (user, 2026-11-03, narrows D-093 item 5 "never dropped"): a frame still
 * pending when the next one of its slot arrives is replaced and counted as an overrun;
 * the receiver's E2E check (max delta 1) reports the gap. With a 1 ms pass this needs the
 * HSEM refused for a whole message cycle (>= 20 passes), which is a fault. Likewise a
 * forward replaced before the bus took it.
 *
 * Health (user, 2026-11-03: flags now, the DTC with a later defs change): the M4's layout
 * word differing from PLATFORM_MAILBOX_LAYOUT_ID for longer than the shortest M4->M7 E2E
 * timeout, and per M4->M7 slot no new frame for longer than its gen/ E2E timeout (a stuck
 * write_seq, a dead or mismatched M4). Level flags, recomputed at each call.
 *
 * Inert until mailbox_init(): before it, offer and step do nothing (step counts the
 * skipped passes). The H7 setup passes the SRAM4 region (PLATFORM_MAILBOX_REGION_ADDR,
 * verified: false) once the image, its linker section and MPU region exist (red zone,
 * not in this module). Static storage, no heap.
 */

#include "services/mailbox_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool initialised;
    bool layout_mismatch;      /* the M4 layout word differs for longer than the grace time */
    uint32_t stale_slots;      /* bit i: M4->M7 slot i had no new frame within its E2E timeout */
    uint32_t write_done;       /* M7->M4 frames committed */
    uint32_t write_deferred;   /* passes a pending M7->M4 frame waited for its HSEM */
    uint32_t write_overruns;   /* M7->M4 frames replaced before they were written */
    uint32_t read_busy;        /* M4->M7 reads with the HSEM not taken */
    uint32_t read_rejected;    /* M4->M7 frames dropped: layout mismatch or bad len */
    uint32_t forward_done;     /* M4->M7 frames the platform port accepted */
    uint32_t forward_deferred; /* forward passes refused with CAN_PORT_TX_FULL */
    uint32_t forward_overruns; /* forwards replaced before the bus took them */
    uint32_t forward_dropped;  /* forwards given up (bus-off, latched port) */
    uint32_t skipped_steps;    /* mailbox_step() calls before mailbox_init() */
} mailbox_health_t;

/* Boot only, before the comms task runs and BEFORE comms_apply_filters(): registers the
 * M4->M7 frame IDs for dedicated replace-on-new TX buffers on the platform port (D-054
 * item 6; refused once the port is sealed or its list is full: then false and inert, so
 * a forwarded M4 frame never goes through the Tx FIFO). Frees this core's mailbox HSEMs (one per slot,
 * never ID 0) and writes the M7 layout word; touches no other word of the region (never
 * clears it, D-093 item 5). The readers take their baseline at their first read. Every
 * call starts over (pending frames, baselines, counters), like the other services' init;
 * NULL makes the module inert and returns false. */
bool mailbox_init(volatile mailbox_region_t* region, uint32_t now_ms);

/* services/com.c only. True if frame_id is a standard M7->M4 slot message of exactly
 * its gen/ length (dlc) and the module is initialised: the frame is now that slot's
 * pending frame (a pending one is replaced: overrun). False otherwise (nothing kept). */
bool mailbox_offer(uint32_t frame_id, bool extended, const uint8_t* data, uint8_t dlc);

/* app/comms.c only, once per comms pass. */
void mailbox_step(uint32_t now_ms);

/* Comms task: the flags and counters at now_ms. */
mailbox_health_t mailbox_health(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_MAILBOX_H */
