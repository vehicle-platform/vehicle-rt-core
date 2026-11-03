#ifndef SERVICES_MAILBOX_CORE_H
#define SERVICES_MAILBOX_CORE_H

/*
 * Pure logic of the M7<->M4 mailbox (D-072 item 4c, D-093 items 3-6): the writer's
 * pending frame, the reader's baseline/sequence/length/layout decision and the
 * locked-section copies of one slot. No HAL, no RTOS, no lock: the caller (the glue,
 * services/mailbox.c, or a test's model of the M4) takes the slot's HSEM around the
 * locked-section helpers. Direction-agnostic: the same code serves either core.
 *
 * Region access (D-093 item 5): only 32-bit loads and stores through a volatile pointer
 * (SRAM4 ECC: no sub-word store, never memcpy). The generated platform_mailbox_t has byte
 * fields (len, data[8]), so the region is seen through mailbox_region_t, a union of the
 * generated layout and its 32-bit words (no pointer cast, MISRA 11.3); word indices come
 * from the generated offsets and are checked at compile time. Little-endian only (the
 * H7 and the hosts this builds on): data[0] is the low byte of the first data word.
 *
 * One slot = 4 words: write_seq, the len word (len in the low byte, reserved bytes
 * written as 0 and never read), data[0..3], data[4..7].
 *
 *   writer  mailbox_pending_offer()        any time (no lock): keep the newest frame
 *           take HSEM -> mailbox_slot_store() -> barrier -> mailbox_slot_commit() -> release
 *           not taken  -> mailbox_pending_defer()   (kept for the next pass)
 *   reader  take HSEM -> mailbox_slot_load() -> release -> mailbox_reader_accept()
 *           not taken  -> mailbox_reader_busy()     ("no new frame" this pass)
 */

#include "platform_mailbox.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAILBOX_WORD_BYTES 4u
#define MAILBOX_REGION_WORDS (PLATFORM_MAILBOX_USED_BYTES / MAILBOX_WORD_BYTES)
#define MAILBOX_SLOT_WORDS (PLATFORM_MAILBOX_SLOT_BYTES / MAILBOX_WORD_BYTES)
#define MAILBOX_PAYLOAD_BYTES 8u
#define MAILBOX_PAYLOAD_WORDS (MAILBOX_PAYLOAD_BYTES / MAILBOX_WORD_BYTES)

/* Word indices inside the region and inside one slot, from the generated offsets. */
#define MAILBOX_M7_LAYOUT_WORD (offsetof(platform_mailbox_header_t, m7_layout_id) / MAILBOX_WORD_BYTES)
#define MAILBOX_M4_LAYOUT_WORD (offsetof(platform_mailbox_header_t, m4_layout_id) / MAILBOX_WORD_BYTES)
#define MAILBOX_SEQ_WORD (offsetof(platform_mailbox_slot_t, write_seq) / MAILBOX_WORD_BYTES)
#define MAILBOX_LEN_WORD (offsetof(platform_mailbox_slot_t, len) / MAILBOX_WORD_BYTES)
#define MAILBOX_DATA_WORD (offsetof(platform_mailbox_slot_t, data) / MAILBOX_WORD_BYTES)

/* The region seen as the generated layout or as its 32-bit words. Access only .word[]. */
typedef union {
    platform_mailbox_t layout;
    uint32_t word[MAILBOX_REGION_WORDS];
} mailbox_region_t;

/* A frame waiting for its slot (writer side) or for the bus (the M7's forward). */
typedef struct {
    uint32_t data[MAILBOX_PAYLOAD_WORDS]; /* payload words, bytes past len are 0 */
    uint8_t len;
    bool pending;
    uint32_t done;     /* frames written / forwarded (saturating) */
    uint32_t deferred; /* passes the frame had to wait (HSEM or TX busy) */
    uint32_t overruns; /* a pending frame replaced by a newer one: lost (D-093 item 5 as narrowed) */
    uint32_t dropped;  /* frames given up (the forward's bus-off or latched port) */
} mailbox_pending_t;

/* A slot's raw content as read under its HSEM. */
typedef struct {
    uint32_t seq;
    uint32_t len_word;
    uint32_t data[MAILBOX_PAYLOAD_WORDS];
} mailbox_raw_t;

typedef enum {
    MAILBOX_READ_NONE = 0,   /* write_seq unchanged (or the HSEM was busy) */
    MAILBOX_READ_BASELINE,   /* first read: write_seq taken as the baseline, no frame */
    MAILBOX_READ_NEW,        /* a new, complete frame */
    MAILBOX_READ_BAD_LAYOUT, /* new write_seq while the peer's layout word differs: dropped */
    MAILBOX_READ_BAD_LEN,    /* new write_seq with len != the compile-time length: dropped */
} mailbox_read_t;

typedef struct {
    uint8_t expected_len; /* the gen/ DBC length of the slot's message */
    bool has_baseline;
    uint32_t last_seq;
    uint32_t fresh_at_ms; /* baseline or last NEW frame */
    uint32_t frames;      /* NEW frames (saturating, like the counters below) */
    uint32_t busy;        /* passes with the HSEM not taken */
    uint32_t bad_layout;
    uint32_t bad_len;
} mailbox_reader_t;

/* ------------------------------------------------------------------- writer side */

void mailbox_pending_init(mailbox_pending_t* p);

/* Keeps data[0..len) (len 1..8) as the frame to write next. A frame still pending is
 * replaced and counted as an overrun. False (nothing changed) for NULL or a bad len. */
bool mailbox_pending_offer(mailbox_pending_t* p, const uint8_t* data, uint8_t len);

/* The frame could not go this pass (HSEM or bus busy): it stays pending. */
void mailbox_pending_defer(mailbox_pending_t* p);

/* The frame went (slot committed, or the forward finished): no longer pending. */
void mailbox_pending_done(mailbox_pending_t* p);

/* The frame was given up (counted as dropped): no longer pending. */
void mailbox_pending_drop(mailbox_pending_t* p);

/* Locked section, writer: payload words and the len word into the slot (4 words at
 * slot[0..3]); write_seq untouched. */
void mailbox_slot_store(const mailbox_pending_t* p, volatile uint32_t* slot);

/* Locked section, writer, after the barrier: write_seq + 1 read-modify-write on the
 * region (a value never repeats across a writer reset; wrap is harmless). */
void mailbox_slot_commit(volatile uint32_t* slot);

/* ------------------------------------------------------------------- reader side */

void mailbox_reader_init(mailbox_reader_t* r, uint8_t expected_len);

/* Locked section, reader: the slot's four words. */
void mailbox_slot_load(const volatile uint32_t* slot, mailbox_raw_t* raw);

/* Decision on a slot read under its HSEM (D-093 item 5): the first read only takes the
 * baseline (a frame from before a reset is never used); afterwards a write_seq different
 * from the last one (inequality only) is a new frame, unless the peer's layout word
 * differs or len is not the compile-time length; such a frame is dropped and its
 * write_seq absorbed, so it is never delivered later. The layout check is made at read
 * time: an image whose word differs never delivers a frame, but a frame committed during
 * a transient wipe of a matching peer's word (the owner rewrites it every pass) and read
 * after the rewrite is delivered; the receiver's E2E check still applies to it. */
mailbox_read_t mailbox_reader_accept(mailbox_reader_t* r, const mailbox_raw_t* raw,
                                     bool peer_layout_ok, uint32_t now_ms);

/* The HSEM was not taken: no new frame this pass. */
void mailbox_reader_busy(mailbox_reader_t* r);

/* True when no baseline exists yet or no NEW frame arrived for longer than timeout_ms
 * since the baseline or the last frame (a stuck write_seq, a dead or mismatched peer). */
bool mailbox_reader_stale(const mailbox_reader_t* r, uint32_t now_ms, uint32_t timeout_ms);

/* --------------------------------------------------------------------- payloads */

/* data[0..8) from the payload words (little-endian). */
void mailbox_payload_bytes(const uint32_t words[MAILBOX_PAYLOAD_WORDS], uint8_t data[MAILBOX_PAYLOAD_BYTES]);

/* The len byte of a slot's len word. */
uint8_t mailbox_len_of(uint32_t len_word);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_MAILBOX_CORE_H */
