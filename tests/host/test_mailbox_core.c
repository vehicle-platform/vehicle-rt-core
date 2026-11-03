/*
 * L0 tests of services/mailbox_core (D-093 items 4-6): the pending frame (offer, replace
 * = overrun, defer, done, drop), the slot's word layout against the generated
 * platform_mailbox_t, write_seq read-modify-write and its wrap, and the reader decision
 * (baseline, unchanged write_seq, new frame, wrap, len mismatch, layout mismatch,
 * absorbed sequences, stale/stuck sequence, torn write = payload without commit).
 */
#include "services/mailbox_core.h"

#include "platform.h"
#include "platform_e2e.h"

#include <string.h>
#include <unity.h>

#define LEN ((uint8_t)PLATFORM_MAILBOX_SLOT_EKF_LEAN_LENGTH)
#define SLOT_IDX PLATFORM_MAILBOX_SLOT_EKF_LEAN_INDEX
#define SLOT_WORD (PLATFORM_MAILBOX_SLOT_EKF_LEAN_OFFSET / MAILBOX_WORD_BYTES)
#define TIMEOUT_MS PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS

static mailbox_region_t region;
static mailbox_pending_t pend;
static mailbox_reader_t rd;

static volatile uint32_t* slot(void)
{
    return &region.word[SLOT_WORD];
}

static void frame(uint8_t k, uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    (void)memset(out, 0, MAILBOX_PAYLOAD_BYTES);
    for (uint8_t j = 0u; j < LEN; j++) {
        out[j] = (uint8_t)(k + (uint8_t)(j * 31u));
    }
}

/* The writer's locked section, as the glue runs it. */
static void write_frame(uint8_t k)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(k, d);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, LEN));
    mailbox_slot_store(&pend, slot());
    mailbox_slot_commit(slot());
    mailbox_pending_done(&pend);
}

static mailbox_read_t read_frame(bool layout_ok, uint32_t now, uint8_t out[MAILBOX_PAYLOAD_BYTES])
{
    mailbox_raw_t raw;
    mailbox_slot_load(slot(), &raw);
    const mailbox_read_t r = mailbox_reader_accept(&rd, &raw, layout_ok, now);
    mailbox_payload_bytes(raw.data, out);
    return r;
}

void setUp(void)
{
    (void)memset(&region, 0, sizeof region);
    mailbox_pending_init(&pend);
    mailbox_reader_init(&rd, LEN);
}

void tearDown(void) {}

static void test_word_indices_match_the_generated_layout(void)
{
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_MAILBOX_USED_BYTES / 4u, MAILBOX_REGION_WORDS);
    region.word[MAILBOX_M7_LAYOUT_WORD] = 0x11111111u;
    region.word[MAILBOX_M4_LAYOUT_WORD] = 0x22222222u;
    TEST_ASSERT_EQUAL_HEX32(0x11111111u, region.layout.header.m7_layout_id);
    TEST_ASSERT_EQUAL_HEX32(0x22222222u, region.layout.header.m4_layout_id);
}

static void test_store_and_commit_fill_the_generated_slot_fields(void)
{
    region.layout.slot[SLOT_IDX].reserved[0] = 0xAAu; /* written as 0 by the owner */
    write_frame(5u);
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(5u, d);
    const platform_mailbox_slot_t* s = &region.layout.slot[SLOT_IDX];
    TEST_ASSERT_EQUAL_UINT32(1u, s->write_seq);
    TEST_ASSERT_EQUAL_UINT8(LEN, s->len);
    TEST_ASSERT_EQUAL_UINT8(0u, s->reserved[0]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(d, s->data, MAILBOX_PAYLOAD_BYTES);
    /* nothing else of the region was touched */
    TEST_ASSERT_EQUAL_UINT32(0u, region.layout.slot[SLOT_IDX + 1u].write_seq);
    TEST_ASSERT_EQUAL_UINT32(0u, region.layout.header.m7_layout_id);
}

static void test_commit_is_a_read_modify_write_that_wraps(void)
{
    region.layout.slot[SLOT_IDX].write_seq = UINT32_MAX;
    mailbox_slot_commit(slot());
    TEST_ASSERT_EQUAL_UINT32(0u, region.layout.slot[SLOT_IDX].write_seq);
    mailbox_slot_commit(slot());
    TEST_ASSERT_EQUAL_UINT32(1u, region.layout.slot[SLOT_IDX].write_seq);
}

static void test_offer_rejects_bad_arguments(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES] = {0};
    TEST_ASSERT_FALSE(mailbox_pending_offer(NULL, d, LEN));
    TEST_ASSERT_FALSE(mailbox_pending_offer(&pend, NULL, LEN));
    TEST_ASSERT_FALSE(mailbox_pending_offer(&pend, d, 0u));
    TEST_ASSERT_FALSE(mailbox_pending_offer(&pend, d, MAILBOX_PAYLOAD_BYTES + 1u));
    TEST_ASSERT_FALSE(pend.pending);
}

static void test_short_frames_zero_the_bytes_past_len(void)
{
    const uint8_t d[MAILBOX_PAYLOAD_BYTES] = {1u, 2u, 3u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, 3u));
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    mailbox_payload_bytes(pend.data, out);
    const uint8_t want[MAILBOX_PAYLOAD_BYTES] = {1u, 2u, 3u, 0u, 0u, 0u, 0u, 0u};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    TEST_ASSERT_EQUAL_UINT8(3u, pend.len);
}

static void test_a_deferred_frame_stays_pending_and_is_counted(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(1u, d);
    mailbox_pending_defer(&pend); /* nothing pending: not counted */
    TEST_ASSERT_EQUAL_UINT32(0u, pend.deferred);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, LEN));
    mailbox_pending_defer(&pend);
    mailbox_pending_defer(&pend);
    TEST_ASSERT_TRUE(pend.pending);
    TEST_ASSERT_EQUAL_UINT32(2u, pend.deferred);
    TEST_ASSERT_EQUAL_UINT32(0u, pend.overruns);
    mailbox_pending_done(&pend);
    TEST_ASSERT_FALSE(pend.pending);
    TEST_ASSERT_EQUAL_UINT32(1u, pend.done);
    mailbox_pending_done(&pend); /* nothing pending: not counted */
    TEST_ASSERT_EQUAL_UINT32(1u, pend.done);
}

static void test_a_newer_offer_replaces_a_pending_frame_as_an_overrun(void)
{
    uint8_t d1[MAILBOX_PAYLOAD_BYTES];
    uint8_t d2[MAILBOX_PAYLOAD_BYTES];
    frame(1u, d1);
    frame(2u, d2);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d1, LEN));
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d2, LEN));
    TEST_ASSERT_EQUAL_UINT32(1u, pend.overruns);
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    mailbox_payload_bytes(pend.data, out);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(d2, out, MAILBOX_PAYLOAD_BYTES); /* the newest is kept */
}

static void test_drop_clears_and_counts(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(1u, d);
    mailbox_pending_drop(&pend);
    TEST_ASSERT_EQUAL_UINT32(0u, pend.dropped);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, LEN));
    mailbox_pending_drop(&pend);
    TEST_ASSERT_FALSE(pend.pending);
    TEST_ASSERT_EQUAL_UINT32(1u, pend.dropped);
    TEST_ASSERT_EQUAL_UINT32(0u, pend.done);
}

static void test_the_first_read_is_only_the_baseline(void)
{
    write_frame(1u); /* a frame left from before a reset */
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, 100u, out));
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, 101u, out));
    TEST_ASSERT_EQUAL_UINT32(0u, rd.frames);
}

static void test_only_a_changed_write_seq_is_a_new_frame(void)
{
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, 0u, out));
    write_frame(7u);
    frame(7u, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 1u, out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, 2u, out)); /* unchanged: not again */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, 3u, out));
    TEST_ASSERT_EQUAL_UINT32(1u, rd.frames);
}

static void test_the_write_seq_wrap_is_a_new_frame(void)
{
    region.layout.slot[SLOT_IDX].write_seq = UINT32_MAX;
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, 0u, out));
    write_frame(3u); /* write_seq 0xFFFFFFFF -> 0 */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 1u, out));
    write_frame(4u); /* 0 -> 1 */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 2u, out));
}

static void test_a_len_mismatch_is_no_frame_and_is_never_delivered_later(void)
{
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, 0u, out));
    write_frame(1u);
    region.layout.slot[SLOT_IDX].len = (uint8_t)(LEN - 1u);
    TEST_ASSERT_EQUAL(MAILBOX_READ_BAD_LEN, read_frame(true, 1u, out));
    region.layout.slot[SLOT_IDX].len = LEN; /* len fixed, same write_seq */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, 2u, out));
    TEST_ASSERT_EQUAL_UINT32(1u, rd.bad_len);
    TEST_ASSERT_EQUAL_UINT32(0u, rd.frames);
}

static void test_a_layout_mismatch_drops_and_absorbs_the_frame(void)
{
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(false, 0u, out));
    write_frame(1u);
    TEST_ASSERT_EQUAL(MAILBOX_READ_BAD_LAYOUT, read_frame(false, 1u, out));
    /* the layout recovers: the frame written under the mismatch is not delivered */
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, 2u, out));
    write_frame(2u);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 3u, out));
    TEST_ASSERT_EQUAL_UINT32(1u, rd.bad_layout);
}

static void test_a_torn_write_without_commit_is_never_delivered(void)
{
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    uint8_t want[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, 0u, out));
    write_frame(1u);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 1u, out));
    /* the writer dies after the payload, before write_seq: half of frame 2 */
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(2u, d);
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, LEN));
    region.word[SLOT_WORD + MAILBOX_DATA_WORD] = pend.data[0];
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, 2u, out));
    /* after its restart the writer writes frame 3 completely: delivered whole */
    mailbox_pending_init(&pend);
    write_frame(3u);
    frame(3u, want);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 3u, out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, MAILBOX_PAYLOAD_BYTES);
}

static void test_a_stuck_sequence_reads_as_stale(void)
{
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_TRUE(mailbox_reader_stale(&rd, 0u, TIMEOUT_MS)); /* no baseline yet */
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, 1000u, out));
    TEST_ASSERT_FALSE(mailbox_reader_stale(&rd, 1000u + TIMEOUT_MS, TIMEOUT_MS));
    TEST_ASSERT_TRUE(mailbox_reader_stale(&rd, 1001u + TIMEOUT_MS, TIMEOUT_MS));
    write_frame(1u);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NEW, read_frame(true, 2000u, out));
    TEST_ASSERT_FALSE(mailbox_reader_stale(&rd, 2000u + TIMEOUT_MS, TIMEOUT_MS));
    /* the writer keeps running but write_seq is stuck: the frames are never fresh */
    for (uint32_t t = 2001u; t <= (2001u + TIMEOUT_MS); t++) {
        TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, read_frame(true, t, out));
    }
    TEST_ASSERT_TRUE(mailbox_reader_stale(&rd, 2001u + TIMEOUT_MS, TIMEOUT_MS));
    TEST_ASSERT_TRUE(mailbox_reader_stale(NULL, 0u, TIMEOUT_MS));
}

static void test_stale_ages_across_the_ms_wrap(void)
{
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    TEST_ASSERT_EQUAL(MAILBOX_READ_BASELINE, read_frame(true, UINT32_MAX - 5u, out));
    TEST_ASSERT_FALSE(mailbox_reader_stale(&rd, 10u, TIMEOUT_MS));
    TEST_ASSERT_TRUE(mailbox_reader_stale(&rd, TIMEOUT_MS, TIMEOUT_MS));
}

static void test_busy_and_null_arguments_are_harmless(void)
{
    mailbox_reader_busy(&rd);
    TEST_ASSERT_EQUAL_UINT32(1u, rd.busy);
    mailbox_reader_busy(NULL);
    mailbox_pending_init(NULL);
    mailbox_reader_init(NULL, LEN);
    mailbox_pending_defer(NULL);
    mailbox_pending_done(NULL);
    mailbox_pending_drop(NULL);
    mailbox_slot_store(NULL, slot());
    mailbox_slot_store(&pend, NULL);
    mailbox_slot_commit(NULL);
    mailbox_raw_t raw;
    mailbox_slot_load(NULL, &raw);
    mailbox_slot_load(slot(), NULL);
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, mailbox_reader_accept(NULL, &raw, true, 0u));
    TEST_ASSERT_EQUAL(MAILBOX_READ_NONE, mailbox_reader_accept(&rd, NULL, true, 0u));
    uint8_t out[MAILBOX_PAYLOAD_BYTES];
    mailbox_payload_bytes(NULL, out);
    mailbox_payload_bytes(pend.data, NULL);
    TEST_ASSERT_EQUAL_UINT8(0x34u, mailbox_len_of(0xABCDEF34u));
}

static void test_counters_saturate(void)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    frame(1u, d);
    pend.overruns = UINT32_MAX;
    pend.deferred = UINT32_MAX;
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, LEN));
    TEST_ASSERT_TRUE(mailbox_pending_offer(&pend, d, LEN));
    mailbox_pending_defer(&pend);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, pend.overruns);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, pend.deferred);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_word_indices_match_the_generated_layout);
    RUN_TEST(test_store_and_commit_fill_the_generated_slot_fields);
    RUN_TEST(test_commit_is_a_read_modify_write_that_wraps);
    RUN_TEST(test_offer_rejects_bad_arguments);
    RUN_TEST(test_short_frames_zero_the_bytes_past_len);
    RUN_TEST(test_a_deferred_frame_stays_pending_and_is_counted);
    RUN_TEST(test_a_newer_offer_replaces_a_pending_frame_as_an_overrun);
    RUN_TEST(test_drop_clears_and_counts);
    RUN_TEST(test_the_first_read_is_only_the_baseline);
    RUN_TEST(test_only_a_changed_write_seq_is_a_new_frame);
    RUN_TEST(test_the_write_seq_wrap_is_a_new_frame);
    RUN_TEST(test_a_len_mismatch_is_no_frame_and_is_never_delivered_later);
    RUN_TEST(test_a_layout_mismatch_drops_and_absorbs_the_frame);
    RUN_TEST(test_a_torn_write_without_commit_is_never_delivered);
    RUN_TEST(test_a_stuck_sequence_reads_as_stale);
    RUN_TEST(test_stale_ages_across_the_ms_wrap);
    RUN_TEST(test_busy_and_null_arguments_are_harmless);
    RUN_TEST(test_counters_saturate);
    return UNITY_END();
}
