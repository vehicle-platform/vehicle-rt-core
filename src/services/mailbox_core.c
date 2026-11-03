#include "services/mailbox_core.h"

/* The word view must match the generated layout exactly (D-093 item 4). */
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "the mailbox word view assumes a little-endian core (STM32H7)"
#endif
_Static_assert(sizeof(mailbox_region_t) == PLATFORM_MAILBOX_USED_BYTES,
               "the word view covers exactly the used region");
_Static_assert((PLATFORM_MAILBOX_SLOT_BYTES % MAILBOX_WORD_BYTES) == 0u, "slots are whole words");
_Static_assert((PLATFORM_MAILBOX_HEADER_BYTES % MAILBOX_WORD_BYTES) == 0u, "the header is whole words");
_Static_assert(MAILBOX_SLOT_WORDS == 4u, "a slot is write_seq, the len word and two data words");
_Static_assert((offsetof(platform_mailbox_slot_t, write_seq) % MAILBOX_WORD_BYTES) == 0u, "write_seq is a word");
_Static_assert((offsetof(platform_mailbox_slot_t, len) % MAILBOX_WORD_BYTES) == 0u,
               "len is the low byte of its own word");
_Static_assert((offsetof(platform_mailbox_slot_t, data) % MAILBOX_WORD_BYTES) == 0u, "data is word aligned");
_Static_assert(sizeof(((platform_mailbox_slot_t*)NULL)->data) == MAILBOX_PAYLOAD_BYTES, "8 payload bytes");
_Static_assert(MAILBOX_SEQ_WORD != MAILBOX_LEN_WORD, "distinct slot words");
_Static_assert((MAILBOX_DATA_WORD + MAILBOX_PAYLOAD_WORDS) == MAILBOX_SLOT_WORDS, "data fills the slot end");
_Static_assert(MAILBOX_M7_LAYOUT_WORD != MAILBOX_M4_LAYOUT_WORD, "distinct layout words");

#define BYTE_MASK 0xFFu
#define BITS_PER_BYTE 8u

static void mbc_sat_inc(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

void mailbox_pending_init(mailbox_pending_t* p)
{
    if (p != NULL) {
        p->data[0] = 0u;
        p->data[1] = 0u;
        p->len = 0u;
        p->pending = false;
        p->done = 0u;
        p->deferred = 0u;
        p->overruns = 0u;
        p->dropped = 0u;
    }
}

bool mailbox_pending_offer(mailbox_pending_t* p, const uint8_t* data, uint8_t len)
{
    if ((p == NULL) || (data == NULL) || (len == 0u) || (len > MAILBOX_PAYLOAD_BYTES)) {
        return false;
    }
    uint32_t words[MAILBOX_PAYLOAD_WORDS] = {0u, 0u};
    for (uint32_t i = 0u; i < (uint32_t)len; i++) {
        const uint32_t shift = (i % MAILBOX_WORD_BYTES) * BITS_PER_BYTE;
        words[i / MAILBOX_WORD_BYTES] |= ((uint32_t)data[i]) << shift;
    }
    if (p->pending) {
        mbc_sat_inc(&p->overruns); /* the older frame is lost; the receiver's E2E sees the gap */
    }
    p->data[0] = words[0];
    p->data[1] = words[1];
    p->len = len;
    p->pending = true;
    return true;
}

void mailbox_pending_defer(mailbox_pending_t* p)
{
    if ((p != NULL) && p->pending) {
        mbc_sat_inc(&p->deferred);
    }
}

void mailbox_pending_done(mailbox_pending_t* p)
{
    if ((p != NULL) && p->pending) {
        p->pending = false;
        mbc_sat_inc(&p->done);
    }
}

void mailbox_pending_drop(mailbox_pending_t* p)
{
    if ((p != NULL) && p->pending) {
        p->pending = false;
        mbc_sat_inc(&p->dropped);
    }
}

void mailbox_slot_store(const mailbox_pending_t* p, volatile uint32_t* slot)
{
    if ((p != NULL) && (slot != NULL)) {
        slot[MAILBOX_DATA_WORD] = p->data[0];
        slot[MAILBOX_DATA_WORD + 1u] = p->data[1];
        slot[MAILBOX_LEN_WORD] = (uint32_t)p->len; /* reserved bytes written as 0 */
    }
}

void mailbox_slot_commit(volatile uint32_t* slot)
{
    if (slot != NULL) {
        const uint32_t seq = slot[MAILBOX_SEQ_WORD];
        slot[MAILBOX_SEQ_WORD] = seq + 1u; /* modulo 2^32 */
    }
}

void mailbox_reader_init(mailbox_reader_t* r, uint8_t expected_len)
{
    if (r != NULL) {
        r->expected_len = expected_len;
        r->has_baseline = false;
        r->last_seq = 0u;
        r->fresh_at_ms = 0u;
        r->frames = 0u;
        r->busy = 0u;
        r->bad_layout = 0u;
        r->bad_len = 0u;
    }
}

void mailbox_slot_load(const volatile uint32_t* slot, mailbox_raw_t* raw)
{
    if ((slot != NULL) && (raw != NULL)) {
        raw->seq = slot[MAILBOX_SEQ_WORD];
        raw->len_word = slot[MAILBOX_LEN_WORD];
        raw->data[0] = slot[MAILBOX_DATA_WORD];
        raw->data[1] = slot[MAILBOX_DATA_WORD + 1u];
    }
}

mailbox_read_t mailbox_reader_accept(mailbox_reader_t* r, const mailbox_raw_t* raw,
                                     bool peer_layout_ok, uint32_t now_ms)
{
    if ((r == NULL) || (raw == NULL)) {
        return MAILBOX_READ_NONE;
    }
    if (!r->has_baseline) {
        r->has_baseline = true;
        r->last_seq = raw->seq;
        r->fresh_at_ms = now_ms;
        return MAILBOX_READ_BASELINE;
    }
    if (raw->seq == r->last_seq) {
        return MAILBOX_READ_NONE;
    }
    r->last_seq = raw->seq; /* absorbed whatever the verdict: never delivered later */
    if (!peer_layout_ok) {
        mbc_sat_inc(&r->bad_layout);
        return MAILBOX_READ_BAD_LAYOUT;
    }
    if (mailbox_len_of(raw->len_word) != r->expected_len) {
        mbc_sat_inc(&r->bad_len);
        return MAILBOX_READ_BAD_LEN;
    }
    r->fresh_at_ms = now_ms;
    mbc_sat_inc(&r->frames);
    return MAILBOX_READ_NEW;
}

void mailbox_reader_busy(mailbox_reader_t* r)
{
    if (r != NULL) {
        mbc_sat_inc(&r->busy);
    }
}

bool mailbox_reader_stale(const mailbox_reader_t* r, uint32_t now_ms, uint32_t timeout_ms)
{
    if ((r == NULL) || !r->has_baseline) {
        return true;
    }
    return (uint32_t)(now_ms - r->fresh_at_ms) > timeout_ms;
}

void mailbox_payload_bytes(const uint32_t words[MAILBOX_PAYLOAD_WORDS], uint8_t data[MAILBOX_PAYLOAD_BYTES])
{
    if ((words != NULL) && (data != NULL)) {
        for (uint32_t i = 0u; i < MAILBOX_PAYLOAD_BYTES; i++) {
            const uint32_t shift = (i % MAILBOX_WORD_BYTES) * BITS_PER_BYTE;
            data[i] = (uint8_t)((words[i / MAILBOX_WORD_BYTES] >> shift) & BYTE_MASK);
        }
    }
}

uint8_t mailbox_len_of(uint32_t len_word)
{
    return (uint8_t)(len_word & BYTE_MASK);
}
