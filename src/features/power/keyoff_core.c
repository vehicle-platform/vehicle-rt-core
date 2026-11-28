#include "features/power/keyoff_core.h"

#define REQUEST_ID_MASK 0x0Fu
#define MS_PER_S 1000u
#define DEADLINE_MAX_S 255u

#define REASON_NONE ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_NONE_CHOICE)
#define REASON_KEY_OFF ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_KEY_OFF_CHOICE)
#define REASON_LOW_VOLTAGE ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_LOW_VOLTAGE_CHOICE)
#define REQ_NONE ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REQUEST_NONE_CHOICE)
#define REQ_SHUTDOWN ((uint8_t)PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REQUEST_SHUTDOWN_CHOICE)
#define ACK_HALTING ((uint8_t)PLATFORM_SHUTDOWN_ACK_SHUTDOWN_ACK_STATE_HALTING_CHOICE)
#define ACK_READY ((uint8_t)PLATFORM_SHUTDOWN_ACK_SHUTDOWN_ACK_STATE_READY_TO_CUT_CHOICE)

/* Wrap-safe elapsed time between two timebase readings. */
static uint32_t keyoff_elapsed_ms(uint32_t since_ms, uint32_t now_ms)
{
    return now_ms - since_ms;
}

void keyoff_init(keyoff_t* k)
{
    k->phase = KEYOFF_PHASE_RUNNING;
    k->reason = REASON_NONE;
    k->request_id = 0u;
    k->halting_seen = false;
    k->seen_off = false;
    k->low_active = false;
    k->off_active = false;
    k->cyc_off_active = false;
    k->on_active = false;
    k->sequence_ms = 0u;
    k->cyc_off_since_ms = 0u;
    k->low_since_ms = 0u;
    k->off_since_ms = 0u;
    k->on_since_ms = 0u;
}

/* Tracks how long a level has been continuously true; true once it passed the debounce. */
static bool held_for(bool level, bool* active, uint32_t* since_ms, uint32_t now_ms, uint32_t limit_ms)
{
    bool held = false;
    if (level) {
        if (!*active) {
            *active = true;
            *since_ms = now_ms;
        }
        held = keyoff_elapsed_ms(*since_ms, now_ms) >= limit_ms;
    } else {
        *active = false;
    }
    return held;
}

static bool low_voltage_trips(keyoff_t* k, const keyoff_in_t* in, uint32_t now_ms)
{
    const bool low = in->kl15_valid && in->kl15_on && in->engine_stopped && in->battery_valid &&
                     (in->battery_mv < KEYOFF_LOW_VOLTAGE_MV);
    return held_for(low, &k->low_active, &k->low_since_ms, now_ms, KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS);
}

static void begin_sequence(keyoff_t* k, uint8_t reason, uint32_t now_ms)
{
    k->phase = KEYOFF_PHASE_SHUTTING_DOWN;
    k->reason = reason;
    k->request_id = (uint8_t)((k->request_id + 1u) & REQUEST_ID_MASK);
    k->halting_seen = false;
    k->seen_off = (reason == REASON_KEY_OFF);
    k->cyc_off_active = false;
    k->sequence_ms = now_ms;
    k->low_active = false;
}

static bool ack_matches(const keyoff_t* k, const keyoff_in_t* in)
{
    return in->ack_fresh && (in->ack_id == k->request_id);
}

static void to_running(keyoff_t* k)
{
    k->phase = KEYOFF_PHASE_RUNNING;
    k->reason = REASON_NONE;
    k->halting_seen = false;
    k->low_active = false;
}

/* Before HALTING a low-voltage sequence ends when the engine runs or the battery recovered. */
static bool low_voltage_cleared(const keyoff_t* k, const keyoff_in_t* in)
{
    return (k->reason == REASON_LOW_VOLTAGE) &&
           (in->engine_running || (in->battery_valid && (in->battery_mv >= KEYOFF_LOW_VOLTAGE_RECOVER_MV)));
}

static void run_shutting_down(keyoff_t* k, const keyoff_in_t* in, uint32_t now_ms, bool key_cycle)
{
    const bool acked = ack_matches(k, in);
    if (acked && (in->ack_state == ACK_HALTING)) {
        k->halting_seen = true;
    }
    if ((acked && (in->ack_state == ACK_READY)) ||
        (keyoff_elapsed_ms(k->sequence_ms, now_ms) >= KEYOFF_TIMEOUT_MS)) {
        k->phase = KEYOFF_PHASE_DONE;
    } else if (!k->halting_seen && (key_cycle || low_voltage_cleared(k, in))) {
        /* a real key cycle, or a cleared low-voltage cause, before the Pi started halting */
        to_running(k);
    } else {
        /* keep waiting for the Pi, bounded by the timeout above */
    }
}

/* DONE is latched: only a real key cycle (or, for low voltage, the engine starting) reopens. */
static void run_done(keyoff_t* k, const keyoff_in_t* in, bool key_cycle)
{
    if (key_cycle || ((k->reason == REASON_LOW_VOLTAGE) && in->engine_running)) {
        /* supply stays up through the diode-OR: reopen the logs, stop asking */
        to_running(k);
    }
}

static void run_phase(keyoff_t* k, const keyoff_in_t* in, uint32_t now_ms)
{
    const bool key_off = held_for(in->kl15_valid && !in->kl15_on && !in->engine_running,
                                  &k->off_active, &k->off_since_ms, now_ms, KEYOFF_KEY_DEBOUNCE_MS);
    const bool key_on = held_for(in->kl15_valid && in->kl15_on, &k->on_active, &k->on_since_ms,
                                 now_ms, KEYOFF_KEY_DEBOUNCE_MS);
    /* the off half of a key cycle is debounced too, so a connector glitch is not a key cycle */
    const bool off_held = held_for(in->kl15_valid && !in->kl15_on, &k->cyc_off_active,
                                   &k->cyc_off_since_ms, now_ms, KEYOFF_KEY_DEBOUNCE_MS);
    if ((k->phase != KEYOFF_PHASE_RUNNING) && off_held) {
        k->seen_off = true;
    }
    /* only a key-on that follows an off since the sequence started counts (MAJOR-1, round 2) */
    const bool key_cycle = key_on && k->seen_off;
    if (k->phase == KEYOFF_PHASE_RUNNING) {
        const bool low = low_voltage_trips(k, in, now_ms);
        if (key_off) {
            begin_sequence(k, REASON_KEY_OFF, now_ms);
        } else if (low) {
            begin_sequence(k, REASON_LOW_VOLTAGE, now_ms);
        } else {
            /* stay running */
        }
    } else if (k->phase == KEYOFF_PHASE_SHUTTING_DOWN) {
        run_shutting_down(k, in, now_ms, key_cycle);
    } else if (k->phase == KEYOFF_PHASE_DONE) {
        run_done(k, in, key_cycle);
    } else {
        /* corrupted phase value: start a new bounded shutdown instead of holding forever */
        begin_sequence(k, REASON_KEY_OFF, now_ms);
    }
}

static uint8_t deadline_left_s(const keyoff_t* k, uint32_t now_ms)
{
    uint32_t left_s = 0u;
    const uint32_t spent_ms = keyoff_elapsed_ms(k->sequence_ms, now_ms);
    if (spent_ms < KEYOFF_TIMEOUT_MS) {
        left_s = (KEYOFF_TIMEOUT_MS - spent_ms + (MS_PER_S - 1u)) / MS_PER_S;
    }
    if (left_s > DEADLINE_MAX_S) {
        left_s = DEADLINE_MAX_S;
    }
    return (uint8_t)left_s;
}

void keyoff_step(keyoff_t* k, const keyoff_in_t* in, uint32_t now_ms, keyoff_out_t* out)
{
    run_phase(k, in, now_ms);

    out->hold = (k->phase != KEYOFF_PHASE_DONE) || (in->kl15_valid && in->kl15_on);
    out->close_logs = (k->phase != KEYOFF_PHASE_RUNNING);
    out->request_id = k->request_id;
    if (k->phase != KEYOFF_PHASE_RUNNING) {
        /* DONE keeps asking, so a Pi that is merely slow still halts before the cut */
        out->request = REQ_SHUTDOWN;
        out->reason = k->reason;
        /* in DONE 0 means "no further deadline: the cut may come at any time" */
        out->deadline_s = (k->phase == KEYOFF_PHASE_DONE) ? 0u : deadline_left_s(k, now_ms);
    } else {
        out->request = REQ_NONE;
        out->reason = REASON_NONE;
        out->deadline_s = 0u;
    }
}
