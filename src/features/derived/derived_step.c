/*
 * Scheduler of the pure derived core (D-070): ingest, the grid step, the stop event, the class-B
 * ride rows and the epoch step.
 * Split from the pure derived-signal core (D-070); see derived_core.h for the sampling rule and
 * features/derived/README.md for the rationale. The units talk through derived_core_internal.h only.
 */
#include "features/derived/derived_core.h"

#include "features/derived/derived_core_internal.h"
#include "features/derived/derived_gear.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* isfinite() and the NaN-false comparisons need IEEE semantics, logf() must not set errno
 * (like lean_core.c: -fno-math-errno, never -ffast-math). */
#if defined(__FAST_MATH__)
#error "derived_step.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_step.c needs -fno-math-errno"
#endif

#define HALF_RANGE (0x7FFFFFFFu)

/* ---- stop event --------------------------------------------------------------------------------- */

static void stop_step(derived_core_t* c, uint32_t sig)
{
    const derived_did_t* const speed = &c->did[VEHICLE_CL250_IDX_VEHICLE_SPEED];
    if (speed->fresh_now && derived_core_is_finite_f(speed->value)) {
        derived_stop_t* const st = &c->stop;
        const uint32_t t = speed->stamp_ms;
        const uint32_t speed_limit =
            (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms; /* D-087 */
        c->evals_step++;
        if (st->has_prev && (((uint32_t)(t - st->prev_ms) > speed_limit))) {
            st->started = false; /* nothing is followed across a gap */
            st->moving = false;
        }
        st->prev_ms = t;
        st->has_prev = true;
        if (st->started) {
            if (speed->value > 0.0f) {
                const uint32_t duration_ms = t - st->start_ms;
                derived_core_store(c, sig, t, DERIVED_FLAG_NONE, (float)duration_ms / DERIVED_MS_PER_S);
                st->started = false;
            }
        } else if (is_zero(speed->value) && st->moving) {
            float rpm = 0.0f;
            const uint32_t rpm_limit =
                (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms; /* D-087 */
            if ((did_fresh(c, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, t, rpm_limit, &rpm) == DERIVED_FLAG_NONE) &&
                (rpm > 0.0f)) {
                st->started = true;
                st->start_ms = t;
            }
        } else {
            /* standing without having moved, or still moving: nothing */
        }
        st->moving = speed->value > 0.0f;
    }
}

/* ---- the step ------------------------------------------------------------------------------------ */

static void record_sample(derived_core_t* c, uint32_t i, float value, uint32_t stamp)
{
    const uint32_t ring = ring_of_did(i);
    if (ring != RING_NONE) {
        derived_core_ring_push(c, ring, value, stamp);
    }
    if ((i == (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE) && (!c->baro_ref_set) && derived_core_is_finite_f(value) &&
        (value > 0.0f)) {
        c->baro_ref = value;
        c->baro_ref_set = true;
    }
}

static void ingest(derived_core_t* c, uint32_t now, const derived_in_t* in)
{
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        derived_did_t* const d = &c->did[i];
        c->dq_present[i] = in[i].present; /* D-104: independent of the ride-local ignore below */
        c->dq_stamp_ms[i] = in[i].stamp_ms;
        d->fresh_now = false;
        if (d->ignore_valid && in[i].present && (in[i].stamp_ms == d->ignore_stamp_ms)) {
            d->present = false; /* the old epoch's last sample, held by the producer */
        } else if (!in[i].present) {
            d->present = false;
        } else {
            d->ignore_valid = false;
            const bool is_new = (!d->present) || (in[i].stamp_ms != d->stamp_ms);
            d->present = true;
            d->value = in[i].value;
            d->stamp_ms = in[i].stamp_ms;
            if (is_new) {
                d->fresh_now = true;
                d->lapsed = false;
                if (!d->seen) {
                    d->seen = true;
                    d->first_ms = in[i].stamp_ms;
                }
                record_sample(c, i, in[i].value, in[i].stamp_ms);
                if (c->ride.open) {
                    derived_ride_sample(&c->ride.acc, i, in[i].value, in[i].stamp_ms);
                    derived_windows_push_did(&c->ride.acc.win, i, in[i].value, in[i].stamp_ms);
                }
            }
            const uint32_t a = now - d->stamp_ms;
            if ((a > c->max_stale_ms) && (a <= HALF_RANGE)) {
                d->lapsed = true; /* a wrapped counter can never make it fresh again */
            }
        }
    }
    if (c->ride.open) {
        derived_ride_settle(&c->ride.acc); /* the fan-on step of this ingest's battery sample */
    }
}

static void event_step(derived_core_t* c, uint32_t sig)
{
    switch (sig) {
    case VEHICLE_CL250_DERIVED_STOP_EVENT:
        stop_step(c, sig);
        break;
    default:
        derived_d103_event_step(c, sig);
        break;
    }
}

/* Arms the grid: start = the latest first-sample time of the inputs (a derived input: the
 * producer's grid start). False while an input has never been seen. */
static bool try_arm(derived_core_t* c, uint32_t sig, uint32_t now)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    derived_sched_t* const s = &c->sched[sig];
    if (sig == (uint32_t)VEHICLE_CL250_DERIVED_DATA_QUALITY_MASK) {
        /* D-104: no DID input to wait for; unknown quality is a result too, so the grid
         * starts at the first step after the glue handed the quality in. */
        if (c->quality.known) {
            s->armed = true;
            s->start_ms = now;
            s->next_ms = now;
        }
        return s->armed;
    }
    bool complete = true;
    uint32_t start = 0u;
    bool first = true;
    for (uint32_t k = 0u; (k < (uint32_t)spec->input_count) && (k < VEHICLE_CL250_DERIVED_INPUT_MAX); k++) {
        const uint32_t idx = spec->inputs[k].index;
        uint32_t t = 0u;
        if (spec->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
            complete = complete && (idx < VEHICLE_CL250_DID_COUNT) && c->did[idx].seen;
            t = c->did[idx].first_ms;
        } else {
            complete = complete && (idx < VEHICLE_CL250_DERIVED_COUNT) && c->sched[idx].armed;
            t = c->sched[idx].start_ms;
        }
        if (complete) {
            start = first ? t : later(start, t);
            first = false;
        }
    }
    if (complete && !first) {
        s->armed = true;
        s->start_ms = start;
        s->next_ms = start;
    }
    return s->armed;
}

static void grid_step(derived_core_t* c, uint32_t sig, uint32_t now)
{
    derived_sched_t* const s = &c->sched[sig];
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    if ((s->armed || try_arm(c, sig, now)) && derived_core_at_or_after(now, s->next_ms)) {
        const uint32_t period = (uint32_t)spec->period_ms;
        const uint32_t tick = s->next_ms + (((now - s->next_ms) / period) * period);
        const derived_ctx_t x = {c, now, sig, spec};
        float value = 0.0f;
        const derived_flag_t f = (kind_of(sig) == KIND_INTEGRAL) ? eval_integral(&x, &value) : eval_point(&x, &value);
        derived_core_store(c, sig, tick, f, value);
        s->next_ms = tick + period;
        c->evals_step++;
    }
}

/* A new ride starts from a fresh class-A state (the reference evaluates every ride on its own):
 * grids, integrals, windows, the altitude reference and the stop event restart, and the DID
 * samples held now (the old epoch's) are ignored until a new sample arrives. Sequence
 * counters keep counting. */
static void reset_class_a(derived_core_t* c)
{
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        /* data_quality_mask belongs to the tester, not the ride (D-104): its grid runs on */
        if (c->sched[sig].computed && (kind_of(sig) != KIND_RIDE) &&
            (sig != (uint32_t)VEHICLE_CL250_DERIVED_DATA_QUALITY_MASK)) {
            derived_out_t* const o = &c->out[sig];
            o->status = DERIVED_NO_DATA;
            o->flag = DERIVED_FLAG_NONE;
            o->value = 0.0f;
            o->unranged = 0.0f;
            c->sched[sig].armed = false;
            c->sched[sig].start_ms = 0u;
            c->sched[sig].next_ms = 0u;
        }
    }
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        derived_did_t* const d = &c->did[i];
        if (d->present) {
            d->ignore_valid = true;
            d->ignore_stamp_ms = d->stamp_ms;
        }
        d->present = false;
        d->seen = false;
        d->fresh_now = false;
        d->lapsed = false;
        d->first_ms = 0u;
    }
    (void)memset(c->integral, 0, sizeof c->integral);
    (void)memset(&c->stop, 0, sizeof c->stop);
    (void)memset(&c->d103, 0, sizeof c->d103);
    (void)memset(c->ring, 0, sizeof c->ring);
    c->baro_ref = 0.0f;
    c->baro_ref_set = false;
}

/*
 * The ride follows the epoch (D-087 item 2): a changed counter (!=, any jump, wrap-safe) or
 * a closed epoch closes the open ride at once, then an open epoch with no ride opens one.
 * Close + reopen in one record (counter + 1, open still true) is therefore two events in
 * this order. A power-down close is not handled (no persistence yet, D-045).
 */
static void epoch_step(derived_core_t* c, uint32_t now, const derived_epoch_t* e)
{
    derived_ride_t* const r = &c->ride;
    r->last_now_ms = now;
    if ((e != NULL) && e->known) {
        if (r->open && ((!e->open) || (e->counter != r->counter))) {
            derived_windows_end(&r->acc.win); /* the last window point, an open instance closes there */
            derived_ride_close(r, now);
        }
        if ((!r->open) && e->open) {
            reset_class_a(c);
            derived_ride_begin(r, e->counter);
            derived_windows_begin(&r->acc.win, c->win_mask, c->rpm_hist, &c->ring[RING_RPM_HIST], DERIVED_WIN_RPM_HIST_CAP);
        }
    }
}

/*
 * The reference evaluates a grid signal on [start, end], end = the last time every input has a
 * sample (a derived input: its own last grid tick). The class-B summaries must not see the rows
 * after it, so a row of the signals in derived_ride_is_tail_signal() waits in the ride until
 * every input has a sample at or after its tick. `end_of[sig]` is that bound, computed in
 * table order (a producer comes before its consumer), and the waiting rows up to it are
 * confirmed. A signal with an input never seen has no bound: its rows keep waiting.
 */
static void ride_confirm_rows(derived_core_t* c)
{
    if (!c->ride.open) {
        return;
    }
    uint32_t end_of[VEHICLE_CL250_DERIVED_COUNT];
    bool has_end[VEHICLE_CL250_DERIVED_COUNT];
    (void)memset(end_of, 0, sizeof end_of);
    (void)memset(has_end, 0, sizeof has_end);
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
        if ((!c->sched[sig].computed) || (spec->derived_class != VEHICLE_CL250_DERIVED_CLASS_LIVE)) {
            continue;
        }
        bool ok = true;
        bool first = true;
        uint32_t bound = 0u;
        for (uint32_t k = 0u; (k < (uint32_t)spec->input_count) && (k < VEHICLE_CL250_DERIVED_INPUT_MAX) && ok; k++) {
            const uint32_t idx = spec->inputs[k].index;
            uint32_t t = 0u;
            if (spec->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
                ok = (idx < VEHICLE_CL250_DID_COUNT) && c->did[idx].seen;
                t = c->did[idx].stamp_ms;
            } else if ((idx < VEHICLE_CL250_DERIVED_COUNT) && has_end[idx]) {
                const vehicle_cl250_derived_t* const dep = &vehicle_cl250_derived[idx];
                if (dep->event) {
                    t = c->out[idx].stamp_ms; /* an event input: the last event row */
                } else {
                    const derived_sched_t* const ds = &c->sched[idx];
                    const uint32_t span = end_of[idx] - ds->start_ms;
                    ok = ds->armed && (span <= HALF_RANGE);
                    t = ds->start_ms + ((span / (uint32_t)dep->period_ms) * (uint32_t)dep->period_ms);
                }
            } else {
                ok = false;
            }
            if (ok) {
                bound = first ? t : (derived_core_at_or_after(bound, t) ? t : bound);
                first = false;
            }
        }
        if (ok && (!first)) {
            end_of[sig] = bound;
            has_end[sig] = true;
            if (derived_ride_is_tail_signal(sig)) {
                derived_ride_advance(&c->ride.acc, sig, bound);
            }
        }
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived glue and the tests
void derived_core_set_quality(derived_core_t* c, const derived_quality_in_t* q)
{
    if ((c != NULL) && (q != NULL)) {
        c->quality = *q;
        c->quality.known = true;
    }
}

void derived_core_step_epoch(derived_core_t* c, uint32_t now_ms, const derived_in_t* in,
                             const derived_epoch_t* epoch)
{
    if ((c == NULL) || (in == NULL)) {
        return;
    }
    c->evals_step = 0u;
    epoch_step(c, now_ms, epoch);
    ingest(c, now_ms, in);
    derived_d103_record(c);
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        if (c->sched[sig].computed) {
            const uint32_t kind = kind_of(sig);
            if (kind == KIND_INTEGRAL) {
                integral_update(c, sig);
                grid_step(c, sig, now_ms);
            } else if (kind == KIND_POINT) {
                grid_step(c, sig, now_ms);
            } else {
                /* an event sees the derived outputs of this step; a class B summary is read at
                 * the close from what was accumulated above */
            }
        }
    }
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        if (c->sched[sig].computed && (kind_of(sig) == KIND_EVENT)) {
            event_step(c, sig);
        }
    }
    if (c->ride.open) {
        derived_windows_run(&c->ride.acc.win); /* D-079 windows: this step's samples, in time order */
    }
    ride_confirm_rows(c);
    if (c->evals_step > c->evals_max) {
        c->evals_max = c->evals_step;
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: the class-A-only step, called by the tests
void derived_core_step(derived_core_t* c, uint32_t now_ms, const derived_in_t* in)
{
    derived_core_step_epoch(c, now_ms, in, NULL);
}
