/*
 * D-079 health windows (pure), event engine: items, open/close, the point step, expiries and the public run/push/end. See derived_windows.h for the rules and the reference
 * (vehicle-server derived/health_windows.py, health_window_summaries.py). Split out of one file (T27);
 * the units talk through derived_windows_internal.h only.
 */
#include "features/derived/derived_windows.h"

#include "features/derived/derived_gear.h"
#include "features/derived/derived_windows_internal.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__)
#error "derived_windows.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_windows.c needs -fno-math-errno"
#endif


static bool after(uint32_t a, uint32_t b)
{
    return (a != b) && at_or_after(a, b);
}

static void apply_item(derived_windows_t* a, uint32_t wi, const derived_win_item_t* it)
{
    derived_win_t* const w = &a->w[wi];
    derived_win_stats_t* const st = &w->st;
    const uint32_t max = vehicle_cl250_window[wi].max_duration_ms;
    if ((!at_or_after(it->t, w->open_ms)) || ((max != 0u) && ((it->t - w->open_ms) > max))) {
        return;
    }
    const double v = (double)it->v;
    if (it->kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) {
        if (it->index == (uint8_t)VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY) {
            add_mean(st, DERIVED_WIN_MEAN_VE, v);
        } else if (st->ratio_n < DERIVED_WIN_CRUISE_RATIO_CAP) {
            /* a final-drive estimate: ordered insert, so the median needs no sort */
            uint32_t j = st->ratio_n;
            while ((j > 0u) && (a->ratio_buf[j - 1u] > it->v)) {
                a->ratio_buf[j] = a->ratio_buf[j - 1u];
                j--;
            }
            a->ratio_buf[j] = it->v;
            st->ratio_n++;
        } else {
            w->broken = true; /* never a wrong median */
        }
        return;
    }
    switch (it->index) {
    case (uint8_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE:
        add_mean(st, DERIVED_WIN_MEAN_BATTERY, v);
        break;
    case (uint8_t)VEHICLE_CL250_IDX_COOLANT_TEMP:
        add_mean(st, DERIVED_WIN_MEAN_COOLANT, v);
        if (!st->has_coolant) {
            st->coolant_first = it->v;
            st->has_coolant = true;
        }
        st->coolant_last = it->v;
        if (wi == VEHICLE_CL250_WINDOW_W5_POST_STOP) {
            uint16_t raw = 0u;
            const uint32_t gap = it->t - st->coolant_tl;
            if ((st->coolant_n < DERIVED_WIN_POST_STOP_COOLANT_CAP) && cool_encode(it->v, &raw) &&
                ((st->coolant_n == 0u) || (gap <= 0xFFFFu))) {
                if (st->coolant_n == 0u) {
                    st->coolant_t0 = it->t;
                }
                a->coolant_buf[st->coolant_n].dt_ms = (uint16_t)((st->coolant_n == 0u) ? 0u : gap);
                a->coolant_buf[st->coolant_n].raw = raw;
                st->coolant_tl = it->t;
                st->coolant_n++;
            } else {
                w->broken = true; /* full, not a decoded raw, or a gap above 65 s: never a wrong tau */
            }
        }
        break;
    case (uint8_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP:
        add_mean(st, DERIVED_WIN_MEAN_INTAKE, v);
        break;
    case (uint8_t)VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP:
        add_mean(st, DERIVED_WIN_MEAN_AMBIENT, v);
        break;
    case (uint8_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE:
        add_mean(st, DERIVED_WIN_MEAN_BARO, v);
        break;
    case (uint8_t)VEHICLE_CL250_IDX_INTAKE_MAP:
        add_mean(st, DERIVED_WIN_MEAN_MAP, v);
        if ((!st->has_map) || (it->v < st->map_min)) {
            st->map_min = it->v;
        }
        if ((!st->has_map) || (it->v > st->map_max)) {
            st->map_max = it->v;
        }
        st->has_map = true;
        break;
    case (uint8_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1: {
        st->stft_n++;
        const double delta = v - st->stft_mean;
        st->stft_mean += delta / (double)st->stft_n;
        st->stft_m2 += delta * (v - st->stft_mean);
        break;
    }
    default:
        break;
    }
}

/* Applies the pending samples before `t` (equal == false) or at `t` (equal == true). */
static void apply_items(derived_windows_t* a, uint32_t wi, uint32_t t, bool equal)
{
    const derived_win_t* const w = &a->w[wi];
    for (uint32_t i = 0u; i < w->pend_n; i++) {
        const derived_win_item_t it = w->pend[i];
        if (equal ? (it.t == t) : after(t, it.t)) {
            apply_item(a, wi, &it);
        }
    }
}

/* A sample is inside [open, close): a close by max duration or the ride end keeps its boundary
 * sample (health_window_summaries.py _inside). */
static bool inside(uint32_t s, uint32_t open_ms, uint32_t close_ms, uint8_t reason)
{
    bool r = at_or_after(s, open_ms) && at_or_after(close_ms, s);
    if (r && (s == close_ms)) {
        r = (reason == CLOSE_MAX_DURATION) || (reason == CLOSE_SESSION_END);
    }
    return r;
}

/* Warm-idle engine speed spread and the drift of the second half over the first, from the
 * core's ENGINE_SPEED history (it holds every sample of the up-to-60 s instance). */
void compute_rpm(const derived_windows_t* a, derived_win_stats_t* st, uint32_t open_ms, uint32_t close_ms,
                        uint8_t reason)
{
    st->rpm_spread_ok = false;
    st->rpm_drift_ok = false;
    if ((a->rpm_data == NULL) || (a->rpm_ring == NULL) || (a->rpm_cap == 0u)) {
        return;
    }
    const uint32_t span = close_ms - open_ms;
    const double half = (double)span / 2.0;
    double lo = 0.0;
    double hi = 0.0;
    double s1 = 0.0;
    double s2 = 0.0;
    uint32_t n1 = 0u;
    uint32_t n2 = 0u;
    for (uint32_t k = a->rpm_ring->count; k > 0u; k--) {
        const derived_sample_t* const s = &a->rpm_data[((uint32_t)a->rpm_ring->head + a->rpm_cap + 1u - k) % a->rpm_cap];
        if (!inside(s->stamp_ms, open_ms, close_ms, reason)) {
            continue;
        }
        const double v = (double)s->value;
        lo = ((n1 + n2) == 0u) ? v : ((v < lo) ? v : lo);
        hi = ((n1 + n2) == 0u) ? v : ((v > hi) ? v : hi);
        const uint32_t rel = s->stamp_ms - open_ms;
        if ((double)rel < half) {
            s1 += v;
            n1++;
        } else {
            s2 += v;
            n2++;
        }
    }
    st->rpm_spread_ok = (n1 + n2) > 0u;
    st->rpm_spread = hi - lo;
    st->rpm_drift_ok = (n1 > 0u) && (n2 > 0u);
    if (st->rpm_drift_ok) {
        st->rpm_drift = (s2 / (double)n2) - (s1 / (double)n1);
    }
}

/* ---- the state machine ------------------------------------------------------------------------ */

static void do_open(derived_windows_t* a, uint32_t wi, uint32_t t)
{
    derived_win_t* const w = &a->w[wi];
    const vehicle_cl250_window_t* const spec = &vehicle_cl250_window[wi];
    w->open = true;
    w->open_ms = t;
    for (uint32_t k = 0u; k < spec->stable_count; k++) {
        double v = 0.0;
        (void)ch_latest(&w->ch[w->stable_ch[k]], t, &v);
        w->lo[k] = v;
        w->hi[k] = v;
    }
    if (!w->has_first) {
        (void)memset(&w->st, 0, sizeof w->st);
    }
}

static void finalize_close(derived_windows_t* a, uint32_t wi, uint32_t close_ms, uint8_t reason)
{
    derived_win_t* const w = &a->w[wi];
    const vehicle_cl250_window_t* const spec = &vehicle_cl250_window[wi];
    const uint32_t duration = close_ms - w->open_ms;
    const bool qualifies = (reason == CLOSE_MAX_DURATION) || (duration >= spec->min_duration_ms);
    if (qualifies) {
        if (w->count < UINT32_MAX) {
            w->count++;
        }
        if (!w->has_first) {
            w->has_first = true;
            w->first_open_ms = w->open_ms;
            w->first_close_ms = close_ms;
            w->first_reason = reason;
            if (wi == VEHICLE_CL250_WINDOW_W3_WARM_IDLE) {
                compute_rpm(a, &w->st, w->open_ms, close_ms, reason);
            }
        }
    } else if (!w->has_first) {
        (void)memset(&w->st, 0, sizeof w->st);
    } else {
        /* the first instance is already kept */
    }
    w->open = false;
}

/* One timeline point of window `wi` (health_windows.py evaluate_window loop body). */
static void win_eval_point(derived_windows_t* a, uint32_t wi, uint32_t t)
{
    derived_win_t* const w = &a->w[wi];
    const vehicle_cl250_window_t* const spec = &vehicle_cl250_window[wi];
    bool transitioned = false;
    if (is_transition(spec->opens_on) && w->trig_has) {
        const double cur = w->trig_v;
        const bool consecutive = w->has_prev && ((t - w->prev_t) <= w->ch[w->trig_ch].stale_ms);
        if (consecutive) {
            transitioned = (spec->opens_on == VEHICLE_CL250_WINDOW_OPEN_ENGINE_START) ? (win_is_zero(w->prev_v) && (cur > 0.0))
                                                                                       : ((w->prev_v > 0.0) && win_is_zero(cur));
        }
        w->has_prev = true;
        w->prev_v = cur;
        w->prev_t = t;
    }
    const bool collecting = !w->has_first;
    const bool was_open = w->open;
    if (was_open) {
        if (collecting) {
            apply_items(a, wi, t, false);
        }
        uint8_t reason = 0u;
        uint32_t close_ms = t;
        if ((spec->max_duration_ms != 0u) && ((t - w->open_ms) >= spec->max_duration_ms)) {
            reason = CLOSE_MAX_DURATION;
            close_ms = w->open_ms + spec->max_duration_ms;
        } else if (spec->opens_on != VEHICLE_CL250_WINDOW_OPEN_ENGINE_START) {
            if (!conds_hold(w, spec, t)) {
                reason = CLOSE_CONDITION;
            } else {
                reason = stable_close(w, spec, t);
            }
        } else {
            double v = 0.0;
            if (!ch_latest(&w->ch[w->trig_ch], t, &v)) {
                reason = CLOSE_UNKNOWN;
            } else if (win_is_zero(v)) {
                reason = CLOSE_ENGINE_ZERO;
            } else {
                reason = 0u;
            }
        }
        if (reason != 0u) {
            if (collecting && (reason == CLOSE_MAX_DURATION) && (close_ms == t)) {
                apply_items(a, wi, t, true); /* the boundary sample of a max duration close */
            }
            finalize_close(a, wi, close_ms, reason);
        }
    }
    if (!w->open) {
        switch (spec->opens_on) {
        case VEHICLE_CL250_WINDOW_OPEN_CONDITION:
            if (was_open) {
                w->can_open = false;
            }
            if (!can_open_at(w, spec, t)) {
                w->can_open = true;
            } else if (w->can_open) {
                do_open(a, wi, t);
            } else {
                /* armed only after the conditions were not met once */
            }
            break;
        case VEHICLE_CL250_WINDOW_OPEN_KEY_ON:
            if (!w->key_on_seen) {
                w->key_on_seen = true;
                if (can_open_at(w, spec, t)) {
                    do_open(a, wi, t);
                }
            }
            break;
        default:
            if (transitioned && can_open_at(w, spec, t)) {
                do_open(a, wi, t);
            }
            break;
        }
        if (w->open && (!w->has_first)) {
            apply_items(a, wi, t, true);
        }
    } else if (!w->has_first) {
        apply_items(a, wi, t, true);
    } else {
        /* the first qualifying instance is kept: nothing more to collect */
    }
    w->pend_n = 0u;
    w->last_t = t;
    w->has_last = true;
}

static void flush_point(derived_windows_t* a, uint32_t wi)
{
    derived_win_t* const w = &a->w[wi];
    if (w->staged) {
        win_eval_point(a, wi, w->staged_t);
        w->staged = false;
        w->trig_has = false;
    }
}

/* Expiry points strictly between the last point and `t` (a later window sample proves them). */
static void run_expiries(derived_windows_t* a, uint32_t wi, uint32_t t)
{
    derived_win_t* const w = &a->w[wi];
    for (uint32_t guard = 0u; guard <= DERIVED_WINDOWS_CH_MAX; guard++) {
        bool found = false;
        uint32_t best = 0u;
        for (uint32_t i = 0u; i < w->nch; i++) {
            derived_win_chan_t* const c = &w->ch[i];
            if (c->exp_pending && w->has_last && (!after(c->exp_t, w->last_t))) {
                c->exp_pending = false; /* coincided with a point already evaluated */
            } else if (c->exp_pending && after(t, c->exp_t) && ((!found) || after(best, c->exp_t))) {
                best = c->exp_t;
                found = true;
            } else {
                /* not due */
            }
        }
        if (!found) {
            break;
        }
        for (uint32_t i = 0u; i < w->nch; i++) {
            if (w->ch[i].exp_pending && (w->ch[i].exp_t == best)) {
                w->ch[i].exp_pending = false;
            }
        }
        w->trig_has = false;
        win_eval_point(a, wi, best);
    }
}

static bool expiry_due_by(const derived_win_t* w, uint32_t t)
{
    for (uint32_t i = 0u; i < w->nch; i++) {
        const derived_win_chan_t* const c = &w->ch[i];
        if (c->exp_pending && ((!w->has_last) || after(c->exp_t, w->last_t)) && at_or_after(t, c->exp_t)) {
            return true;
        }
    }
    return false;
}

static void feed_chan(derived_windows_t* a, uint32_t wi, uint32_t ci, uint32_t t_in, bool valid, float v)
{
    derived_win_t* const w = &a->w[wi];
    uint32_t t = t_in;
    if (w->staged && after(w->staged_t, t)) {
        t = w->staged_t; /* never go back in time */
    }
    if (w->staged && (t != w->staged_t)) {
        flush_point(a, wi);
    }
    if (!w->staged) {
        run_expiries(a, wi, t);
        w->staged = true;
        w->staged_t = t;
        w->trig_has = false;
    }
    derived_win_chan_t* const c = &w->ch[ci];
    c->t = t;
    c->v = v;
    c->has = true;
    c->valid = valid;
    c->exp_t = t + c->stale_ms + 1u;
    c->exp_pending = true;
    if (ci == w->trig_ch) {
        w->trig_has = true;
        w->trig_v = (double)v;
    }
}

/* The final-drive estimate of a measured gearbox ratio at time `t` (D-106): measured x
 * secondary_reduction_ratio / gear_ratio_n with the gear_estimate row at or before `t` (not
 * stale, valid, not no_gear). False without such a gear. */
static bool final_drive_estimate(const derived_win_t* w, uint32_t t, float measured, float* est)
{
    bool ok = false;
    for (uint32_t i = 0u; (i < w->nch) && (!ok); i++) {
        const derived_win_chan_t* const c = &w->ch[i];
        double g = 0.0;
        if ((c->kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) && ((uint32_t)c->index == (uint32_t)VEHICLE_CL250_DERIVED_GEAR_ESTIMATE) &&
            ch_latest(c, t, &g) && (g >= 1.0) && (g <= (double)VEHICLE_CL250_GEAR_COUNT)) {
            const float ratio = derived_gear_ratio((uint8_t)g);
            if (ratio > 0.0f) {
                *est = (float)(((double)measured * (double)VEHICLE_CL250_SECONDARY_REDUCTION_RATIO) / (double)ratio);
                ok = true;
            }
        }
    }
    return ok;
}

static void feed_stat(derived_windows_t* a, uint32_t wi, const derived_win_event_t* e)
{
    derived_win_t* const w = &a->w[wi];
    const uint32_t t = e->t;
    float v = e->v;
    if (w->has_first || (!wants_stat_row(wi, e->kind, e->index)) || (!e->valid)) {
        return; /* an invalid derived row is no sample (the reference skips None) */
    }
    if ((e->kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) && (e->index == (uint8_t)VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED) &&
        (!final_drive_estimate(w, t, e->v, &v))) {
        return; /* no valid gear at that time */
    }
    if (w->staged && after(t, w->staged_t)) {
        flush_point(a, wi);
    }
    bool keep = false;
    if (w->staged) {
        keep = (t == w->staged_t);
    } else {
        keep = w->open && (!expiry_due_by(w, t));
    }
    if (keep) {
        if (w->pend_n < DERIVED_WINDOWS_PEND_CAP) {
            w->pend[w->pend_n].t = t;
            w->pend[w->pend_n].v = v;
            w->pend[w->pend_n].kind = e->kind;
            w->pend[w->pend_n].index = e->index;
            w->pend_n++;
        } else {
            w->broken = true;
        }
    }
}

/* ---- events ------------------------------------------------------------------------------------- */

static void push(derived_windows_t* a, uint8_t kind, uint32_t index, bool valid, float value, uint32_t stamp_ms)
{
    bool wanted = false;
    for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
        const derived_win_t* const w = &a->w[wi];
        if (!w->enabled) {
            continue;
        }
        for (uint32_t i = 0u; i < w->nch; i++) {
            wanted = wanted || ((w->ch[i].kind == kind) && ((uint32_t)w->ch[i].index == index));
        }
        wanted = wanted || wants_stat_row(wi, kind, index);
    }
    if (!wanted) {
        return;
    }
    if (a->ev_n >= DERIVED_WINDOWS_EVENT_CAP) {
        for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
            a->w[wi].broken = true;
        }
        return;
    }
    derived_win_event_t* const e = &a->ev[a->ev_n];
    e->t = stamp_ms;
    e->v = value;
    e->kind = kind;
    e->index = (uint8_t)index;
    e->valid = valid;
    a->ev_n++;
}

void derived_windows_push_did(derived_windows_t* a, uint32_t did, float value, uint32_t stamp_ms)
{
    if ((a != NULL) && (did < VEHICLE_CL250_DID_COUNT)) {
        push(a, VEHICLE_CL250_DERIVED_INPUT_DID, did, true, value, stamp_ms);
    }
}

void derived_windows_push_derived(derived_windows_t* a, uint32_t sig, bool valid, float value, uint32_t stamp_ms)
{
    if ((a != NULL) && (sig < VEHICLE_CL250_DERIVED_COUNT)) {
        push(a, VEHICLE_CL250_DERIVED_INPUT_DERIVED, sig, valid, value, stamp_ms);
    }
}

static void feed_channels(derived_windows_t* a, const derived_win_event_t* e)
{
    for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
        if (!a->w[wi].enabled) {
            continue;
        }
        for (uint32_t i = 0u; i < a->w[wi].nch; i++) {
            if ((a->w[wi].ch[i].kind == e->kind) && (a->w[wi].ch[i].index == e->index)) {
                feed_chan(a, wi, i, e->t, e->valid, e->v);
            }
        }
    }
}

void derived_windows_run(derived_windows_t* a)
{
    if (a == NULL) {
        return;
    }
    /* insertion sort by time (a stable order keeps the arrival order of equal times) */
    for (uint32_t i = 1u; i < a->ev_n; i++) {
        const derived_win_event_t key = a->ev[i];
        uint32_t j = i;
        while ((j > 0u) && after(a->ev[j - 1u].t, key.t)) {
            a->ev[j] = a->ev[j - 1u];
            j--;
        }
        a->ev[j] = key;
    }
    uint32_t i = 0u;
    while (i < a->ev_n) {
        uint32_t end = i + 1u;
        while ((end < a->ev_n) && (a->ev[end].t == a->ev[i].t)) {
            end++;
        }
        for (uint32_t k = i; k < end; k++) { /* window inputs of this time first */
            feed_channels(a, &a->ev[k]);
        }
        for (uint32_t k = i; k < end; k++) { /* then the statistics samples */
            for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
                if (a->w[wi].enabled) {
                    feed_stat(a, wi, &a->ev[k]);
                }
            }
        }
        i = end;
    }
    a->ev_n = 0u;
}

void derived_windows_end(derived_windows_t* a)
{
    if (a == NULL) {
        return;
    }
    derived_windows_run(a);
    for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
        derived_win_t* const w = &a->w[wi];
        if (!w->enabled) {
            continue;
        }
        flush_point(a, wi);
        if (w->open) {
            finalize_close(a, wi, w->last_t, CLOSE_SESSION_END);
        }
        w->pend_n = 0u;
    }
}
