/*
 * D-079 health windows (pure). See derived_windows.h for the rules and the reference
 * (vehicle-server derived/health_windows.py, health_window_summaries.py).
 */
#include "features/derived/derived_windows.h"

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

#define HALF_RANGE (0x7FFFFFFFu)
#define MS_PER_S (1000.0)

/* close reasons (the reference's CLOSE_*) */
#define CLOSE_CONDITION (1u)
#define CLOSE_STABLE (2u)
#define CLOSE_MAX_DURATION (3u)
#define CLOSE_ENGINE_ZERO (4u)
#define CLOSE_UNKNOWN (5u)
#define CLOSE_SESSION_END (6u)

_Static_assert(VEHICLE_CL250_DID_COUNT <= 64u, "the DID masks are 64 bits");

/* ---- time helpers (wrap-safe) -------------------------------------------------------------- */

static bool at_or_after(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b) <= HALF_RANGE;
}

static bool after(uint32_t a, uint32_t b)
{
    return (a != b) && at_or_after(a, b);
}

static bool is_transition(uint8_t opens_on)
{
    return (opens_on == VEHICLE_CL250_WINDOW_OPEN_ENGINE_START) || (opens_on == VEHICLE_CL250_WINDOW_OPEN_ENGINE_STOP);
}

/* Window `wi` reads input (kind, index) as a condition, a stable band or its engine trigger. */
static bool window_reads(uint32_t wi, uint8_t kind, uint32_t index)
{
    const vehicle_cl250_window_t* const spec = &vehicle_cl250_window[wi];
    bool r = is_transition(spec->opens_on) && (kind == VEHICLE_CL250_DERIVED_INPUT_DID) &&
             (index == (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED);
    for (uint32_t k = 0u; (k < spec->condition_count) && (k < VEHICLE_CL250_WINDOW_CONDITION_MAX); k++) {
        r = r || ((spec->conditions[k].input.kind == kind) && ((uint32_t)spec->conditions[k].input.index == index));
    }
    for (uint32_t k = 0u; (k < spec->stable_count) && (k < VEHICLE_CL250_WINDOW_STABLE_MAX); k++) {
        r = r || ((spec->stable[k].input.kind == kind) && ((uint32_t)spec->stable[k].input.index == index));
    }
    return r;
}

static bool win_is_zero(double x)
{
    return (x <= 0.0) && (x >= 0.0);
}

/* ---- which DIDs a window's statistics read ------------------------------------------------- */

static bool wants_stat(uint32_t wi, uint32_t did)
{
    bool r = false;
    switch (wi) {
    case VEHICLE_CL250_WINDOW_W1_KEY_ON:
        r = (did == (uint32_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE) || (did == (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP) ||
            (did == (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP) || (did == (uint32_t)VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP) ||
            (did == (uint32_t)VEHICLE_CL250_IDX_INTAKE_MAP) || (did == (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE);
        break;
    case VEHICLE_CL250_WINDOW_W3_WARM_IDLE:
        r = (did == (uint32_t)VEHICLE_CL250_IDX_INTAKE_MAP) || (did == (uint32_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1);
        break;
    case VEHICLE_CL250_WINDOW_W5_POST_STOP:
        r = (did == (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP) || (did == (uint32_t)VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP);
        break;
    default:
        r = false;
        break;
    }
    return r;
}

#define STAT_DID_COUNT (7u)

/* ---- gen/ checks ------------------------------------------------------------------------------ */

uint32_t derived_windows_of_signal(uint32_t sig)
{
    uint32_t r = DERIVED_WINDOWS_NONE;
    if ((sig < VEHICLE_CL250_DERIVED_COUNT) && (vehicle_cl250_derived[sig].derived_class == VEHICLE_CL250_DERIVED_CLASS_RIDE)) {
        for (uint32_t w = 0u; w < DERIVED_WINDOWS_COUNT; w++) {
            if (vehicle_cl250_derived[sig].record_did == vehicle_cl250_window[w].record_did) {
                r = w;
            }
        }
    }
    return r;
}

bool derived_windows_has_summary(uint32_t sig)
{
    bool r = false;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE:
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_SENSOR_SPREAD:
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_MAP_BARO_DELTA:
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_WARM_RESTART:
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_COUNT:
    case VEHICLE_CL250_DERIVED_WINDOW_CRANK_COUNT:
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_SPREAD:
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_DRIFT:
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_MAP_SPREAD:
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_MEAN:
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_STD:
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_COUNT:
    case VEHICLE_CL250_DERIVED_WINDOW_CRUISE_COUNT:
    case VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU:
    case VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START:
    case VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_END:
    case VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COUNT:
        r = true;
        break;
    default:
        r = false;
        break;
    }
    return r;
}

static uint32_t input_stale(uint8_t kind, uint8_t index)
{
    uint32_t s = 0u;
    if (kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
        s = vehicle_cl250_dids[index].stale_after_ms;
    } else if (kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) {
        s = vehicle_cl250_derived[index].stale_after_ms;
    } else {
        s = 0u;
    }
    return s;
}

static float cool_decode(uint32_t raw);
static bool cool_encode(float v, uint16_t* raw);

/* The step event buffer holds at most one sample per distinct wanted DID and one row per
 * distinct derived window input; true when that worst case fits DERIVED_WINDOWS_EVENT_CAP. */
static bool events_fit(void)
{
    uint32_t n = 0u;
    for (uint32_t did = 0u; did < VEHICLE_CL250_DID_COUNT; did++) {
        bool wanted = false;
        for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
            wanted = wanted || wants_stat(wi, did) || window_reads(wi, VEHICLE_CL250_DERIVED_INPUT_DID, did);
        }
        n += wanted ? 1u : 0u;
    }
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        bool wanted = false;
        for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
            wanted = wanted || window_reads(wi, VEHICLE_CL250_DERIVED_INPUT_DERIVED, sig);
        }
        n += wanted ? 1u : 0u;
    }
    return n <= DERIVED_WINDOWS_EVENT_CAP;
}

bool derived_windows_sizing_ok(uint32_t w, uint32_t rpm_hist_cap)
{
    if (w >= DERIVED_WINDOWS_COUNT) {
        return false;
    }
    const vehicle_cl250_window_t* const spec = &vehicle_cl250_window[w];
    uint32_t max_stale = 0u;
    for (uint32_t k = 0u; k < spec->condition_count; k++) {
        const uint32_t s = input_stale(spec->conditions[k].input.kind, spec->conditions[k].input.index);
        max_stale = (s > max_stale) ? s : max_stale;
    }
    for (uint32_t k = 0u; k < spec->stable_count; k++) {
        const uint32_t s = input_stale(spec->stable[k].input.kind, spec->stable[k].input.index);
        max_stale = (s > max_stale) ? s : max_stale;
    }
    if (is_transition(spec->opens_on)) {
        const uint32_t s = input_stale(VEHICLE_CL250_DERIVED_INPUT_DID, (uint8_t)VEHICLE_CL250_IDX_ENGINE_SPEED);
        max_stale = (s > max_stale) ? s : max_stale;
    }
    static const uint8_t stat_dids[STAT_DID_COUNT] = {
        (uint8_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE,       (uint8_t)VEHICLE_CL250_IDX_COOLANT_TEMP,
        (uint8_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP,       (uint8_t)VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP,
        (uint8_t)VEHICLE_CL250_IDX_INTAKE_MAP,            (uint8_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE,
        (uint8_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1,
    };
    uint32_t need = 0u;
    for (uint32_t k = 0u; k < STAT_DID_COUNT; k++) {
        if (wants_stat(w, stat_dids[k])) {
            const uint32_t poll = vehicle_cl250_dids[stat_dids[k]].poll_period_ms;
            if (poll == 0u) {
                return false;
            }
            need += (((max_stale + 1u) + (poll - 1u)) / poll) + 1u;
        }
    }
    bool ok = (need <= DERIVED_WINDOWS_PEND_CAP) && events_fit();
    if (w == VEHICLE_CL250_WINDOW_W3_WARM_IDLE) {
        /* the warm-idle RPM statistics read the core's ENGINE_SPEED history: it must hold every
         * sample of the longest instance */
        const uint32_t poll = vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].poll_period_ms;
        ok = ok && (poll != 0u) && (spec->max_duration_ms != 0u) &&
             (((spec->max_duration_ms / poll) + 2u) <= rpm_hist_cap);
    }
    if (w == VEHICLE_CL250_WINDOW_W5_POST_STOP) {
        const uint32_t poll = vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP].poll_period_ms;
        /* stale_after_ms is a uint16 in gen/, so a gap that keeps a coolant sample fresh fits the 16-bit delta */
        const vehicle_cl250_did_t* const d = &vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP];
        ok = ok && (poll != 0u) && (((spec->max_duration_ms / poll) + 2u) <= DERIVED_WIN_POST_STOP_COOLANT_CAP) &&
             (d->length <= 2u) && (d->factor_num != 0) && (d->factor_den != 0);
        /* the stored raw integer must rebuild every decodable value exactly */
        const uint32_t raws = (d->length == 2u) ? 65536u : 256u;
        for (uint32_t r = 0u; (r < raws) && ok; r++) {
            uint16_t back = 0u;
            ok = cool_encode(cool_decode(r), &back) && ((uint32_t)back == r);
        }
    }
    return ok;
}

uint32_t derived_windows_derived_inputs(uint32_t w, uint8_t out[DERIVED_WINDOWS_CH_MAX])
{
    uint32_t n = 0u;
    if (w >= DERIVED_WINDOWS_COUNT) {
        return DERIVED_WINDOWS_CH_MAX + 1u;
    }
    const vehicle_cl250_window_t* const spec = &vehicle_cl250_window[w];
    for (uint32_t k = 0u; k < (spec->condition_count + spec->stable_count); k++) {
        const vehicle_cl250_derived_input_t in = (k < spec->condition_count)
                                                     ? spec->conditions[k].input
                                                     : spec->stable[k - spec->condition_count].input;
        if (in.kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) {
            out[n] = in.index;
            n++;
        } else if (in.kind != VEHICLE_CL250_DERIVED_INPUT_DID) {
            return DERIVED_WINDOWS_CH_MAX + 1u;
        } else {
            /* a DID is always available */
        }
    }
    return n;
}

/* ---- wiring --------------------------------------------------------------------------------- */

static uint8_t add_channel(derived_win_t* w, uint8_t kind, uint8_t index)
{
    uint8_t found = DERIVED_WINDOWS_NONE;
    for (uint8_t i = 0u; i < w->nch; i++) {
        if ((w->ch[i].kind == kind) && (w->ch[i].index == index)) {
            found = i;
        }
    }
    if ((found == DERIVED_WINDOWS_NONE) && (w->nch < DERIVED_WINDOWS_CH_MAX)) {
        found = w->nch;
        w->ch[found].kind = kind;
        w->ch[found].index = index;
        w->ch[found].stale_ms = input_stale(kind, index);
        w->nch++;
    }
    return found;
}

static void wire(derived_win_t* w, const vehicle_cl250_window_t* spec)
{
    for (uint32_t k = 0u; k < spec->condition_count; k++) {
        w->cond_ch[k] = add_channel(w, spec->conditions[k].input.kind, spec->conditions[k].input.index);
    }
    for (uint32_t k = 0u; k < spec->stable_count; k++) {
        w->stable_ch[k] = add_channel(w, spec->stable[k].input.kind, spec->stable[k].input.index);
    }
    if (is_transition(spec->opens_on)) {
        w->trig_ch = add_channel(w, VEHICLE_CL250_DERIVED_INPUT_DID, (uint8_t)VEHICLE_CL250_IDX_ENGINE_SPEED);
    }
}

void derived_windows_begin(derived_windows_t* a, uint32_t enabled_mask, const derived_sample_t* rpm_data,
                           const derived_ring_t* rpm_ring, uint32_t rpm_cap)
{
    if (a == NULL) {
        return;
    }
    (void)memset(a, 0, sizeof *a);
    a->rpm_data = rpm_data;
    a->rpm_ring = rpm_ring;
    a->rpm_cap = rpm_cap;
    for (uint32_t i = 0u; i < DERIVED_WINDOWS_COUNT; i++) {
        derived_win_t* const w = &a->w[i];
        w->trig_ch = DERIVED_WINDOWS_NONE;
        w->enabled = ((enabled_mask >> i) & 1u) != 0u;
        w->can_open = true;
        if (w->enabled) {
            wire(w, &vehicle_cl250_window[i]);
        }
    }
}

/* ---- reading a window input ------------------------------------------------------------------ */

static bool ch_latest(const derived_win_chan_t* c, uint32_t t, double* v)
{
    bool ok = c->has && c->valid;
    if (ok) {
        const uint32_t age = at_or_after(t, c->t) ? (t - c->t) : 0u;
        ok = age <= c->stale_ms;
    }
    if (ok) {
        *v = (double)c->v;
    }
    return ok;
}

static bool compare(double v, uint8_t op, double expected)
{
    bool r = false;
    switch (op) {
    case VEHICLE_CL250_WINDOW_OP_LT:
        r = v < expected;
        break;
    case VEHICLE_CL250_WINDOW_OP_LE:
        r = v <= expected;
        break;
    case VEHICLE_CL250_WINDOW_OP_GT:
        r = v > expected;
        break;
    case VEHICLE_CL250_WINDOW_OP_GE:
        r = v >= expected;
        break;
    default: /* VEHICLE_CL250_WINDOW_OP_EQ */
        r = (v <= expected) && (v >= expected);
        break;
    }
    return r;
}

static bool conds_hold(const derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t)
{
    for (uint32_t k = 0u; k < spec->condition_count; k++) {
        double v = 0.0;
        if ((!ch_latest(&w->ch[w->cond_ch[k]], t, &v)) || (!compare(v, spec->conditions[k].op, (double)spec->conditions[k].value))) {
            return false;
        }
    }
    return true;
}

static bool can_open_at(const derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t)
{
    bool ok = conds_hold(w, spec, t);
    for (uint32_t k = 0u; (k < spec->stable_count) && ok; k++) {
        double v = 0.0;
        ok = ch_latest(&w->ch[w->stable_ch[k]], t, &v);
    }
    return ok;
}

/* The stable band check of the open instance; the close reason or 0. */
static uint8_t stable_close(derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t)
{
    for (uint32_t k = 0u; k < spec->stable_count; k++) {
        double v = 0.0;
        if (!ch_latest(&w->ch[w->stable_ch[k]], t, &v)) {
            return CLOSE_CONDITION;
        }
        w->lo[k] = (v < w->lo[k]) ? v : w->lo[k];
        w->hi[k] = (v > w->hi[k]) ? v : w->hi[k];
        if ((w->hi[k] - w->lo[k]) > (double)spec->stable[k].max_spread) {
            return CLOSE_STABLE;
        }
    }
    return 0u;
}

/* ---- the statistics ----------------------------------------------------------------------------- */

static void add_mean(derived_win_stats_t* st, uint32_t slot, double v)
{
    st->mean[slot].sum += v;
    st->mean[slot].n++;
}

/* The coolant DID's physical value from its raw integer: vehicle_cl250_decode() verbatim. */
static float cool_decode(uint32_t raw)
{
    const vehicle_cl250_did_t* const d = &vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP];
    float value = ((float)raw * (float)d->factor_num) / (float)d->factor_den;
    value += (float)d->offset;
    return value;
}

/* The raw integer of a coolant value, false when it is not exactly a decoded raw. */
static bool cool_encode(float v, uint16_t* raw)
{
    const vehicle_cl250_did_t* const d = &vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP];
    const double r = floor((((double)v - (double)d->offset) * (double)d->factor_den) / (double)d->factor_num + 0.5);
    bool ok = (r >= 0.0) && (r <= 65535.0);
    if (ok) {
        *raw = (uint16_t)r;
        ok = cool_decode((uint32_t)*raw) == v;
    }
    return ok;
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
    switch (it->did) {
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
static void compute_rpm(const derived_windows_t* a, derived_win_stats_t* st, uint32_t open_ms, uint32_t close_ms,
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

static void feed_stat(derived_windows_t* a, uint32_t wi, uint32_t did, uint32_t t, float v)
{
    derived_win_t* const w = &a->w[wi];
    if (w->has_first || (!wants_stat(wi, did))) {
        return;
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
            w->pend[w->pend_n].did = (uint8_t)did;
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
        wanted = wanted || ((kind == VEHICLE_CL250_DERIVED_INPUT_DID) && wants_stat(wi, index));
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
            if (a->ev[k].kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
                for (uint32_t wi = 0u; wi < DERIVED_WINDOWS_COUNT; wi++) {
                    if (a->w[wi].enabled) {
                        feed_stat(a, wi, a->ev[k].index, a->ev[k].t, a->ev[k].v);
                    }
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

/* ---- the summaries ---------------------------------------------------------------------------- */

typedef struct {
    derived_win_stats_t st;
    uint32_t open_ms;
    uint32_t count;
    bool have;
} instance_t;

/* The first qualifying instance, or the open one as if the ride ended at the last point. */
static void pick_instance(const derived_windows_t* a, uint32_t wi, instance_t* out)
{
    const derived_win_t* const w = &a->w[wi];
    instance_t r;
    (void)memset(&r, 0, sizeof r);
    r.count = w->count;
    const bool peek = w->open && w->has_last && ((w->last_t - w->open_ms) >= vehicle_cl250_window[wi].min_duration_ms);
    if (peek && (r.count < UINT32_MAX)) {
        r.count++;
    }
    if (w->has_first) {
        r.st = w->st;
        r.open_ms = w->first_open_ms;
        r.have = true;
    } else if (peek) {
        r.st = w->st;
        r.open_ms = w->open_ms;
        r.have = true;
        if (wi == VEHICLE_CL250_WINDOW_W3_WARM_IDLE) {
            compute_rpm(a, &r.st, w->open_ms, w->last_t, CLOSE_SESSION_END);
        }
    } else {
        r.have = false;
    }
    *out = r;
}

static derived_flag_t mean_of(const derived_win_stats_t* st, uint32_t slot, double* out)
{
    if (st->mean[slot].n == 0u) {
        return DERIVED_FLAG_NO_DATA;
    }
    *out = st->mean[slot].sum / (double)st->mean[slot].n;
    return DERIVED_FLAG_NONE;
}

/* Newton cooling time constant of the post-stop coolant (health_window_summaries.py). */
static derived_flag_t tau_of(const derived_windows_t* a, const instance_t* in, double* out)
{
    const derived_win_stats_t* const st = &in->st;
    double ambient = 0.0;
    derived_flag_t f = mean_of(st, DERIVED_WIN_MEAN_AMBIENT, &ambient);
    if (f != DERIVED_FLAG_NONE) {
        return f;
    }
    uint32_t n = 0u;
    double sx = 0.0;
    double sy = 0.0;
    uint32_t t = st->coolant_t0;
    for (uint32_t i = 0u; i < st->coolant_n; i++) {
        t += a->coolant_buf[i].dt_ms;
        const double d = (double)cool_decode(a->coolant_buf[i].raw) - ambient;
        if (d > 0.0) {
            const uint32_t rel = t - in->open_ms;
            sx += (double)rel / MS_PER_S;
            sy += log(d);
            n++;
        }
    }
    if (n < 3u) {
        return DERIVED_FLAG_INSUFFICIENT_SAMPLES;
    }
    const double mx = sx / (double)n;
    const double my = sy / (double)n;
    double var = 0.0;
    double cov = 0.0;
    t = st->coolant_t0;
    for (uint32_t i = 0u; i < st->coolant_n; i++) {
        t += a->coolant_buf[i].dt_ms;
        const double d = (double)cool_decode(a->coolant_buf[i].raw) - ambient;
        if (d > 0.0) {
            const uint32_t rel = t - in->open_ms;
            const double x = (double)rel / MS_PER_S;
            var += (x - mx) * (x - mx);
            cov += (x - mx) * (log(d) - my);
        }
    }
    if (win_is_zero(var)) {
        return DERIVED_FLAG_INSUFFICIENT_SAMPLES;
    }
    const double slope = cov / var;
    if (slope >= 0.0) {
        return DERIVED_FLAG_NO_DATA;
    }
    *out = -1.0 / slope;
    return DERIVED_FLAG_NONE;
}

static derived_flag_t value_of(const derived_windows_t* a, uint32_t sig, const instance_t* in, double* out)
{
    const derived_win_stats_t* const st = &in->st;
    derived_flag_t f = DERIVED_FLAG_NONE;
    double m1 = 0.0;
    double m2 = 0.0;
    double m3 = 0.0;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE:
        f = mean_of(st, DERIVED_WIN_MEAN_BATTERY, out);
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_SENSOR_SPREAD:
        f = mean_of(st, DERIVED_WIN_MEAN_COOLANT, &m1);
        if (f == DERIVED_FLAG_NONE) {
            f = mean_of(st, DERIVED_WIN_MEAN_INTAKE, &m2);
        }
        if (f == DERIVED_FLAG_NONE) {
            f = mean_of(st, DERIVED_WIN_MEAN_AMBIENT, &m3);
        }
        if (f == DERIVED_FLAG_NONE) {
            const double hi = (m1 > m2) ? ((m1 > m3) ? m1 : m3) : ((m2 > m3) ? m2 : m3);
            const double lo = (m1 < m2) ? ((m1 < m3) ? m1 : m3) : ((m2 < m3) ? m2 : m3);
            *out = hi - lo;
        }
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_MAP_BARO_DELTA:
        f = mean_of(st, DERIVED_WIN_MEAN_MAP, &m1);
        if (f == DERIVED_FLAG_NONE) {
            f = mean_of(st, DERIVED_WIN_MEAN_BARO, &m2);
        }
        if (f == DERIVED_FLAG_NONE) {
            *out = m1 - m2;
        }
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_WARM_RESTART:
        f = mean_of(st, DERIVED_WIN_MEAN_COOLANT, &m1);
        if (f == DERIVED_FLAG_NONE) {
            f = mean_of(st, DERIVED_WIN_MEAN_AMBIENT, &m2);
        }
        if (f == DERIVED_FLAG_NONE) {
            *out = (fabs(m1 - m2) > (double)VEHICLE_CL250_WINDOW_COLD_SOAK_MAX_DELTA_DEGC) ? 1.0 : 0.0;
        }
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_SPREAD:
        f = st->rpm_spread_ok ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        *out = st->rpm_spread;
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_DRIFT:
        f = st->rpm_drift_ok ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        *out = st->rpm_drift;
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_MAP_SPREAD:
        f = st->has_map ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        *out = (double)st->map_max - (double)st->map_min;
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_MEAN:
        f = (st->stft_n > 0u) ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        *out = st->stft_mean;
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_STD:
        f = (st->stft_n >= 2u) ? DERIVED_FLAG_NONE : DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        if (st->stft_n >= 2u) {
            const uint32_t dof = st->stft_n - 1u;
            *out = sqrt(st->stft_m2 / (double)dof);
        }
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START:
        f = st->has_coolant ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        *out = (double)st->coolant_first;
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_END:
        f = st->has_coolant ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        *out = (double)st->coolant_last;
        break;
    default: /* VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU */
        f = tau_of(a, in, out);
        break;
    }
    return f;
}

static bool is_count(uint32_t sig)
{
    return (sig == VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_COUNT) || (sig == VEHICLE_CL250_DERIVED_WINDOW_CRANK_COUNT) ||
           (sig == VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_COUNT) || (sig == VEHICLE_CL250_DERIVED_WINDOW_CRUISE_COUNT) ||
           (sig == VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COUNT);
}

derived_flag_t derived_windows_value(const derived_windows_t* a, uint32_t sig, double* out)
{
    const uint32_t wi = derived_windows_of_signal(sig);
    if ((a == NULL) || (out == NULL) || (wi >= DERIVED_WINDOWS_COUNT) || (!derived_windows_has_summary(sig)) ||
        (!a->w[wi].enabled) || a->w[wi].broken) {
        return DERIVED_FLAG_MATH_ERROR;
    }
    instance_t in;
    pick_instance(a, wi, &in);
    if (is_count(sig)) {
        *out = (double)in.count;
        return DERIVED_FLAG_NONE;
    }
    if (!in.have) {
        return DERIVED_FLAG_NO_WINDOW;
    }
    return value_of(a, sig, &in, out);
}
