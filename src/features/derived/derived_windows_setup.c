/*
 * D-079 health windows (pure), configuration: which windows exist, sizing, channel wiring, condition checks. See derived_windows.h for the rules and the reference
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
#error "derived_windows_setup.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_windows_setup.c needs -fno-math-errno"
#endif

#define HALF_RANGE (0x7FFFFFFFu)

_Static_assert(VEHICLE_CL250_DID_COUNT <= 64u, "the DID masks are 64 bits");

/* ---- time helpers (wrap-safe) -------------------------------------------------------------- */

bool at_or_after(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b) <= HALF_RANGE;
}

bool is_transition(uint8_t opens_on)
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

bool win_is_zero(double x)
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

/* The derived rows whose samples inside a window feed a statistic (D-106, W4 only): the measured
 * gearbox ratio (final-drive residual, with the gear at its time) and the volumetric efficiency. */
static bool wants_stat_derived(uint32_t wi, uint32_t sig)
{
    return (wi == (uint32_t)VEHICLE_CL250_WINDOW_W4_CRUISE) &&
           ((sig == (uint32_t)VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED) ||
            (sig == (uint32_t)VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY));
}

bool wants_stat_row(uint32_t wi, uint8_t kind, uint32_t index)
{
    return (kind == VEHICLE_CL250_DERIVED_INPUT_DID) ? wants_stat(wi, index) : wants_stat_derived(wi, index);
}

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
    case VEHICLE_CL250_DERIVED_WINDOW_CRUISE_FINAL_DRIVE_RESIDUAL:
    case VEHICLE_CL250_DERIVED_WINDOW_CRUISE_VE_RESIDUAL:
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
            wanted = wanted || window_reads(wi, VEHICLE_CL250_DERIVED_INPUT_DERIVED, sig) || wants_stat_derived(wi, sig);
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
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        if (wants_stat_derived(w, sig)) {
            const uint32_t poll = vehicle_cl250_derived[sig].period_ms;
            if (poll == 0u) {
                return false;
            }
            need += (((max_stale + 1u) + (poll - 1u)) / poll) + 1u;
        }
    }
    bool ok = (need <= DERIVED_WINDOWS_PEND_CAP) && events_fit();
    if (w == VEHICLE_CL250_WINDOW_W4_CRUISE) {
        /* the final-drive median keeps every ratio row of the longest instance, and needs the gear
         * estimate as a window channel (read at the time of each ratio row) */
        const uint32_t poll = vehicle_cl250_derived[VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED].period_ms;
        bool gear_ch = false;
        for (uint32_t k = 0u; k < spec->condition_count; k++) {
            gear_ch = gear_ch || ((spec->conditions[k].input.kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) &&
                                  ((uint32_t)spec->conditions[k].input.index == (uint32_t)VEHICLE_CL250_DERIVED_GEAR_ESTIMATE));
        }
        ok = ok && gear_ch && (poll != 0u) && (spec->max_duration_ms != 0u) &&
             (((spec->max_duration_ms / poll) + 2u) <= DERIVED_WIN_CRUISE_RATIO_CAP);
    }
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

bool ch_latest(const derived_win_chan_t* c, uint32_t t, double* v)
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

bool conds_hold(const derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t)
{
    for (uint32_t k = 0u; k < spec->condition_count; k++) {
        double v = 0.0;
        if ((!ch_latest(&w->ch[w->cond_ch[k]], t, &v)) || (!compare(v, spec->conditions[k].op, (double)spec->conditions[k].value))) {
            return false;
        }
    }
    return true;
}

bool can_open_at(const derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t)
{
    bool ok = conds_hold(w, spec, t);
    for (uint32_t k = 0u; (k < spec->stable_count) && ok; k++) {
        double v = 0.0;
        ok = ch_latest(&w->ch[w->stable_ch[k]], t, &v);
    }
    return ok;
}

/* The stable band check of the open instance; the close reason or 0. */
uint8_t stable_close(derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t)
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

void add_mean(derived_win_stats_t* st, uint32_t slot, double v)
{
    st->mean[slot].sum += v;
    st->mean[slot].n++;
}

/* The coolant DID's physical value from its raw integer: vehicle_cl250_decode() verbatim. */
float cool_decode(uint32_t raw)
{
    const vehicle_cl250_did_t* const d = &vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP];
    float value = ((float)raw * (float)d->factor_num) / (float)d->factor_den;
    value += (float)d->offset;
    return value;
}

/* The raw integer of a coolant value, false when it is not exactly a decoded raw. */
bool cool_encode(float v, uint16_t* raw)
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
