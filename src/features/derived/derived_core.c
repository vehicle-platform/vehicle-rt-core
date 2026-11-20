/*
 * Pure derived-signal core (D-070). See derived_core.h for the sampling rule and
 * features/derived/README.md for the rationale. The formulas follow the Python reference
 * (vehicle-server src/vehicle_server/derived/formulas.py), input by input in the same order, so a
 * flag is the one the reference gives when more than one input is unusable.
 * 
 * This file keeps the time helpers, the sample rings, the input access, the integrals, the output
 * store and the accessors; derived_formulas.c, derived_step.c and derived_init.c hold the rest.
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
#error "derived_core.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_core.c needs -fno-math-errno"
#endif

#define HALF_RANGE (0x7FFFFFFFu)

_Static_assert(VEHICLE_CL250_DERIVED_COUNT <= 255u, "signal indices are small");

/* ---- time helpers (wrap-safe) -------------------------------------------------------- */

/* Age of a stamp: a stamp ahead of now (half range or more) is age 0, never a wrapped age. */
uint32_t derived_core_age_of(uint32_t now, uint32_t stamp)
{
    const uint32_t a = now - stamp;
    return (a > HALF_RANGE) ? 0u : a;
}

/* True when a is at or after b. */
bool derived_core_at_or_after(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b) <= HALF_RANGE;
}

uint32_t later(uint32_t a, uint32_t b)
{
    return derived_core_at_or_after(a, b) ? a : b;
}

bool is_zero(float x)
{
    return (x <= 0.0f) && (x >= 0.0f);
}

bool derived_core_is_finite_f(float x)
{
    return isfinite(x) != 0;
}

/* ---- which signal has which formula ---------------------------------------------------- */

uint32_t kind_of(uint32_t sig)
{
    uint32_t kind = KIND_POINT;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_TRIP_DISTANCE:
    case VEHICLE_CL250_DERIVED_TRIP_TIME:
    case VEHICLE_CL250_DERIVED_IDLE_TIME:
        kind = KIND_INTEGRAL;
        break;
    case VEHICLE_CL250_DERIVED_STOP_EVENT:
    case VEHICLE_CL250_DERIVED_SHIFT_EVENT:
    case VEHICLE_CL250_DERIVED_LAUNCH_EVENT:
    case VEHICLE_CL250_DERIVED_BRAKING_EVENT:
    case VEHICLE_CL250_DERIVED_CRANKING_DIP:
    case VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE:
    case VEHICLE_CL250_DERIVED_REFUEL_EVENT:
        kind = KIND_EVENT;
        break;
    case VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED:
    case VEHICLE_CL250_DERIVED_GEAR_ESTIMATE:
    case VEHICLE_CL250_DERIVED_VEHICLE_ACCEL:
    case VEHICLE_CL250_DERIVED_VEHICLE_JERK:
    case VEHICLE_CL250_DERIVED_ENGINE_TORQUE:
    case VEHICLE_CL250_DERIVED_DEMAND_TORQUE:
    case VEHICLE_CL250_DERIVED_TORQUE_GAP:
    case VEHICLE_CL250_DERIVED_ENGINE_POWER:
    case VEHICLE_CL250_DERIVED_AIR_DENSITY:
    case VEHICLE_CL250_DERIVED_POWER_CORRECTION_FACTOR:
    case VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO:
    case VEHICLE_CL250_DERIVED_FUEL_RATE_MAF:
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_INSTANT:
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG:
    case VEHICLE_CL250_DERIVED_FUEL_RANGE:
    case VEHICLE_CL250_DERIVED_BSFC:
    case VEHICLE_CL250_DERIVED_FUEL_PER_STROKE:
    case VEHICLE_CL250_DERIVED_TOTAL_FUEL_TRIM:
    case VEHICLE_CL250_DERIVED_THROTTLE_CHANNEL_RESIDUAL:
    case VEHICLE_CL250_DERIVED_THROTTLE_COMMAND_RESIDUAL:
    case VEHICLE_CL250_DERIVED_GRIP_CHANNEL_RESIDUAL:
    case VEHICLE_CL250_DERIVED_ALTITUDE_CHANGE:
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
    case VEHICLE_CL250_DERIVED_OIL_TEMP_RATE:
    case VEHICLE_CL250_DERIVED_CLUTCH_STATE:
    case VEHICLE_CL250_DERIVED_CLUTCH_SLIP_RATIO:
    case VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE:
    case VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY:
    case VEHICLE_CL250_DERIVED_INTAKE_FLOW_RESIDUAL:
    case VEHICLE_CL250_DERIVED_IDLE_MAP_RESIDUAL:
    case VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA:
    case VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE:
    case VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE:
    case VEHICLE_CL250_DERIVED_OVER_REV:
    case VEHICLE_CL250_DERIVED_LUGGING:
    case VEHICLE_CL250_DERIVED_IDLE_RPM_STD:
    case VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT:
    case VEHICLE_CL250_DERIVED_ENGINE_BRAKING_PHASE:
    case VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED:
    case VEHICLE_CL250_DERIVED_DATA_QUALITY_MASK:
        kind = KIND_POINT;
        break;
    default:
        /* a class-B ride summary, or no formula yet */
        kind = (derived_ride_slot(sig) != DERIVED_RIDE_NO_SLOT) ? KIND_RIDE : KIND_NONE;
        break;
    }
    return kind;
}

/* ---- sample rings ---------------------------------------------------------------------- */

derived_sample_t* derived_core_ring_data(derived_core_t* c, uint32_t id, uint32_t* cap)
{
    derived_sample_t* d = NULL;
    switch (id) {
    case RING_SPEED:
        d = c->speed_win;
        *cap = DERIVED_WIN_SPEED_CAP;
        break;
    case RING_FUEL_RATE:
        d = c->fuel_rate_win;
        *cap = DERIVED_WIN_FUEL_RATE_CAP;
        break;
    case RING_COOLANT:
        d = c->coolant_win;
        *cap = DERIVED_WIN_COOLANT_CAP;
        break;
    case RING_OIL:
        d = c->oil_win;
        *cap = DERIVED_WIN_OIL_CAP;
        break;
    case RING_ACCEL:
        d = c->accel_win;
        *cap = DERIVED_WIN_ACCEL_CAP;
        break;
    case RING_RPM_HIST:
        d = c->rpm_hist;
        *cap = DERIVED_WIN_RPM_HIST_CAP;
        break;
    case RING_BATT_HIST:
        d = c->batt_hist;
        *cap = DERIVED_WIN_BATT_HIST_CAP;
        break;
    case RING_FUEL_HIST:
        d = c->fuel_hist;
        *cap = DERIVED_WIN_FUEL_HIST_CAP;
        break;
    default:
        d = c->accel_hist;
        *cap = DERIVED_WIN_ACCEL_HIST_CAP;
        break;
    }
    return d;
}

void derived_core_ring_push(derived_core_t* c, uint32_t id, float value, uint32_t stamp)
{
    uint32_t cap = 0u;
    derived_sample_t* const d = derived_core_ring_data(c, id, &cap);
    derived_ring_t* const r = &c->ring[id];
    r->head = (uint16_t)(((uint32_t)r->head + 1u) % cap);
    d[r->head].value = value;
    d[r->head].stamp_ms = stamp;
    if ((uint32_t)r->count < cap) {
        r->count++;
    }
}

uint32_t ring_of_did(uint32_t did)
{
    uint32_t id = RING_NONE;
    switch (did) {
    case (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED:
        id = RING_SPEED;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE:
        id = RING_FUEL_RATE;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP:
        id = RING_COOLANT;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_OIL_TEMP:
        id = RING_OIL;
        break;
    default:
        id = RING_NONE;
        break;
    }
    return id;
}

/*
 * Samples of ring `id` in (now - window_ms, now], newest first, each times `scale`: their
 * count, mean and least-squares slope per second. The loop is bounded by the ring size.
 */
void win_collect(derived_core_t* c, uint32_t id, uint32_t now, uint32_t window_ms, float scale,
                  bool want_slope, win_t* w)
{
    uint32_t cap = 0u;
    const derived_sample_t* const d = derived_core_ring_data(c, id, &cap);
    const derived_ring_t* const r = &c->ring[id];
    float sum = 0.0f;
    uint32_t n = 0u;
    for (uint32_t k = 0u; k < (uint32_t)r->count; k++) {
        const derived_sample_t* const s = &d[((uint32_t)r->head + cap - k) % cap];
        if (derived_core_age_of(now, s->stamp_ms) >= window_ms) {
            break; /* older ones are older still */
        }
        sum += s->value * scale;
        n++;
    }
    w->n = n;
    w->mean = (n > 0u) ? (sum / (float)n) : 0.0f;
    w->slope_per_s = 0.0f;
    w->slope_ok = false;
    if (want_slope && (n >= DERIVED_MIN_WINDOW_SAMPLES)) {
        float mean_x = 0.0f;
        for (uint32_t k = 0u; k < n; k++) {
            mean_x -= (float)derived_core_age_of(now, d[((uint32_t)r->head + cap - k) % cap].stamp_ms);
        }
        mean_x /= (float)n;
        float var = 0.0f;
        float cov = 0.0f;
        for (uint32_t k = 0u; k < n; k++) {
            const derived_sample_t* const s = &d[((uint32_t)r->head + cap - k) % cap];
            const float dx = -(float)derived_core_age_of(now, s->stamp_ms) - mean_x;
            var += dx * dx;
            cov += dx * ((s->value * scale) - w->mean);
        }
        if (var > 0.0f) {
            w->slope_per_s = (cov / var) * DERIVED_MS_PER_S;
            w->slope_ok = true;
        }
    }
}

/* ---- input access (fresh, declared) ---------------------------------------------------- */

static bool declares(const vehicle_cl250_derived_t* spec, uint8_t kind, uint32_t index)
{
    bool found = false;
    for (uint32_t i = 0u; (i < (uint32_t)spec->input_count) && (i < VEHICLE_CL250_DERIVED_INPUT_MAX); i++) {
        if ((spec->inputs[i].kind == kind) && ((uint32_t)spec->inputs[i].index == index)) {
            found = true;
        }
    }
    return found;
}

/* A DID sample is usable at `now` within `limit` ms. */
derived_flag_t did_fresh(const derived_core_t* c, uint32_t did, uint32_t now, uint32_t limit,
                         float* v)
{
    derived_flag_t f = DERIVED_FLAG_NONE;
    if (did >= VEHICLE_CL250_DID_COUNT) {
        f = DERIVED_FLAG_MATH_ERROR;
    } else {
        const derived_did_t* const d = &c->did[did];
        if ((!d->present) || d->lapsed || (derived_core_age_of(now, d->stamp_ms) > limit)) {
            f = DERIVED_FLAG_STALE_INPUT;
        } else if (!derived_core_is_finite_f(d->value)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else {
            *v = d->value;
        }
    }
    return f;
}

/* A declared DID input, with that DID's own stale_after_ms from the gen/ DID table (D-070). */
derived_flag_t derived_core_did_value(const derived_ctx_t* x, uint32_t did, float* v)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR; /* an undeclared input is a bug: fail closed */
    if (declares(x->spec, VEHICLE_CL250_DERIVED_INPUT_DID, did)) {
        f = did_fresh(x->c, did, x->now, (uint32_t)vehicle_cl250_dids[did].stale_after_ms, v);
    }
    return f;
}

/* A declared derived input: fresh by the producer's stale_after_ms, then valid. */
derived_flag_t derived_core_derived_value(const derived_ctx_t* x, uint32_t sig, float* v)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR;
    if ((sig < VEHICLE_CL250_DERIVED_COUNT) && declares(x->spec, VEHICLE_CL250_DERIVED_INPUT_DERIVED, sig)) {
        const derived_out_t* const o = &x->c->out[sig];
        if (((o->status != DERIVED_VALID) && (o->status != DERIVED_INVALID)) ||
            (derived_core_age_of(x->now, o->stamp_ms) > (uint32_t)vehicle_cl250_derived[sig].stale_after_ms)) {
            f = DERIVED_FLAG_STALE_INPUT;
        } else if (o->status == DERIVED_INVALID) {
            f = DERIVED_FLAG_INVALID_INPUT;
        } else {
            f = DERIVED_FLAG_NONE;
            *v = o->value;
        }
    }
    return f;
}

/* The window of a DID input: the newest sample must be fresh. */
derived_flag_t did_window(const derived_ctx_t* x, uint32_t did, float scale, bool want_slope, win_t* w)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_core_did_value(x, did, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        const uint32_t id = ring_of_did(did);
        if ((id == RING_NONE) || (x->spec->window_ms == 0u)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else {
            win_collect(x->c, id, x->now, (uint32_t)x->spec->window_ms, scale, want_slope, w);
        }
    }
    return f;
}

/* ---- integrals (trip distance, trip time, idle time) -------------------------------------- */

static uint32_t integral_slot(uint32_t sig)
{
    uint32_t slot = 2u;
    if (sig == VEHICLE_CL250_DERIVED_TRIP_DISTANCE) {
        slot = 0u;
    } else if (sig == VEHICLE_CL250_DERIVED_TRIP_TIME) {
        slot = 1u;
    } else {
        slot = 2u;
    }
    return slot;
}

/* The DIDs an integral reads, in the reference's order; returns their count. */
static uint32_t integral_inputs(uint32_t sig, uint32_t* ids)
{
    uint32_t n = 1u;
    if (sig == VEHICLE_CL250_DERIVED_TRIP_DISTANCE) {
        ids[0] = (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED;
    } else if (sig == VEHICLE_CL250_DERIVED_TRIP_TIME) {
        ids[0] = (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED;
    } else {
        ids[0] = (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED;
        ids[1] = (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED;
        n = 2u;
    }
    return n;
}

/* The rate (unit per second) held from a breakpoint; v[] in the order of integral_inputs. */
static double integral_rate(uint32_t sig, const float* v)
{
    double rate = 0.0;
    if (sig == VEHICLE_CL250_DERIVED_TRIP_DISTANCE) {
        rate = (double)(v[0] / DERIVED_SECONDS_PER_HOUR);
    } else if (sig == VEHICLE_CL250_DERIVED_TRIP_TIME) {
        rate = (v[0] > 0.0f) ? 1.0 : 0.0;
    } else {
        rate = (is_zero(v[0]) && (v[1] > 0.0f)) ? 1.0 : 0.0;
    }
    return rate;
}

/*
 * A new sample of any input is a breakpoint b (the newest new stamp of this step). The
 * segment held since the last usable breakpoint ends there, at most as long as the
 * shortest remaining freshness of its inputs (a gap adds nothing); a new segment starts
 * when every input is fresh at b. The reference does the same over the whole session.
 */
void integral_update(derived_core_t* c, uint32_t sig)
{
    uint32_t ids[2] = {0u, 0u};
    const uint32_t n = integral_inputs(sig, ids);
    derived_integral_t* const g = &c->integral[integral_slot(sig)];
    bool any = false;
    uint32_t b = 0u;
    for (uint32_t k = 0u; k < n; k++) {
        const derived_did_t* const d = &c->did[ids[k]];
        if (d->fresh_now) {
            b = any ? later(b, d->stamp_ms) : d->stamp_ms;
            any = true;
        }
    }
    if (any) {
        c->evals_step++;
        if (g->seg_active) {
            const uint32_t run = (uint32_t)(b - g->seg_start_ms);
            int32_t span = (run > HALF_RANGE) ? 0 : (int32_t)run;
            if (span > g->seg_remaining_ms) {
                span = g->seg_remaining_ms;
            }
            if (span < 0) {
                span = 0;
            }
            g->prefix += g->rate * ((double)span / 1000.0);
            g->seg_active = false;
        }
        float v[2] = {0.0f, 0.0f};
        bool usable = true;
        int32_t remaining = 0;
        for (uint32_t k = 0u; k < n; k++) {
            float one = 0.0f;
            const uint32_t limit_ms = (uint32_t)vehicle_cl250_dids[ids[k]].stale_after_ms;
            if (did_fresh(c, ids[k], b, limit_ms, &one) != DERIVED_FLAG_NONE) {
                usable = false;
            } else {
                v[k] = one;
                const int32_t left = (int32_t)limit_ms - (int32_t)derived_core_age_of(b, c->did[ids[k]].stamp_ms);
                remaining = (k == 0u) ? left : ((left < remaining) ? left : remaining);
            }
        }
        if (usable) {
            g->rate = integral_rate(sig, v);
            g->seg_start_ms = b;
            g->seg_remaining_ms = remaining;
            g->seg_active = true;
        }
    }
}

derived_flag_t eval_integral(const derived_ctx_t* x, float* out)
{
    uint32_t ids[2] = {0u, 0u};
    const uint32_t n = integral_inputs(x->sig, ids);
    const derived_integral_t* const g = &x->c->integral[integral_slot(x->sig)];
    derived_flag_t f = DERIVED_FLAG_NONE;
    for (uint32_t k = 0u; k < n; k++) {
        float dummy = 0.0f;
        if (f == DERIVED_FLAG_NONE) {
            f = derived_core_did_value(x, ids[k], &dummy);
        }
    }
    if (f == DERIVED_FLAG_NONE) {
        double total = g->prefix;
        if (g->seg_active) {
            const uint32_t run = (uint32_t)(x->now - g->seg_start_ms);
            int32_t elapsed = (run > HALF_RANGE) ? 0 : (int32_t)run;
            if (elapsed > g->seg_remaining_ms) {
                elapsed = g->seg_remaining_ms;
            }
            if (elapsed < 0) {
                elapsed = 0;
            }
            total += g->rate * ((double)elapsed / 1000.0);
        }
        *out = (float)total;
    }
    return f;
}

/* ---- storing an output ---------------------------------------------------------------------- */

void derived_core_store(derived_core_t* c, uint32_t sig, uint32_t stamp, derived_flag_t flag, float value)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    derived_out_t* const o = &c->out[sig];
    derived_flag_t f = flag;
    if (f == DERIVED_FLAG_NONE) {
        if (!derived_core_is_finite_f(value)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else if ((value < spec->min) || (value > spec->max)) {
            f = DERIVED_FLAG_OUT_OF_RANGE; /* never clamped */
        } else {
            /* in range */
        }
    }
    o->unranged = (flag == DERIVED_FLAG_NONE) ? value : 0.0f;
    o->flag = f;
    o->status = (f == DERIVED_FLAG_NONE) ? DERIVED_VALID : DERIVED_INVALID;
    o->value = (f == DERIVED_FLAG_NONE) ? value : 0.0f;
    o->stamp_ms = stamp;
    o->seq++;
    if (sig == VEHICLE_CL250_DERIVED_VEHICLE_ACCEL) {
        derived_core_ring_push(c, RING_ACCEL_HIST, (f == DERIVED_FLAG_NONE) ? value : 0.0f, stamp);
        c->accel_hist_valid[(uint32_t)c->ring[RING_ACCEL_HIST].head] = (uint8_t)((f == DERIVED_FLAG_NONE) ? 1u : 0u);
        if (f == DERIVED_FLAG_NONE) {
            derived_core_ring_push(c, RING_ACCEL, value, stamp);
        }
    }
    if (c->ride.open) {
        /* every row: the ride's last-valid / count / max / first and the flag times */
        derived_ride_class_a(&c->ride.acc, sig, stamp, f == DERIVED_FLAG_NONE, value);
        /* a derived window input (W4's gear estimate): the row, valid or not */
        derived_windows_push_derived(&c->ride.acc.win, sig, f == DERIVED_FLAG_NONE, value, stamp);
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the tests and diagnostics
bool derived_core_is_computed(const derived_core_t* c, uint32_t sig)
{
    return (c != NULL) && (sig < VEHICLE_CL250_DERIVED_COUNT) && c->sched[sig].computed;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: read by the tests and diagnostics
derived_nc_reason_t derived_core_reason(const derived_core_t* c, uint32_t sig)
{
    return ((c == NULL) || (sig >= VEHICLE_CL250_DERIVED_COUNT)) ? DERIVED_NC_GEN : c->sched[sig].reason;
}

bool derived_core_ride_closed(const derived_core_t* c, uint16_t* counter)
{
    if ((c == NULL) || (counter == NULL) || !c->ride.have_prev) {
        return false;
    }
    *counter = c->ride.prev_counter;
    return true;
}

bool derived_core_ride_out(const derived_core_t* c, derived_ride_which_t which, uint32_t sig,
                           derived_out_t* out)
{
    if ((c == NULL) || (out == NULL) || (sig >= VEHICLE_CL250_DERIVED_COUNT)) {
        return false;
    }
    const uint32_t slot = derived_ride_slot(sig);
    (void)memset(out, 0, sizeof *out);
    if ((slot == DERIVED_RIDE_NO_SLOT) || (!c->sched[sig].computed)) {
        out->status = DERIVED_NOT_COMPUTED;
    } else if (which == DERIVED_RIDE_PREVIOUS) {
        if (c->ride.have_prev) {
            *out = c->ride.prev[slot];
        } else {
            out->status = DERIVED_NO_DATA;
        }
    } else if (c->ride.open) {
        derived_ride_finalize(&c->ride.acc, sig, out);
        out->stamp_ms = c->ride.last_now_ms;
    } else {
        out->status = DERIVED_NO_DATA;
    }
    return true;
}
