/*
 * Init of the pure derived core (D-070): which signals are computed (sizing against the rings, the
 * DID polls and the gen/ table) and why a signal is not.
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
#error "derived_init.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_init.c needs -fno-math-errno"
#endif

/* ---- init: which signals are computed ---------------------------------------------------------- */

static bool fits(uint32_t window_ms, uint32_t period_ms, uint32_t cap)
{
    return (window_ms != 0u) && (period_ms != 0u) && (((window_ms / period_ms) + 2u) <= cap);
}

static bool sizing_ok(uint32_t sig)
{
    const vehicle_cl250_derived_t* const s = &vehicle_cl250_derived[sig];
    const uint32_t w = s->window_ms;
    bool ok = true;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_VEHICLE_ACCEL:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].poll_period_ms, DERIVED_WIN_SPEED_CAP);
        break;
    case VEHICLE_CL250_DERIVED_VEHICLE_JERK:
        ok = fits(w, vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL].period_ms, DERIVED_WIN_ACCEL_CAP);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].poll_period_ms, DERIVED_WIN_SPEED_CAP) &&
             fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_FUEL_RATE].poll_period_ms, DERIVED_WIN_FUEL_RATE_CAP);
        break;
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP].poll_period_ms, DERIVED_WIN_COOLANT_CAP);
        break;
    case VEHICLE_CL250_DERIVED_OIL_TEMP_RATE:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_OIL_TEMP].poll_period_ms, DERIVED_WIN_OIL_CAP);
        break;
    case VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_BATTERY_VOLTAGE].poll_period_ms,
                  DERIVED_WIN_BATT_HIST_CAP) &&
             fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].poll_period_ms,
                  DERIVED_WIN_RPM_HIST_CAP);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_FUEL_LEVEL].poll_period_ms,
                  DERIVED_WIN_FUEL_HIST_CAP) &&
             fits(w, vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL].period_ms,
                  DERIVED_WIN_ACCEL_HIST_CAP);
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_STD:
    case VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT:
        ok = fits(w, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].poll_period_ms,
                  DERIVED_WIN_RPM_HIST_CAP);
        break;
    default:
        ok = true;
        break;
    }
    return ok && derived_ride_tail_fits(sig); /* the waiting rows of the ride summaries */
}

/* D-106 item 3: throttle_torque_lag is resolvable only when both lag DIDs are polled at least
 * every throttle_torque_lag_search_max_ms / throttle_torque_lag_min_samples (gen/ poll periods). */
static bool lag_resolvable(void)
{
    const uint32_t min_samples = (uint32_t)VEHICLE_CL250_THROTTLE_TORQUE_LAG_MIN_SAMPLES;
    const uint32_t thr = vehicle_cl250_dids[VEHICLE_CL250_IDX_THROTTLE_POS].poll_period_ms;
    const uint32_t trq = vehicle_cl250_dids[VEHICLE_CL250_IDX_ACTUAL_TORQUE_PCT].poll_period_ms;
    const uint32_t slowest = (thr > trq) ? thr : trq;
    return (min_samples != 0u) && (slowest != 0u) &&
           (slowest <= ((uint32_t)VEHICLE_CL250_THROTTLE_TORQUE_LAG_SEARCH_MAX_MS / min_samples));
}

static derived_nc_reason_t classify(const derived_core_t* c, uint32_t sig)
{
    const vehicle_cl250_derived_t* const s = &vehicle_cl250_derived[sig];
    const bool ride = (s->derived_class == VEHICLE_CL250_DERIVED_CLASS_RIDE);
    derived_nc_reason_t r = DERIVED_NC_NONE;
    if ((s->derived_class != VEHICLE_CL250_DERIVED_CLASS_LIVE) && (!ride)) {
        r = DERIVED_NC_CLASS;
    } else if (!s->rt_core_ready) {
        r = DERIVED_NC_NOT_READY;
    } else if (ride && ((s->elements != 1u) || (s->axis_count != 0u) || s->per_gear)) {
        r = DERIVED_NC_ELEMENTS; /* binned or per-gear: more than a scalar state */
    } else if ((sig == VEHICLE_CL250_DERIVED_RIDE_AGGRESSIVENESS_SCORE) ||
               ((sig == VEHICLE_CL250_DERIVED_THROTTLE_TORQUE_LAG) && lag_resolvable())) {
        /* a p95 / a median over every sample or step of the ride; a lag whose polls are too slow
         * (the CL250) is computed instead: always NA, DERIVED_FLAG_POLL_TOO_SLOW (D-106) */
        r = DERIVED_NC_UNBOUNDED;
    } else if (kind_of(sig) == KIND_NONE) {
        r = DERIVED_NC_NO_FORMULA;
    } else if ((s->input_count == 0u) || (s->input_count > VEHICLE_CL250_DERIVED_INPUT_MAX) ||
               ((!ride) && (!s->event) && (s->period_ms == 0u)) ||
               ((kind_of(sig) == KIND_EVENT) ? (!s->event) : s->event)) {
        r = DERIVED_NC_GEN;
    } else {
        for (uint32_t k = 0u; k < (uint32_t)s->input_count; k++) {
            const uint32_t idx = s->inputs[k].index;
            const uint8_t kind = s->inputs[k].kind;
            if (kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
                if (idx >= VEHICLE_CL250_DID_COUNT) {
                    r = DERIVED_NC_GEN;
                }
            } else if (kind == VEHICLE_CL250_DERIVED_INPUT_DERIVED) {
                /* the producer must come earlier in the table and be computed itself */
                if ((idx >= sig) || (!c->sched[idx].computed)) {
                    r = DERIVED_NC_INPUT;
                }
            } else if ((sig == (uint32_t)VEHICLE_CL250_DERIVED_DATA_QUALITY_MASK) &&
                       (kind == VEHICLE_CL250_DERIVED_INPUT_SENSOR)) {
                /* D-104, the one exception: its sensors (bus-off count, step gap) come from
                 * the tester's quality record, not from a sensor source */
                if (!dq_bits_cover_dids()) {
                    r = DERIVED_NC_GEN; /* a poll period without a bit in the defs */
                }
            } else {
                r = DERIVED_NC_INPUT; /* a sensor: none is computed yet */
            }
        }
        if ((r == DERIVED_NC_NONE) && (!sizing_ok(sig))) {
            r = DERIVED_NC_SIZING;
        }
        const uint32_t win = ride ? derived_windows_of_signal(sig) : DERIVED_WINDOWS_NONE;
        if ((r == DERIVED_NC_NONE) && (win != DERIVED_WINDOWS_NONE)) {
            /* a D-079 window summary also needs the window's own inputs (a derived channel that
             * is not computed, a sensor) and its sample buffers */
            uint8_t chans[DERIVED_WINDOWS_CH_MAX];
            const uint32_t n = derived_windows_derived_inputs(win, chans);
            if (n > DERIVED_WINDOWS_CH_MAX) {
                r = DERIVED_NC_INPUT;
            } else {
                for (uint32_t k = 0u; k < n; k++) {
                    if ((chans[k] >= sig) || (!c->sched[chans[k]].computed)) {
                        r = DERIVED_NC_INPUT;
                    }
                }
            }
            if ((r == DERIVED_NC_NONE) && (!derived_windows_sizing_ok(win, DERIVED_WIN_RPM_HIST_CAP))) {
                r = DERIVED_NC_SIZING;
            }
        }
    }
    return r;
}

void derived_core_init(derived_core_t* c)
{
    if (c == NULL) {
        return;
    }
    (void)memset(c, 0, sizeof *c);
    derived_ride_init(&c->ride);
    uint32_t max_stale = 0u;
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        const derived_nc_reason_t r = classify(c, sig);
        c->sched[sig].reason = r;
        c->sched[sig].computed = (r == DERIVED_NC_NONE);
        c->out[sig].status = (r == DERIVED_NC_NONE) ? DERIVED_NO_DATA : DERIVED_NOT_COMPUTED;
        c->out[sig].flag = DERIVED_FLAG_NONE;
        if (r == DERIVED_NC_NONE) {
            c->computed_count++;
            if ((uint32_t)vehicle_cl250_derived[sig].stale_after_ms > max_stale) {
                max_stale = vehicle_cl250_derived[sig].stale_after_ms;
            }
        }
    }
    for (uint32_t sig = 0u; sig < VEHICLE_CL250_DERIVED_COUNT; sig++) {
        const uint32_t win = derived_windows_of_signal(sig);
        if (c->sched[sig].computed && (win != DERIVED_WINDOWS_NONE)) {
            c->win_mask |= (1u << win);
        }
    }
    /* the DIDs' own limits (every DID input, events included, D-087) */
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if ((uint32_t)vehicle_cl250_dids[i].stale_after_ms > max_stale) {
            max_stale = vehicle_cl250_dids[i].stale_after_ms;
        }
    }
    c->max_stale_ms = max_stale;
}

const derived_out_t* derived_core_out(const derived_core_t* c, uint32_t sig)
{
    return ((c == NULL) || (sig >= VEHICLE_CL250_DERIVED_COUNT)) ? NULL : &c->out[sig];
}
