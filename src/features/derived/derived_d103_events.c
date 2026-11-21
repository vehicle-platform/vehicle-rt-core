/*
 * D-103 sample history and the shift, launch, braking, refuel, battery-rest and cranking event state
 * machines of the pure derived core (D-070, D-103). Split from derived_d103.c (the point formulas) and
 * derived_core.c; the units talk through derived_core_internal.h only. See features/derived/README.md.
 */
#include "features/derived/derived_core_internal.h"
#include "features/derived/derived_gear.h"

#include "vehicle_cl250_constants.h"

#include <math.h>

/* isfinite() and the NaN-false comparisons need IEEE semantics, logf() must not set errno
 * (like lean_core.c: -fno-math-errno, never -ffast-math). */
#if defined(__FAST_MATH__)
#error "derived_d103_events.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_d103_events.c needs -fno-math-errno"
#endif

/* ---- D-103 sample history and event state ---------------------------------------------------- */

void derived_d103_record(derived_core_t* c)
{
    const derived_did_t* const rpm = &c->did[VEHICLE_CL250_IDX_ENGINE_SPEED];
    if (rpm->fresh_now && derived_core_is_finite_f(rpm->value)) {
        derived_core_ring_push(c, RING_RPM_HIST, rpm->value, rpm->stamp_ms);
        const uint32_t head = (uint32_t)c->ring[RING_RPM_HIST].head;
        bool warm = false;
        float speed = 0.0f;
        float coolant = 0.0f;
        if ((rpm->value >= VEHICLE_CL250_DERIVED_IDLE_RPM_MIN) &&
            (rpm->value <= VEHICLE_CL250_DERIVED_IDLE_RPM_MAX) &&
            ring_lookup(c, RING_SPEED, rpm->stamp_ms,
                        (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms,
                        &speed) &&
            (speed <= VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH) &&
            ring_lookup(c, RING_COOLANT, rpm->stamp_ms,
                        (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP].stale_after_ms,
                        &coolant) &&
            (coolant >= VEHICLE_CL250_DERIVED_OPERATING_COOLANT_DEGC)) {
            warm = true;
        }
        c->rpm_warm[head] = (uint8_t)(warm ? 1u : 0u);
        if (warm) {
            derived_d103_state_t* const d = &c->d103;
            const uint32_t window = (uint32_t)vehicle_cl250_derived[VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT].window_ms;
            if (!d->idle_baseline_ok) {
                d->idle_baseline_ok = true;
                d->idle_baseline_end_ms = rpm->stamp_ms + window;
            }
            if (!derived_core_at_or_after(rpm->stamp_ms, d->idle_baseline_end_ms)) {
                d->idle_baseline_sum += rpm->value;
                d->idle_baseline_n++;
            }
        }
    }
    const derived_did_t* const batt = &c->did[VEHICLE_CL250_IDX_BATTERY_VOLTAGE];
    if (batt->fresh_now && derived_core_is_finite_f(batt->value)) {
        derived_core_ring_push(c, RING_BATT_HIST, batt->value, batt->stamp_ms);
    }
    const derived_did_t* const fuel = &c->did[VEHICLE_CL250_IDX_FUEL_LEVEL];
    if (fuel->fresh_now && derived_core_is_finite_f(fuel->value)) {
        derived_core_ring_push(c, RING_FUEL_HIST, fuel->value, fuel->stamp_ms);
    }
}

/* D-103 event times use the sample's own stamp, not the step time. */
static derived_ctx_t event_ctx(derived_core_t* c, uint32_t sig, uint32_t stamp)
{
    const derived_ctx_t x = {c, stamp, sig, &vehicle_cl250_derived[sig]};
    return x;
}

static void shift_step(derived_core_t* c, uint32_t sig)
{
    const derived_did_t* const rpm = &c->did[VEHICLE_CL250_IDX_ENGINE_SPEED];
    if ((!rpm->fresh_now) || (!derived_core_is_finite_f(rpm->value))) {
        return;
    }
    derived_d103_state_t* const d = &c->d103;
    const uint32_t t = rpm->stamp_ms;
    const uint32_t limit = (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms;
    if (d->shift_has_prev && (derived_core_age_of(t, d->shift_prev_ms) > limit)) {
        d->shift_floor_set = true;
        d->shift_floor_ms = t;
    }
    d->shift_prev_ms = t;
    d->shift_has_prev = true;
    const uint32_t speed_limit =
        (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms;
    float speed_now = 0.0f;
    if (!ring_lookup(c, RING_SPEED, t, speed_limit, &speed_now)) {
        return;
    }
    const derived_ring_t* const r = &c->ring[RING_RPM_HIST];
    const uint32_t cap = DERIVED_WIN_RPM_HIST_CAP;
    for (uint32_t back = (uint32_t)r->count; back > 1u; back--) {
        const uint32_t i = ((uint32_t)r->head + cap - (back - 1u)) % cap;
        const derived_sample_t* const s = &c->rpm_hist[i];
        if (derived_core_age_of(t, s->stamp_ms) > (uint32_t)VEHICLE_CL250_SHIFT_EVENT_WINDOW_MS) {
            continue;
        }
        if (d->shift_floor_set && (!derived_core_at_or_after(s->stamp_ms, d->shift_floor_ms))) {
            continue;
        }
        float speed_start = 0.0f;
        if ((!ring_lookup(c, RING_SPEED, s->stamp_ms, speed_limit, &speed_start)) ||
            (fabsf(rpm->value - s->value) < VEHICLE_CL250_SHIFT_EVENT_RPM_STEP_MIN_RPM) ||
            (fabsf(speed_now - speed_start) > VEHICLE_CL250_SHIFT_EVENT_SPEED_CHANGE_MAX_KMH)) {
            continue;
        }
        const uint32_t duration_ms = t - s->stamp_ms;
        derived_core_store(c, sig, t, DERIVED_FLAG_NONE, (float)duration_ms);
        d->shift_floor_set = true;
        d->shift_floor_ms = t;
        break;
    }
}

static void launch_step(derived_core_t* c, uint32_t sig)
{
    const derived_did_t* const speed = &c->did[VEHICLE_CL250_IDX_VEHICLE_SPEED];
    if ((!speed->fresh_now) || (!derived_core_is_finite_f(speed->value))) {
        return;
    }
    derived_d103_state_t* const d = &c->d103;
    const uint32_t t = speed->stamp_ms;
    const float v = speed->value;
    const uint32_t limit = (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms;
    if (d->launch_has_prev && (derived_core_age_of(t, d->launch_prev_ms) > limit)) {
        d->launch_active = false;
        d->launch_prev_valid = false;
    }
    if (d->launch_active) {
        const derived_ctx_t x = event_ctx(c, sig, t);
        float ratio = 0.0f;
        bool in_first = false;
        if (derived_core_derived_value(&x, VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, &ratio) == DERIVED_FLAG_NONE) {
            in_first = fabsf((ratio / VEHICLE_CL250_GEAR_RATIO_1) - 1.0f) <=
                       VEHICLE_CL250_DERIVED_GEAR_RATIO_TOLERANCE;
        }
        if ((v > VEHICLE_CL250_LAUNCH_EVENT_END_SPEED_KMH) || in_first) {
            const uint32_t duration_ms = t - d->launch_start_ms;
            derived_core_store(c, sig, t, DERIVED_FLAG_NONE, (float)duration_ms);
            d->launch_active = false;
        } else {
            if (v <= VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH) {
                d->launch_active = false;
            }
        }
    } else {
        if (d->launch_prev_valid &&
            (d->launch_prev_kmh <= VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH) &&
            (v > VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH)) {
            d->launch_active = true;
            d->launch_start_ms = t;
        }
    }
    d->launch_prev_ms = t;
    d->launch_prev_valid = true;
    d->launch_prev_kmh = v;
    d->launch_has_prev = true;
}

static void braking_step(derived_core_t* c, uint32_t sig)
{
    derived_d103_state_t* const d = &c->d103;
    const derived_out_t* const o = &c->out[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL];
    if (o->seq == d->braking_seq) {
        return;
    }
    d->braking_seq = o->seq;
    const uint32_t t = o->stamp_ms;
    const uint32_t limit =
        (uint32_t)vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL].stale_after_ms;
    if (o->status != DERIVED_VALID) {
        d->braking_active = false;
        d->braking_has_prev = false;
        return;
    }
    if (d->braking_has_prev && (derived_core_age_of(t, d->braking_prev_ms) > limit)) {
        d->braking_active = false;
    }
    d->braking_prev_ms = t;
    d->braking_has_prev = true;
    const float threshold = -VEHICLE_CL250_BRAKING_EVENT_DECEL_MIN_MPS2;
    if (o->value < threshold) {
        if (!d->braking_active) {
            d->braking_active = true;
            d->braking_run_ms = t;
            d->braking_peak_mps2 = o->value;
        } else {
            if (o->value < d->braking_peak_mps2) {
                d->braking_peak_mps2 = o->value;
            }
        }
    } else {
        if (d->braking_active &&
            (derived_core_age_of(t, d->braking_run_ms) >= (uint32_t)VEHICLE_CL250_BRAKING_EVENT_MIN_DURATION_MS)) {
            derived_core_store(c, sig, t, DERIVED_FLAG_NONE, d->braking_peak_mps2);
        }
        d->braking_active = false;
    }
}

static void refuel_step(derived_core_t* c, uint32_t sig)
{
    const derived_did_t* const speed = &c->did[VEHICLE_CL250_IDX_VEHICLE_SPEED];
    if ((!speed->fresh_now) || (!derived_core_is_finite_f(speed->value))) {
        return;
    }
    derived_d103_state_t* const d = &c->d103;
    const uint32_t t = speed->stamp_ms;
    if (speed->value > VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH) {
        if (d->refuel_has_moving && d->refuel_rested) {
            const derived_ctx_t x = event_ctx(c, sig, t);
            float fuel_now = 0.0f;
            if (d->refuel_fuel_ok &&
                (derived_core_did_value(&x, (uint32_t)VEHICLE_CL250_IDX_FUEL_LEVEL, &fuel_now) == DERIVED_FLAG_NONE)) {
                const float rise = fuel_now - d->refuel_fuel_pct;
                if (rise >= VEHICLE_CL250_REFUEL_EVENT_STEP_MIN_PCT) {
                    derived_core_store(c, sig, t, DERIVED_FLAG_NONE, (rise * VEHICLE_CL250_FUEL_TANK_CAPACITY_L) / DERIVED_PERCENT);
                }
            }
        }
        d->refuel_moving_ms = t;
        d->refuel_has_moving = true;
        d->refuel_rested = false;
        const derived_ctx_t x = event_ctx(c, sig, t);
        d->refuel_fuel_ok =
            derived_core_did_value(&x, (uint32_t)VEHICLE_CL250_IDX_FUEL_LEVEL, &d->refuel_fuel_pct) == DERIVED_FLAG_NONE;
        d->refuel_fuel_stamp_ms = t;
    } else {
        if (d->refuel_has_moving) {
            d->refuel_rested = true;
        }
    }
}

static void battery_rest_step(derived_core_t* c, uint32_t sig)
{
    const derived_did_t* const rpm = &c->did[VEHICLE_CL250_IDX_ENGINE_SPEED];
    if ((!rpm->fresh_now) || (!derived_core_is_finite_f(rpm->value))) {
        return;
    }
    derived_d103_state_t* const d = &c->d103;
    const uint32_t t = rpm->stamp_ms;
    const uint32_t limit = (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms;
    if (d->rest_has_prev && (derived_core_age_of(t, d->rest_prev_ms) > limit)) {
        d->rest_zero_valid = false;
    }
    d->rest_prev_ms = t;
    d->rest_has_prev = true;
    if (rpm->value <= 0.0f) {
        const derived_ctx_t x = event_ctx(c, sig, t);
        d->rest_zero_ms = t;
        d->rest_zero_valid =
            derived_core_did_value(&x, (uint32_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE, &d->rest_zero_v) == DERIVED_FLAG_NONE;
    } else {
        if (d->rest_zero_valid) {
            derived_core_store(c, sig, t, DERIVED_FLAG_NONE, d->rest_zero_v);
            d->rest_zero_valid = false;
        }
    }
}

static void cranking_step(derived_core_t* c, uint32_t sig)
{
    derived_d103_state_t* const d = &c->d103;
    const derived_did_t* const rpm = &c->did[VEHICLE_CL250_IDX_ENGINE_SPEED];
    if (rpm->fresh_now && derived_core_is_finite_f(rpm->value)) {
        const uint32_t t = rpm->stamp_ms;
        const uint32_t limit = (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms;
        if (d->crank_has_prev && (derived_core_age_of(t, d->crank_prev_ms) > limit)) {
            d->crank_zero_valid = false;
            d->crank_active = false;
            d->crank_min_ok = false;
            d->crank_end_pending = false;
        }
        d->crank_prev_ms = t;
        d->crank_has_prev = true;
        if (rpm->value <= 0.0f) {
            d->crank_zero_ms = t;
            d->crank_zero_valid = true;
            d->crank_active = false;
            d->crank_min_ok = false;
            d->crank_end_pending = false;
        } else if (d->crank_active) {
            if (rpm->value > VEHICLE_CL250_DERIVED_IDLE_RPM_MIN) {
                d->crank_end_ms = t;
                d->crank_end_pending = true;
            }
        } else if (d->crank_zero_valid) {
            d->crank_active = true;
            d->crank_start_ms = t;
            d->crank_min_ok = false;
            d->crank_zero_valid = false;
        } else {
            /* no 0 -> >0 edge */
        }
    }
    const derived_did_t* const batt = &c->did[VEHICLE_CL250_IDX_BATTERY_VOLTAGE];
    if (d->crank_active && batt->fresh_now && derived_core_is_finite_f(batt->value) &&
        derived_core_at_or_after(batt->stamp_ms, d->crank_start_ms) &&
        ((!d->crank_end_pending) || derived_core_at_or_after(d->crank_end_ms, batt->stamp_ms))) {
        if ((!d->crank_min_ok) || (batt->value < d->crank_min_v)) {
            d->crank_min_v = batt->value;
            d->crank_min_ok = true;
        }
    }
    if (d->crank_end_pending) {
        if (d->crank_min_ok) {
            derived_core_store(c, sig, d->crank_end_ms, DERIVED_FLAG_NONE, d->crank_min_v);
        }
        d->crank_active = false;
        d->crank_min_ok = false;
        d->crank_end_pending = false;
    }
}

/* Dispatch of the D-103 event state machines; any other signal does nothing. */
void derived_d103_event_step(derived_core_t* c, uint32_t sig)
{
    switch (sig) {
    case VEHICLE_CL250_DERIVED_SHIFT_EVENT:
        shift_step(c, sig);
        break;
    case VEHICLE_CL250_DERIVED_LAUNCH_EVENT:
        launch_step(c, sig);
        break;
    case VEHICLE_CL250_DERIVED_BRAKING_EVENT:
        braking_step(c, sig);
        break;
    case VEHICLE_CL250_DERIVED_REFUEL_EVENT:
        refuel_step(c, sig);
        break;
    case VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE:
        battery_rest_step(c, sig);
        break;
    case VEHICLE_CL250_DERIVED_CRANKING_DIP:
        cranking_step(c, sig);
        break;
    default:
        break;
    }
}

