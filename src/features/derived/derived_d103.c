/*
 * D-103 class-A signals of the pure derived core (D-070, D-103): clutch, neutral and engine-braking
 * states, abuse flags, intake and idle residuals, charging and fuel-level windows, and the shift,
 * launch, braking, refuel, battery-rest and cranking event state machines. Split from
 * derived_core.c, which keeps the scheduler, the rings and the output store; the two files talk
 * through derived_core_internal.h only. See features/derived/README.md.
 */
#include "features/derived/derived_core_internal.h"
#include "features/derived/derived_gear.h"

#include "vehicle_cl250_constants.h"

#include <math.h>

/* isfinite() and the NaN-false comparisons need IEEE semantics, logf() must not set errno
 * (like lean_core.c: -fno-math-errno, never -ffast-math). */
#if defined(__FAST_MATH__)
#error "derived_d103.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_d103.c needs -fno-math-errno"
#endif

/* The newest sample of ring `id` at or before `at`, if it is within `limit` ms. */
static bool ring_lookup(derived_core_t* c, uint32_t id, uint32_t at, uint32_t limit, float* v)
{
    uint32_t cap = 0u;
    const derived_sample_t* const d = derived_core_ring_data(c, id, &cap);
    const derived_ring_t* const r = &c->ring[id];
    bool found = false;
    for (uint32_t k = 0u; (k < (uint32_t)r->count) && (!found); k++) {
        const derived_sample_t* const s = &d[((uint32_t)r->head + cap - k) % cap];
        if (derived_core_at_or_after(at, s->stamp_ms)) {
            if ((derived_core_age_of(at, s->stamp_ms) <= limit) && derived_core_is_finite_f(s->value)) {
                *v = s->value;
                found = true;
            } else {
                return false;
            }
        }
    }
    return found;
}

/* The newest vehicle-acceleration sample at or before `at`; `*valid` is the output status. */
static bool ring_lookup_accel(const derived_core_t* c, uint32_t at, uint32_t limit, float* v,
                              bool* valid)
{
    const derived_ring_t* const r = &c->ring[RING_ACCEL_HIST];
    const uint32_t cap = DERIVED_WIN_ACCEL_HIST_CAP;
    bool found = false;
    for (uint32_t k = 0u; (k < (uint32_t)r->count) && (!found); k++) {
        const uint32_t i = ((uint32_t)r->head + cap - k) % cap;
        const derived_sample_t* const s = &c->accel_hist[i];
        if (derived_core_at_or_after(at, s->stamp_ms)) {
            if (derived_core_age_of(at, s->stamp_ms) <= limit) {
                *v = s->value;
                *valid = c->accel_hist_valid[i] != 0u;
                found = true;
            } else {
                return false;
            }
        }
    }
    return found;
}

/* ---- D-103 states, residuals and windows ---------------------------------------------------- */

static derived_flag_t require_moving(const derived_ctx_t* x, float* speed)
{
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, speed);
    if ((f == DERIVED_FLAG_NONE) && (*speed <= VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH)) {
        f = DERIVED_FLAG_NOT_MOVING;
    }
    return f;
}

static float flag_value(bool condition)
{
    return condition ? 1.0f : 0.0f;
}

static derived_flag_t f_clutch_state(const derived_ctx_t* x, float* out)
{
    float speed = 0.0f;
    derived_flag_t f = require_moving(x, &speed);
    if (f == DERIVED_FLAG_NONE) {
        float rpm = 0.0f;
        float measured = 0.0f;
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
        if (f == DERIVED_FLAG_NONE) {
            f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, &measured);
        }
        if (f == DERIVED_FLAG_NONE) {
            const float nearest = derived_gear_nearest_ratio(measured);
            const float relative = (measured / nearest) - 1.0f;
            if (fabsf(relative) <= VEHICLE_CL250_DERIVED_GEAR_RATIO_TOLERANCE) {
                *out = VEHICLE_CL250_DERIVED_VALUE_CLUTCH_STATE_ENGAGED;
            } else if (relative > VEHICLE_CL250_DERIVED_GEAR_RATIO_TOLERANCE) {
                float throttle = 0.0f;
                f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, &throttle);
                if (f == DERIVED_FLAG_NONE) {
                    *out = (throttle > VEHICLE_CL250_DERIVED_THROTTLE_LOAD_MIN_PCT)
                               ? VEHICLE_CL250_DERIVED_VALUE_CLUTCH_STATE_SLIPPING
                               : VEHICLE_CL250_DERIVED_VALUE_CLUTCH_STATE_DISENGAGED;
                }
            } else {
                *out = VEHICLE_CL250_DERIVED_VALUE_CLUTCH_STATE_DISENGAGED;
            }
        }
    }
    return f;
}

static derived_flag_t f_clutch_slip_ratio(const derived_ctx_t* x, float* out)
{
    float gear_value = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, &gear_value);
    if (f == DERIVED_FLAG_NONE) {
        if (gear_value <= VEHICLE_CL250_DERIVED_VALUE_GEAR_ESTIMATE_NO_GEAR) {
            f = DERIVED_FLAG_NO_GEAR;
        } else {
            float measured = 0.0f;
            f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, &measured);
            if (f == DERIVED_FLAG_NONE) {
                *out = measured / derived_gear_ratio((uint8_t)gear_value);
            }
        }
    }
    return f;
}

static derived_flag_t f_neutral_coasting(const derived_ctx_t* x, float* out)
{
    float speed = 0.0f;
    derived_flag_t f = require_moving(x, &speed);
    if (f == DERIVED_FLAG_NONE) {
        float rpm = 0.0f;
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
        if (f == DERIVED_FLAG_NONE) {
            float gear = 0.0f;
            const derived_flag_t gear_flag = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, &gear);
            if ((gear_flag == DERIVED_FLAG_STALE_INPUT) || (gear_flag == DERIVED_FLAG_MATH_ERROR)) {
                f = gear_flag;
            } else if ((gear_flag == DERIVED_FLAG_INVALID_INPUT) || (gear <= VEHICLE_CL250_DERIVED_VALUE_GEAR_ESTIMATE_NO_GEAR)) {
                *out = VEHICLE_CL250_DERIVED_VALUE_NEUTRAL_COASTING_STATE_NEUTRAL;
            } else {
                float throttle = 0.0f;
                f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, &throttle);
                if (f == DERIVED_FLAG_NONE) {
                    *out = (throttle <= VEHICLE_CL250_DERIVED_THROTTLE_CLOSED_MAX_PCT)
                               ? VEHICLE_CL250_DERIVED_VALUE_NEUTRAL_COASTING_STATE_COASTING
                               : VEHICLE_CL250_DERIVED_VALUE_NEUTRAL_COASTING_STATE_DRIVEN;
                }
            }
        }
    }
    return f;
}

static derived_flag_t f_engine_braking_phase(const derived_ctx_t* x, float* out)
{
    float speed = 0.0f;
    float throttle = 0.0f;
    float rpm = 0.0f;
    float fuel = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, &speed);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, &throttle);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &fuel);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = flag_value((speed > VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH) &&
                          (throttle <= VEHICLE_CL250_DERIVED_THROTTLE_CLOSED_MAX_PCT) &&
                          (rpm > VEHICLE_CL250_DERIVED_IDLE_RPM_MAX) &&
                          (fuel <= VEHICLE_CL250_ENGINE_BRAKING_PHASE_FUEL_RATE_MAX_LPH));
    }
    return f;
}

static derived_flag_t f_over_rev(const derived_ctx_t* x, float* out)
{
    float rpm = 0.0f;
    const derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    if (f == DERIVED_FLAG_NONE) {
        *out = flag_value(rpm >= VEHICLE_CL250_OVER_REV_RPM_MIN);
    }
    return f;
}

static derived_flag_t f_lugging(const derived_ctx_t* x, float* out)
{
    float gear = 0.0f;
    float rpm = 0.0f;
    float load = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_GEAR_ESTIMATE, &gear);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_LOAD, &load);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = flag_value((gear >= 2.0f) && (rpm < VEHICLE_CL250_LUGGING_RPM_MAX) &&
                          (load > VEHICLE_CL250_LUGGING_LOAD_MIN_PCT));
    }
    return f;
}

static derived_flag_t f_cold_engine_abuse(const derived_ctx_t* x, float* out)
{
    float coolant = 0.0f;
    float rpm = 0.0f;
    float load = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP, &coolant);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_LOAD, &load);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = flag_value((coolant < VEHICLE_CL250_DERIVED_OPERATING_COOLANT_DEGC) &&
                          ((rpm > VEHICLE_CL250_COLD_ENGINE_ABUSE_RPM_MAX) ||
                           (load > VEHICLE_CL250_COLD_ENGINE_ABUSE_LOAD_MAX_PCT)));
    }
    return f;
}

static derived_flag_t f_heat_soak_delta(const derived_ctx_t* x, float* out)
{
    float speed = 0.0f;
    float intake = 0.0f;
    float ambient = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, &speed);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, &intake);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_AMBIENT_AIR_TEMP, &ambient);
    }
    if (f == DERIVED_FLAG_NONE) {
        if (speed > VEHICLE_CL250_HEAT_SOAK_DELTA_SPEED_MAX_KMH) {
            f = DERIVED_FLAG_NOT_APPLICABLE;
        } else {
            *out = intake - ambient;
        }
    }
    return f;
}

static derived_flag_t f_idle_map_residual(const derived_ctx_t* x, float* out)
{
    float map = 0.0f;
    float throttle = 0.0f;
    float rpm = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_MAP, &map);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, &throttle);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    }
    if (f == DERIVED_FLAG_NONE) {
        if ((throttle > VEHICLE_CL250_DERIVED_THROTTLE_CLOSED_MAX_PCT) ||
            (rpm < VEHICLE_CL250_DERIVED_IDLE_RPM_MIN) || (rpm > VEHICLE_CL250_DERIVED_IDLE_RPM_MAX)) {
            f = DERIVED_FLAG_NOT_APPLICABLE;
        } else {
            *out = map - VEHICLE_CL250_IDLE_MAP_RESIDUAL_EXPECTED_KPA;
        }
    }
    return f;
}

static derived_flag_t f_theoretical_air_flow(const derived_ctx_t* x, float* out)
{
    float rpm = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    if (f == DERIVED_FLAG_NONE) {
        if (rpm <= 0.0f) {
            f = DERIVED_FLAG_ENGINE_OFF;
        } else {
            float map = 0.0f;
            float temperature = 0.0f;
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_MAP, &map);
            if (f == DERIVED_FLAG_NONE) {
                f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, &temperature);
            }
            if (f == DERIVED_FLAG_NONE) {
                const float density = (map * DERIVED_PA_PER_KPA) /
                                      (DERIVED_R_AIR_VE * (temperature + DERIVED_KELVIN_OFFSET));
                const float displacement = (float)VEHICLE_CL250_ENGINE_DISPLACEMENT_CM3 * DERIVED_M3_PER_CM3;
                *out = ((density * displacement * rpm) /
                        (DERIVED_SECONDS_PER_MINUTE * (float)VEHICLE_CL250_ENGINE_REVOLUTIONS_PER_CYCLE)) * DERIVED_G_PER_KG;
            }
        }
    }
    return f;
}

static derived_flag_t f_volumetric_efficiency(const derived_ctx_t* x, float* out)
{
    float maf = 0.0f;
    float theoretical = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_MAF_RATE, &maf);
    if (f == DERIVED_FLAG_NONE) {
        f = f_theoretical_air_flow(x, &theoretical);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = (maf / theoretical) * DERIVED_PERCENT;
    }
    return f;
}

static derived_flag_t f_intake_flow_residual(const derived_ctx_t* x, float* out)
{
    float theoretical = 0.0f;
    derived_flag_t f = f_theoretical_air_flow(x, &theoretical);
    if (f == DERIVED_FLAG_NONE) {
        const float expected = (VEHICLE_CL250_INTAKE_FLOW_RESIDUAL_VE_REFERENCE_PCT / DERIVED_PERCENT) * theoretical;
        float maf = 0.0f;
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_MAF_RATE, &maf);
        if (f == DERIVED_FLAG_NONE) {
            *out = ((maf - expected) / expected) * DERIVED_PERCENT;
        }
    }
    return f;
}

static derived_flag_t f_charging_plateau(const derived_ctx_t* x, float* out)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        const derived_ring_t* const r = &x->c->ring[RING_BATT_HIST];
        const uint32_t cap = DERIVED_WIN_BATT_HIST_CAP;
        float sum = 0.0f;
        uint32_t n = 0u;
        for (uint32_t k = 0u; k < (uint32_t)r->count; k++) {
            const uint32_t i = ((uint32_t)r->head + cap - k) % cap;
            const derived_sample_t* const s = &x->c->batt_hist[i];
            if (derived_core_age_of(x->now, s->stamp_ms) >= (uint32_t)x->spec->window_ms) {
                break;
            }
            float rpm = 0.0f;
            if (ring_lookup(x->c, RING_RPM_HIST, s->stamp_ms,
                            (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms,
                            &rpm) &&
                (rpm > VEHICLE_CL250_CHARGING_PLATEAU_RPM_MIN)) {
                sum += s->value;
                n++;
            }
        }
        if (n == 0u) {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        } else {
            *out = sum / (float)n;
        }
    }
    return f;
}

static derived_flag_t f_fuel_level_filtered(const derived_ctx_t* x, float* out)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_FUEL_LEVEL, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        const derived_ring_t* const r = &x->c->ring[RING_FUEL_HIST];
        const uint32_t cap = DERIVED_WIN_FUEL_HIST_CAP;
        float sum = 0.0f;
        uint32_t n = 0u;
        for (uint32_t k = 0u; k < (uint32_t)r->count; k++) {
            const uint32_t i = ((uint32_t)r->head + cap - k) % cap;
            const derived_sample_t* const s = &x->c->fuel_hist[i];
            if (derived_core_age_of(x->now, s->stamp_ms) >= (uint32_t)x->spec->window_ms) {
                break;
            }
            float acceleration = 0.0f;
            bool valid = false;
            if (ring_lookup_accel(x->c, s->stamp_ms,
                                  (uint32_t)vehicle_cl250_derived[VEHICLE_CL250_DERIVED_VEHICLE_ACCEL].stale_after_ms,
                                  &acceleration, &valid) &&
                valid && (fabsf(acceleration) <= VEHICLE_CL250_FUEL_LEVEL_FILTERED_ACCEL_MAX_MPS2)) {
                sum += s->value;
                n++;
            }
        }
        if (n == 0u) {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        } else {
            *out = sum / (float)n;
        }
    }
    return f;
}

/* Sum of the warm-idle RPM samples of `sig`'s window; also the squared deviations later. */
static uint32_t idle_window(derived_core_t* c, uint32_t now, uint32_t window_ms, float* sum)
{
    const derived_ring_t* const r = &c->ring[RING_RPM_HIST];
    const uint32_t cap = DERIVED_WIN_RPM_HIST_CAP;
    uint32_t n = 0u;
    *sum = 0.0f;
    for (uint32_t k = 0u; k < (uint32_t)r->count; k++) {
        const uint32_t i = ((uint32_t)r->head + cap - k) % cap;
        const derived_sample_t* const s = &c->rpm_hist[i];
        if (derived_core_age_of(now, s->stamp_ms) >= window_ms) {
            break;
        }
        if (c->rpm_warm[i] != 0u) {
            *sum += s->value;
            n++;
        }
    }
    return n;
}

static derived_flag_t f_idle_rpm_std(const derived_ctx_t* x, float* out)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        float sum = 0.0f;
        const uint32_t n = idle_window(x->c, x->now, (uint32_t)x->spec->window_ms, &sum);
        if (n < 2u) {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        } else {
            const float mean = sum / (float)n;
            const derived_ring_t* const r = &x->c->ring[RING_RPM_HIST];
            const uint32_t cap = DERIVED_WIN_RPM_HIST_CAP;
            float squares = 0.0f;
            for (uint32_t k = 0u; k < (uint32_t)r->count; k++) {
                const uint32_t i = ((uint32_t)r->head + cap - k) % cap;
                const derived_sample_t* const s = &x->c->rpm_hist[i];
                if (derived_core_age_of(x->now, s->stamp_ms) >= (uint32_t)x->spec->window_ms) {
                    break;
                }
                if (x->c->rpm_warm[i] != 0u) {
                    const float d = s->value - mean;
                    squares += d * d;
                }
            }
            *out = sqrtf(squares / (float)n);
        }
    }
    return f;
}

static derived_flag_t f_idle_rpm_drift(const derived_ctx_t* x, float* out)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        const derived_d103_state_t* const d = &x->c->d103;
        if ((!d->idle_baseline_ok) || (d->idle_baseline_n == 0u) ||
            (!derived_core_at_or_after(x->now, d->idle_baseline_end_ms))) {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        } else {
            float sum = 0.0f;
            const uint32_t n = idle_window(x->c, x->now, (uint32_t)x->spec->window_ms, &sum);
            if (n == 0u) {
                f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
            } else {
                *out = (sum / (float)n) - (d->idle_baseline_sum / (float)d->idle_baseline_n);
            }
        }
    }
    return f;
}

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

/* Dispatch of the D-103 point formulas; any other signal is a bug (fail closed). */
derived_flag_t derived_d103_eval_point(const derived_ctx_t* x, float* out)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR;
    switch (x->sig) {
    case VEHICLE_CL250_DERIVED_CLUTCH_STATE:
        f = f_clutch_state(x, out);
        break;
    case VEHICLE_CL250_DERIVED_CLUTCH_SLIP_RATIO:
        f = f_clutch_slip_ratio(x, out);
        break;
    case VEHICLE_CL250_DERIVED_NEUTRAL_COASTING_STATE:
        f = f_neutral_coasting(x, out);
        break;
    case VEHICLE_CL250_DERIVED_ENGINE_BRAKING_PHASE:
        f = f_engine_braking_phase(x, out);
        break;
    case VEHICLE_CL250_DERIVED_OVER_REV:
        f = f_over_rev(x, out);
        break;
    case VEHICLE_CL250_DERIVED_LUGGING:
        f = f_lugging(x, out);
        break;
    case VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE:
        f = f_cold_engine_abuse(x, out);
        break;
    case VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA:
        f = f_heat_soak_delta(x, out);
        break;
    case VEHICLE_CL250_DERIVED_IDLE_MAP_RESIDUAL:
        f = f_idle_map_residual(x, out);
        break;
    case VEHICLE_CL250_DERIVED_VOLUMETRIC_EFFICIENCY:
        f = f_volumetric_efficiency(x, out);
        break;
    case VEHICLE_CL250_DERIVED_INTAKE_FLOW_RESIDUAL:
        f = f_intake_flow_residual(x, out);
        break;
    case VEHICLE_CL250_DERIVED_CHARGING_PLATEAU_VOLTAGE:
        f = f_charging_plateau(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_LEVEL_FILTERED:
        f = f_fuel_level_filtered(x, out);
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_STD:
        f = f_idle_rpm_std(x, out);
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT:
        f = f_idle_rpm_drift(x, out);
        break;
    default:
        f = DERIVED_FLAG_MATH_ERROR;
        break;
    }
    return f;
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
