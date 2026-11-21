/*
 * D-103 class-A point formulas of the pure derived core (D-070, D-103): clutch, neutral and engine-braking
 * states, abuse flags, intake and idle residuals, charging and fuel-level windows. Split from
 * derived_core.c, which keeps the rings and the output store (the scheduler is derived_step.c); the
 * event state machines are derived_d103_events.c; the units talk through derived_core_internal.h
 * only. See features/derived/README.md.
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
bool ring_lookup(derived_core_t* c, uint32_t id, uint32_t at, uint32_t limit, float* v)
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
                                      (VEHICLE_CL250_DRY_AIR_GAS_CONSTANT_J_PER_KG_K * (temperature + DERIVED_KELVIN_OFFSET));
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

