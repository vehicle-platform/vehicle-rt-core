/*
 * Class-A point formulas of the pure derived core (D-070, D-104): the DID-only formulas, the
 * data_quality_mask and the point dispatcher `eval_point`.
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
#error "derived_formulas.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_formulas.c needs -fno-math-errno"
#endif

/* ---- formulas ---------------------------------------------------------------------------- */

static float air_density(float pressure_kpa, float temperature_c)
{
    return (pressure_kpa * DERIVED_PA_PER_KPA) /
           (VEHICLE_CL250_DRY_AIR_GAS_CONSTANT_J_PER_KG_K * (temperature_c + DERIVED_KELVIN_OFFSET));
}

static derived_flag_t f_vehicle_accel(const derived_ctx_t* x, float* out)
{
    win_t w;
    derived_flag_t f = did_window(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, DERIVED_KMH_TO_MPS, true, &w);
    if (f == DERIVED_FLAG_NONE) {
        if (w.slope_ok) {
            *out = w.slope_per_s;
        } else {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        }
    }
    return f;
}

static derived_flag_t f_vehicle_jerk(const derived_ctx_t* x, float* out)
{
    float dummy = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_VEHICLE_ACCEL, &dummy);
    if (f == DERIVED_FLAG_NONE) {
        win_t w;
        win_collect(x->c, RING_ACCEL, x->now, (uint32_t)x->spec->window_ms, 1.0f, true, &w);
        if (w.slope_ok) {
            *out = w.slope_per_s;
        } else {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        }
    }
    return f;
}

static derived_flag_t f_gearbox_ratio(const derived_ctx_t* x, float* out)
{
    float speed_kmh = 0.0f;
    float rpm = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, &speed_kmh);
    if (f == DERIVED_FLAG_NONE) {
        const float speed = speed_kmh * DERIVED_KMH_TO_MPS;
        if (speed <= 0.0f) {
            f = DERIVED_FLAG_STANDSTILL;
        } else {
            const float wheel_rpm = (speed / (2.0f * DERIVED_PI * VEHICLE_CL250_REAR_WHEEL_RADIUS_DYNAMIC_M)) * DERIVED_SECONDS_PER_MINUTE;
            const float reduction = VEHICLE_CL250_PRIMARY_REDUCTION_RATIO * VEHICLE_CL250_SECONDARY_REDUCTION_RATIO;
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
            if (f == DERIVED_FLAG_NONE) {
                *out = rpm / (wheel_rpm * reduction);
            }
        }
    }
    return f;
}

static derived_flag_t f_gear_estimate(const derived_ctx_t* x, float* out)
{
    float ratio = 0.0f;
    const derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED, &ratio);
    if (f == DERIVED_FLAG_NONE) {
        *out = (float)derived_gear_estimate(ratio);
    }
    return f;
}

/* a * b / 100 of two DIDs (torques: percent of the reference torque). */
static derived_flag_t torque_of(const derived_ctx_t* x, uint32_t pct_did, float* out)
{
    float pct = 0.0f;
    float ref = 0.0f;
    derived_flag_t f = derived_core_did_value(x, pct_did, &pct);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_REFERENCE_TORQUE, &ref);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = (pct * ref) / DERIVED_PERCENT;
    }
    return f;
}

static derived_flag_t f_torque_gap(const derived_ctx_t* x, float* out)
{
    float demand = 0.0f;
    float actual = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_DEMAND_TORQUE, &demand);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_ENGINE_TORQUE, &actual);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = demand - actual;
    }
    return f;
}

static derived_flag_t f_engine_power(const derived_ctx_t* x, float* out)
{
    float torque = 0.0f;
    float rpm = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_ENGINE_TORQUE, &torque);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = ((torque * rpm) * DERIVED_RPM_TO_RAD_PER_S) / DERIVED_G_PER_KG;
    }
    return f;
}

static derived_flag_t f_air_density(const derived_ctx_t* x, float* out)
{
    float p = 0.0f;
    float t = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, &p);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, &t);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = air_density(p, t);
    }
    return f;
}

static derived_flag_t f_power_correction(const derived_ctx_t* x, float* out)
{
    float rho = 0.0f;
    const derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_AIR_DENSITY, &rho);
    if (f == DERIVED_FLAG_NONE) {
        const float reference = air_density(VEHICLE_CL250_J1349_REFERENCE_PRESSURE_KPA, VEHICLE_CL250_J1349_REFERENCE_TEMPERATURE_C);
        *out = reference / rho;
    }
    return f;
}

static derived_flag_t f_air_fuel_ratio(const derived_ctx_t* x, float* out)
{
    float lambda = 0.0f;
    const derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, &lambda);
    if (f == DERIVED_FLAG_NONE) {
        *out = lambda * VEHICLE_CL250_STOICHIOMETRIC_AFR;
    }
    return f;
}

static derived_flag_t f_fuel_rate_maf(const derived_ctx_t* x, float* out)
{
    float lambda = 0.0f;
    float maf = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_COMMANDED_EQUIV_RATIO, &lambda);
    if (f == DERIVED_FLAG_NONE) {
        const float afr = lambda * VEHICLE_CL250_STOICHIOMETRIC_AFR;
        if (afr <= 0.0f) {
            f = DERIVED_FLAG_INVALID_INPUT;
        } else {
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_MAF_RATE, &maf);
            if (f == DERIVED_FLAG_NONE) {
                *out = (maf * DERIVED_GPS_TO_KG_PER_H) / (afr * VEHICLE_CL250_FUEL_DENSITY_KG_PER_L);
            }
        }
    }
    return f;
}

static derived_flag_t f_fuel_consumption_instant(const derived_ctx_t* x, float* out)
{
    float speed = 0.0f;
    float rate = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, &speed);
    if (f == DERIVED_FLAG_NONE) {
        if (speed <= 0.0f) {
            f = DERIVED_FLAG_STANDSTILL;
        } else {
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &rate);
            if (f == DERIVED_FLAG_NONE) {
                *out = (rate / speed) * DERIVED_PERCENT;
            }
        }
    }
    return f;
}

static derived_flag_t f_fuel_consumption_avg(const derived_ctx_t* x, float* out)
{
    win_t rate;
    win_t speed;
    derived_flag_t f = did_window(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, 1.0f, false, &rate);
    if (f == DERIVED_FLAG_NONE) {
        f = did_window(x, (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED, 1.0f, false, &speed);
    }
    if (f == DERIVED_FLAG_NONE) {
        if ((rate.n < DERIVED_MIN_WINDOW_SAMPLES) || (speed.n < DERIVED_MIN_WINDOW_SAMPLES)) {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        } else if (speed.mean <= 0.0f) {
            f = DERIVED_FLAG_STANDSTILL;
        } else {
            *out = (DERIVED_PERCENT * rate.mean) / speed.mean;
        }
    }
    return f;
}

static derived_flag_t f_fuel_range(const derived_ctx_t* x, float* out)
{
    float avg = 0.0f;
    float level = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG, &avg);
    if (f == DERIVED_FLAG_NONE) {
        if (avg <= 0.0f) {
            f = DERIVED_FLAG_NO_CONSUMPTION;
        } else {
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_FUEL_LEVEL, &level);
            if (f == DERIVED_FLAG_NONE) {
                const float litres = (level / DERIVED_PERCENT) * VEHICLE_CL250_FUEL_TANK_CAPACITY_L;
                *out = (litres / avg) * DERIVED_PERCENT;
            }
        }
    }
    return f;
}

static derived_flag_t f_bsfc(const derived_ctx_t* x, float* out)
{
    float power = 0.0f;
    float rate = 0.0f;
    derived_flag_t f = derived_core_derived_value(x, VEHICLE_CL250_DERIVED_ENGINE_POWER, &power);
    if (f == DERIVED_FLAG_NONE) {
        if (power <= 0.0f) {
            f = DERIVED_FLAG_NO_POWER;
        } else {
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &rate);
            if (f == DERIVED_FLAG_NONE) {
                const float mass_rate = rate * VEHICLE_CL250_FUEL_DENSITY_KG_PER_L;
                *out = (mass_rate * DERIVED_G_PER_KG) / power;
            }
        }
    }
    return f;
}

static derived_flag_t f_fuel_per_stroke(const derived_ctx_t* x, float* out)
{
    float rpm = 0.0f;
    float rate = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED, &rpm);
    if (f == DERIVED_FLAG_NONE) {
        if (rpm <= 0.0f) {
            f = DERIVED_FLAG_ENGINE_OFF;
        } else {
            f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE, &rate);
            if (f == DERIVED_FLAG_NONE) {
                const float mg_per_hour = (rate * VEHICLE_CL250_FUEL_DENSITY_KG_PER_L) * DERIVED_MG_PER_KG;
                *out = mg_per_hour / ((rpm * DERIVED_SECONDS_PER_MINUTE) / (float)VEHICLE_CL250_ENGINE_REVOLUTIONS_PER_CYCLE);
            }
        }
    }
    return f;
}

/* a +/- b of two DIDs: sum = true adds, else a - b. */
static derived_flag_t two_dids(const derived_ctx_t* x, uint32_t a_did, uint32_t b_did, bool sum, float* out)
{
    float a = 0.0f;
    float b = 0.0f;
    derived_flag_t f = derived_core_did_value(x, a_did, &a);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, b_did, &b);
    }
    if (f == DERIVED_FLAG_NONE) {
        *out = sum ? (a + b) : (a - b);
    }
    return f;
}

static derived_flag_t f_throttle_channel(const derived_ctx_t* x, float* out)
{
    float a = 0.0f;
    float b = 0.0f;
    float r = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, &a);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_ABSOLUTE_THROTTLE_POS_B, &b);
    }
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_RELATIVE_THROTTLE_POS, &r);
    }
    if (f == DERIVED_FLAG_NONE) {
        float m = fabsf(a - b);
        m = fmaxf(m, fabsf(a - r));
        m = fmaxf(m, fabsf(b - r));
        *out = m;
    }
    return f;
}

static derived_flag_t f_altitude_change(const derived_ctx_t* x, float* out)
{
    float p = 0.0f;
    float t = 0.0f;
    derived_flag_t f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_BAROMETRIC_PRESSURE, &p);
    if (f == DERIVED_FLAG_NONE) {
        f = derived_core_did_value(x, (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP, &t);
    }
    if (f == DERIVED_FLAG_NONE) {
        const float temperature = t + DERIVED_KELVIN_OFFSET;
        if ((!x->c->baro_ref_set) || (p <= 0.0f) || (temperature <= 0.0f)) {
            f = DERIVED_FLAG_INVALID_INPUT;
        } else {
            const float scale_height = (VEHICLE_CL250_DRY_AIR_GAS_CONSTANT_J_PER_KG_K * temperature) / DERIVED_STANDARD_GRAVITY;
            *out = scale_height * logf(x->c->baro_ref / p);
        }
    }
    return f;
}

static derived_flag_t f_temp_rate(const derived_ctx_t* x, uint32_t did, float* out)
{
    win_t w;
    derived_flag_t f = did_window(x, did, 1.0f, true, &w);
    if (f == DERIVED_FLAG_NONE) {
        if (w.slope_ok) {
            *out = w.slope_per_s * DERIVED_SECONDS_PER_MINUTE;
        } else {
            f = DERIVED_FLAG_INSUFFICIENT_SAMPLES;
        }
    }
    return f;
}

/* ---- data_quality_mask (D-104) --------------------------------------------------------------- */

/*
 * The poll bit of a poll period: each period that vehicle_cl250_dids[] uses, with the gen/
 * compile-time copy of one DID that has it (so the period is never written here) and the
 * signal's bit macro. 0 for a period that has no bit.
 */
typedef struct {
    uint32_t period_ms;
    uint32_t bit;
} dq_poll_bit_t;

static uint32_t dq_bit_of_period(uint32_t period_ms)
{
    static const dq_poll_bit_t dq_poll_bits[] = {
        {VEHICLE_CL250_POLL_MS_ENGINE_SPEED, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_110MS},
        {VEHICLE_CL250_POLL_MS_THROTTLE_POS, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_200MS},
        {VEHICLE_CL250_POLL_MS_COOLANT_TEMP, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_800MS},
        {VEHICLE_CL250_POLL_MS_ENGINE_LOAD, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_2000MS},
        {VEHICLE_CL250_POLL_MS_ENGINE_OIL_TEMP, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_5000MS},
        {VEHICLE_CL250_POLL_MS_RUN_TIME_SINCE_START, VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_POLL_10000MS},
    };
    uint32_t bit = 0u;
    for (uint32_t i = 0u; i < ((sizeof dq_poll_bits) / (sizeof dq_poll_bits[0])); i++) {
        if (dq_poll_bits[i].period_ms == period_ms) {
            bit = dq_poll_bits[i].bit;
        }
    }
    return bit;
}

/* True when every DID's poll period has a bit (init checks it: else the signal is not computed). */
bool dq_bits_cover_dids(void)
{
    bool ok = true;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        ok = ok && (dq_bit_of_period(vehicle_cl250_dids[i].poll_period_ms) != 0u);
    }
    return ok;
}

/* A DID that counts toward its poll bit: never received, stale or NRC; stopped and skipped
 * never count. (Unknown tester state is handled by the caller: every DID is bad then.) */
static bool dq_did_bad(const derived_core_t* c, uint32_t i, uint32_t now)
{
    const uint8_t st = c->quality.did[i];
    bool bad;
    if ((st == (uint8_t)DERIVED_QUALITY_STOPPED) || (st == (uint8_t)DERIVED_QUALITY_SKIPPED)) {
        bad = false;
    } else if (st == (uint8_t)DERIVED_QUALITY_NRC) {
        bad = true;
    } else {
        /* the record's own entry, not the ride-local table: quality belongs to the tester */
        bad = (!c->dq_present[i]) ||
              (derived_core_age_of(now, c->dq_stamp_ms[i]) > (uint32_t)vehicle_cl250_dids[i].stale_after_ms);
    }
    return bad;
}

/* The rule of the data_quality_mask description in the defs (D-104). Always a value on its grid. */
static derived_flag_t f_data_quality(const derived_ctx_t* x, float* out)
{
    derived_core_t* const c = x->c;
    const derived_quality_in_t* const q = &c->quality;
    const bool usable = q->known && q->valid &&
                        (derived_core_age_of(x->now, q->stamp_ms) <= (uint32_t)x->spec->stale_after_ms);
    uint32_t mask = 0u;
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        if ((!usable) || dq_did_bad(c, i, x->now)) {
            mask |= dq_bit_of_period(vehicle_cl250_dids[i].poll_period_ms);
        }
    }
    if (usable) {
        if (q->bus_off_count > c->quality_prev_bus_off) {
            mask |= VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_BUS_OFF;
        }
        c->quality_prev_bus_off = q->bus_off_count; /* unknown leaves the remembered count */
        if (q->step_gap_ms > (uint32_t)VEHICLE_CL250_DATA_QUALITY_TESTER_STALL_GAP_MS) {
            mask |= VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_TESTER_STALL;
        }
    } else {
        mask |= VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_BUS_OFF |
                VEHICLE_CL250_DERIVED_BIT_DATA_QUALITY_MASK_TESTER_STALL; /* unknown counts as bad */
    }
    *out = (float)mask;
    return DERIVED_FLAG_NONE;
}

/* Dispatch of the point formulas (gen index to formula). */
derived_flag_t eval_point(const derived_ctx_t* x, float* out)
{
    derived_flag_t f = DERIVED_FLAG_MATH_ERROR;
    switch (x->sig) {
    case VEHICLE_CL250_DERIVED_DATA_QUALITY_MASK:
        f = f_data_quality(x, out);
        break;
    case VEHICLE_CL250_DERIVED_GEARBOX_RATIO_MEASURED:
        f = f_gearbox_ratio(x, out);
        break;
    case VEHICLE_CL250_DERIVED_GEAR_ESTIMATE:
        f = f_gear_estimate(x, out);
        break;
    case VEHICLE_CL250_DERIVED_VEHICLE_ACCEL:
        f = f_vehicle_accel(x, out);
        break;
    case VEHICLE_CL250_DERIVED_VEHICLE_JERK:
        f = f_vehicle_jerk(x, out);
        break;
    case VEHICLE_CL250_DERIVED_ENGINE_TORQUE:
        f = torque_of(x, (uint32_t)VEHICLE_CL250_IDX_ACTUAL_TORQUE_PCT, out);
        break;
    case VEHICLE_CL250_DERIVED_DEMAND_TORQUE:
        f = torque_of(x, (uint32_t)VEHICLE_CL250_IDX_DEMAND_TORQUE_PCT, out);
        break;
    case VEHICLE_CL250_DERIVED_TORQUE_GAP:
        f = f_torque_gap(x, out);
        break;
    case VEHICLE_CL250_DERIVED_ENGINE_POWER:
        f = f_engine_power(x, out);
        break;
    case VEHICLE_CL250_DERIVED_AIR_DENSITY:
        f = f_air_density(x, out);
        break;
    case VEHICLE_CL250_DERIVED_POWER_CORRECTION_FACTOR:
        f = f_power_correction(x, out);
        break;
    case VEHICLE_CL250_DERIVED_AIR_FUEL_RATIO:
        f = f_air_fuel_ratio(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_RATE_MAF:
        f = f_fuel_rate_maf(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_INSTANT:
        f = f_fuel_consumption_instant(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_CONSUMPTION_AVG:
        f = f_fuel_consumption_avg(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_RANGE:
        f = f_fuel_range(x, out);
        break;
    case VEHICLE_CL250_DERIVED_BSFC:
        f = f_bsfc(x, out);
        break;
    case VEHICLE_CL250_DERIVED_FUEL_PER_STROKE:
        f = f_fuel_per_stroke(x, out);
        break;
    case VEHICLE_CL250_DERIVED_TOTAL_FUEL_TRIM:
        f = two_dids(x, (uint32_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1,
                     (uint32_t)VEHICLE_CL250_IDX_LONG_TERM_FUEL_TRIM_B1, true, out);
        break;
    case VEHICLE_CL250_DERIVED_THROTTLE_CHANNEL_RESIDUAL:
        f = f_throttle_channel(x, out);
        break;
    case VEHICLE_CL250_DERIVED_THROTTLE_COMMAND_RESIDUAL:
        f = two_dids(x, (uint32_t)VEHICLE_CL250_IDX_COMMANDED_THROTTLE,
                     (uint32_t)VEHICLE_CL250_IDX_THROTTLE_POS, false, out);
        break;
    case VEHICLE_CL250_DERIVED_GRIP_CHANNEL_RESIDUAL:
        f = two_dids(x, (uint32_t)VEHICLE_CL250_IDX_ACCEL_PEDAL_POS_D,
                     (uint32_t)VEHICLE_CL250_IDX_RELATIVE_ACCEL_PEDAL_POS, false, out);
        break;
    case VEHICLE_CL250_DERIVED_ALTITUDE_CHANGE:
        f = f_altitude_change(x, out);
        break;
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
        f = f_temp_rate(x, (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP, out);
        break;
    case VEHICLE_CL250_DERIVED_OIL_TEMP_RATE:
        f = f_temp_rate(x, (uint32_t)VEHICLE_CL250_IDX_ENGINE_OIL_TEMP, out);
        break;
    default:
        f = derived_d103_eval_point(x, out);
        break;
    }
    return f;
}
