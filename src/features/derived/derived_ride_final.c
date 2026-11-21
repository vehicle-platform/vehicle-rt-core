/*
 * Class-B ride summaries, closed values (D-070, D-087): the raw result of each summary and its range
 * check. See derived_ride.h. The formulas follow vehicle-server src/vehicle_server/derived/summary.py.
 * Split from derived_ride.c (the accumulators); the units talk through derived_ride_internal.h only.
 */
#include "features/derived/derived_ride.h"
#include "features/derived/derived_ride_internal.h"

#include "vehicle_cl250.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__)
#error "derived_ride_final.c must not be built with -ffast-math"
#endif

/* The reference's consumption factor (LITRES_PER_100KM_FACTOR). */
#define RIDE_L_PER_100KM (100.0)

/* Raw result of a summary: a flag, or NONE with a value. */
typedef struct {
    derived_flag_t flag;
    double value;
} raw_t;

static raw_t raw_value(double v)
{
    const raw_t r = {DERIVED_FLAG_NONE, v};
    return r;
}

static raw_t raw_flag(derived_flag_t f)
{
    const raw_t r = {f, 0.0};
    return r;
}

static raw_t last_valid(bool has, float v)
{
    return has ? raw_value((double)v) : raw_flag(DERIVED_FLAG_INVALID_INPUT);
}

/* The range check and the output row, as the reference's _checked(). */
static void conclude(uint32_t sig, raw_t raw, derived_out_t* out)
{
    const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
    derived_flag_t f = raw.flag;
    const float value = (float)raw.value;
    if (f == DERIVED_FLAG_NONE) {
        if (!ride_finite(value)) {
            f = DERIVED_FLAG_MATH_ERROR;
        } else if ((value < spec->min) || (value > spec->max)) {
            f = DERIVED_FLAG_OUT_OF_RANGE; /* never clamped */
        } else {
            /* in range */
        }
    }
    out->unranged = (raw.flag == DERIVED_FLAG_NONE) ? value : 0.0f;
    out->flag = f;
    out->status = (f == DERIVED_FLAG_NONE) ? DERIVED_VALID : DERIVED_INVALID;
    out->value = (f == DERIVED_FLAG_NONE) ? value : 0.0f;
}

static raw_t raw_simple(const derived_ride_acc_t* a, uint32_t sig);

/* ride_fuel_consumption reads two finalized class-B results (invalid or out of range is
 * "invalid input"), distance <= 0 is standstill. */
static raw_t consumption(const derived_ride_acc_t* a)
{
    derived_out_t used;
    derived_out_t dist;
    (void)memset(&used, 0, sizeof used);
    (void)memset(&dist, 0, sizeof dist);
    conclude(VEHICLE_CL250_DERIVED_RIDE_FUEL_USED, raw_simple(a, VEHICLE_CL250_DERIVED_RIDE_FUEL_USED), &used);
    conclude(VEHICLE_CL250_DERIVED_RIDE_DISTANCE, raw_simple(a, VEHICLE_CL250_DERIVED_RIDE_DISTANCE), &dist);
    raw_t r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
    if ((used.status == DERIVED_VALID) && (dist.status == DERIVED_VALID)) {
        if (dist.value <= 0.0f) {
            r = raw_flag(DERIVED_FLAG_STANDSTILL);
        } else {
            r = raw_value(((double)used.value / (double)dist.value) * RIDE_L_PER_100KM);
        }
    }
    return r;
}

static raw_t flag_time_raw(const derived_ride_acc_t* a, uint32_t idx)
{
    const derived_ride_flag_time_t* const f = &a->flag_time[idx];
    return f->any_valid ? raw_value(f->total_s) : raw_flag(DERIVED_FLAG_INVALID_INPUT);
}

/* ride_warmup_time (D-103): first ENGINE_SPEED > 0 to the first COOLANT >= operating. */
static raw_t warmup_raw(const derived_ride_acc_t* a)
{
    raw_t r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
    if (a->has_coolant && a->has_rpm_start) {
        if (a->first_coolant_warm) {
            r = raw_flag(DERIVED_FLAG_STARTS_WARM);
        } else if (a->has_warm) {
            const uint32_t d = a->warm_ms - a->rpm_start_ms;
            r = (d > 0x7FFFFFFFu) ? raw_flag(DERIVED_FLAG_STARTS_WARM) /* before the engine start */
                                  : raw_value((double)d / RIDE_MS_PER_S);
        } else {
            r = raw_flag(DERIVED_FLAG_NEVER_WARM);
        }
    }
    return r;
}

static raw_t raw_simple(const derived_ride_acc_t* a, uint32_t sig)
{
    raw_t r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
    switch (sig) {
    case VEHICLE_CL250_DERIVED_THROTTLE_TORQUE_LAG:
        r = raw_flag(DERIVED_FLAG_POLL_TOO_SLOW); /* only slotted computed when the gen/ polls cannot resolve it */
        break;
    case VEHICLE_CL250_DERIVED_RIDE_DISTANCE:
        r = last_valid(a->has_trip_distance, a->trip_distance);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_TIME:
        r = last_valid(a->has_trip_time, a->trip_time);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_IDLE_TIME:
        r = last_valid(a->has_idle_time, a->idle_time);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_STOP_COUNT:
        r = raw_value((double)a->stop_count);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_FUEL_USED:
        r = a->fuel_seen ? raw_value(a->fuel_prefix) : raw_flag(DERIVED_FLAG_INVALID_INPUT);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_LTFT_MEAN:
        if (a->ltft_n > 0u) {
            r = raw_value(a->ltft_sum / (double)a->ltft_n);
        }
        break;
    case VEHICLE_CL250_DERIVED_RIDE_STFT_STD:
        if (a->stft_n >= 2u) {
            r = raw_value(sqrt(a->stft_m2 / (double)a->stft_n));
        } else if (a->stft_n == 1u) {
            r = raw_flag(DERIVED_FLAG_INSUFFICIENT_SAMPLES);
        } else {
            r = raw_flag(DERIVED_FLAG_INVALID_INPUT);
        }
        break;
    case VEHICLE_CL250_DERIVED_RIDE_COOLANT_RATE_MAX:
        r = last_valid(a->has_coolant_rate, a->coolant_rate_max);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP:
        r = last_valid(a->has_coolant, a->coolant_first);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_START_INTAKE_TEMP:
        r = last_valid(a->has_intake, a->intake_first);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP:
        r = last_valid(a->has_coolant, a->coolant_last);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_HEAT_SOAK_MAX:
        r = last_valid(a->has_heat_soak, a->heat_soak_max);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_CRANKING_DIP:
        r = last_valid(a->has_cranking_dip, a->cranking_dip);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_REST_VOLTAGE:
        r = last_valid(a->has_rest_voltage, a->rest_voltage);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_COLD_ABUSE_TIME:
        r = flag_time_raw(a, 0u);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_OVER_REV_TIME:
        r = flag_time_raw(a, 1u);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_LUGGING_TIME:
        r = flag_time_raw(a, 2u);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_STD:
        r = last_valid(a->has_idle_rpm_std, a->idle_rpm_std_max);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_DRIFT:
        r = last_valid(a->has_idle_rpm_drift, a->idle_rpm_drift);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_SHIFT_COUNT:
        r = raw_value((double)a->shift_count);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_LAUNCH_COUNT:
        r = raw_value((double)a->launch_count);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_WARMUP_TIME:
        r = warmup_raw(a);
        break;
    case VEHICLE_CL250_DERIVED_RIDE_THERMAL_CYCLE_COUNT:
        r = raw_value((double)a->thermal_cycles);
        break;
    default: /* VEHICLE_CL250_DERIVED_RIDE_FAN_ON_TEMPERATURE */
        r = a->fan_found ? raw_value((double)a->fan_temp) : raw_flag(DERIVED_FLAG_NO_FAN_STEP);
        break;
    }
    return r;
}

/* A D-079 window summary: counts are always a value, the rest is invalid without a qualifying
 * instance (NO_WINDOW) or without samples (NO_DATA, INSUFFICIENT_SAMPLES). */
static raw_t window_raw(const derived_ride_acc_t* a, uint32_t sig)
{
    double v = 0.0;
    const derived_flag_t f = derived_windows_value(&a->win, sig, &v);
    return (f == DERIVED_FLAG_NONE) ? raw_value(v) : raw_flag(f);
}

static raw_t raw_of(const derived_ride_acc_t* a, uint32_t sig)
{
    raw_t r;
    if (sig == VEHICLE_CL250_DERIVED_RIDE_FUEL_CONSUMPTION) {
        r = consumption(a);
    } else if (derived_windows_has_summary(sig)) {
        r = window_raw(a, sig);
    } else {
        r = raw_simple(a, sig);
    }
    return r;
}

void derived_ride_finalize(const derived_ride_acc_t* a, uint32_t sig, derived_out_t* out)
{
    if ((a == NULL) || (out == NULL)) {
        return;
    }
    if (derived_ride_slot(sig) == DERIVED_RIDE_NO_SLOT) {
        out->status = DERIVED_NOT_COMPUTED;
        out->flag = DERIVED_FLAG_NONE;
        out->value = 0.0f;
        out->unranged = 0.0f;
        return;
    }
    conclude(sig, raw_of(a, sig), out);
}
