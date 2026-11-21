/*
 * D-079 health windows (pure), summaries: the first qualifying instance and the summary values. See derived_windows.h for the rules and the reference
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
#error "derived_windows_value.c must not be built with -ffast-math"
#endif
#if defined(__GNUC__) && !defined(__NO_MATH_ERRNO__)
#error "derived_windows_value.c needs -fno-math-errno"
#endif

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
    case VEHICLE_CL250_DERIVED_WINDOW_CRUISE_FINAL_DRIVE_RESIDUAL:
        f = (st->ratio_n > 0u) ? DERIVED_FLAG_NONE : DERIVED_FLAG_NO_DATA;
        if (st->ratio_n > 0u) {
            const uint32_t mid = st->ratio_n / 2u;
            const double median = ((st->ratio_n % 2u) != 0u)
                                      ? (double)a->ratio_buf[mid]
                                      : (((double)a->ratio_buf[mid - 1u] + (double)a->ratio_buf[mid]) / 2.0);
            *out = median - (double)VEHICLE_CL250_SECONDARY_REDUCTION_RATIO;
        }
        break;
    case VEHICLE_CL250_DERIVED_WINDOW_CRUISE_VE_RESIDUAL:
        f = mean_of(st, DERIVED_WIN_MEAN_VE, &m1);
        if (f == DERIVED_FLAG_NONE) {
            *out = m1 - (double)VEHICLE_CL250_INTAKE_FLOW_RESIDUAL_VE_REFERENCE_PCT;
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
