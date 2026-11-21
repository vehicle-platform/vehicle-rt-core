/*
 * Class-B ride accumulators and closed values (D-070, D-087). See derived_ride.h. The
 * formulas follow vehicle-server src/vehicle_server/derived/summary.py. The closed values (the summary
 * formulas) are derived_ride_final.c; the units talk through derived_ride_internal.h only.
 */
#include "features/derived/derived_ride.h"
#include "features/derived/derived_ride_internal.h"

#include "vehicle_cl250.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__)
#error "derived_ride.c must not be built with -ffast-math"
#endif

/* The reference's unit conversion of the fuel integral: L/h to L/s (summary.py SECONDS_PER_HOUR). */
#define RIDE_SECONDS_PER_HOUR (3600.0)

static const uint16_t slot_signal[DERIVED_RIDE_SLOTS] = {
    VEHICLE_CL250_DERIVED_RIDE_DISTANCE,          VEHICLE_CL250_DERIVED_RIDE_TIME,
    VEHICLE_CL250_DERIVED_RIDE_IDLE_TIME,         VEHICLE_CL250_DERIVED_RIDE_STOP_COUNT,
    VEHICLE_CL250_DERIVED_RIDE_FUEL_USED,         VEHICLE_CL250_DERIVED_RIDE_FUEL_CONSUMPTION,
    VEHICLE_CL250_DERIVED_RIDE_LTFT_MEAN,         VEHICLE_CL250_DERIVED_RIDE_STFT_STD,
    VEHICLE_CL250_DERIVED_RIDE_COOLANT_RATE_MAX,  VEHICLE_CL250_DERIVED_RIDE_START_COOLANT_TEMP,
    VEHICLE_CL250_DERIVED_RIDE_START_INTAKE_TEMP, VEHICLE_CL250_DERIVED_RIDE_END_COOLANT_TEMP,
    /* the 13 D-103 summaries (LR21b) */
    VEHICLE_CL250_DERIVED_RIDE_HEAT_SOAK_MAX,     VEHICLE_CL250_DERIVED_RIDE_CRANKING_DIP,
    VEHICLE_CL250_DERIVED_RIDE_REST_VOLTAGE,      VEHICLE_CL250_DERIVED_RIDE_COLD_ABUSE_TIME,
    VEHICLE_CL250_DERIVED_RIDE_OVER_REV_TIME,     VEHICLE_CL250_DERIVED_RIDE_LUGGING_TIME,
    VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_STD,      VEHICLE_CL250_DERIVED_RIDE_IDLE_RPM_DRIFT,
    VEHICLE_CL250_DERIVED_RIDE_SHIFT_COUNT,       VEHICLE_CL250_DERIVED_RIDE_LAUNCH_COUNT,
    VEHICLE_CL250_DERIVED_RIDE_WARMUP_TIME,       VEHICLE_CL250_DERIVED_RIDE_THERMAL_CYCLE_COUNT,
    VEHICLE_CL250_DERIVED_RIDE_FAN_ON_TEMPERATURE,
    /* the 17 D-079 window summaries */
    VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_REST_VOLTAGE,       VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_SENSOR_SPREAD,
    VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_MAP_BARO_DELTA,     VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_WARM_RESTART,
    VEHICLE_CL250_DERIVED_WINDOW_KEY_ON_COUNT,              VEHICLE_CL250_DERIVED_WINDOW_CRANK_COUNT,
    VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_SPREAD,      VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_RPM_DRIFT,
    VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_MAP_SPREAD,      VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_MEAN,
    VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_STFT_STD,        VEHICLE_CL250_DERIVED_WINDOW_WARM_IDLE_COUNT,
    VEHICLE_CL250_DERIVED_WINDOW_CRUISE_COUNT,              VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_TAU,
    VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_START,   VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COOLANT_END,
    VEHICLE_CL250_DERIVED_WINDOW_POST_STOP_COUNT,
    /* D-106: the two W4 cruise residuals and the resolvability rule of throttle_torque_lag */
    VEHICLE_CL250_DERIVED_WINDOW_CRUISE_FINAL_DRIVE_RESIDUAL, VEHICLE_CL250_DERIVED_WINDOW_CRUISE_VE_RESIDUAL,
    VEHICLE_CL250_DERIVED_THROTTLE_TORQUE_LAG,
};

uint32_t derived_ride_slot(uint32_t sig)
{
    uint32_t slot = DERIVED_RIDE_NO_SLOT;
    for (uint32_t i = 0u; i < DERIVED_RIDE_SLOTS; i++) {
        if ((uint32_t)slot_signal[i] == sig) {
            slot = i;
        }
    }
    return slot;
}

void derived_ride_init(derived_ride_t* r)
{
    if (r == NULL) {
        return;
    }
    (void)memset(r, 0, sizeof *r);
    for (uint32_t i = 0u; i < DERIVED_RIDE_SLOTS; i++) {
        r->prev[i].status = DERIVED_NO_DATA;
        r->prev[i].flag = DERIVED_FLAG_NONE;
    }
}

void derived_ride_begin(derived_ride_t* r, uint16_t counter)
{
    if (r == NULL) {
        return;
    }
    (void)memset(&r->acc, 0, sizeof r->acc);
    r->counter = counter;
    r->open = true;
}

bool ride_finite(float x)
{
    return isfinite(x) != 0;
}

/* The fuel integral: a sample ends the open segment (at most the DID's own stale_after_ms
 * long, so a gap adds nothing) and starts the next one. */
static void fuel_sample(derived_ride_acc_t* a, float value, uint32_t stamp_ms)
{
    const int32_t limit = (int32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_FUEL_RATE].stale_after_ms;
    if (a->fuel_seg_active) {
        const uint32_t run = stamp_ms - a->fuel_seg_start_ms;
        int32_t span = (run > 0x7FFFFFFFu) ? 0 : (int32_t)run;
        if (span > a->fuel_seg_remaining_ms) {
            span = a->fuel_seg_remaining_ms;
        }
        if (span < 0) {
            span = 0;
        }
        a->fuel_prefix += a->fuel_rate * ((double)span / RIDE_MS_PER_S);
        a->fuel_seg_active = false;
    }
    a->fuel_seen = true;
    if (ride_finite(value)) {
        a->fuel_rate = (double)value / RIDE_SECONDS_PER_HOUR;
        a->fuel_seg_start_ms = stamp_ms;
        a->fuel_seg_remaining_ms = limit; /* the sample is fresh at its own stamp */
        a->fuel_seg_active = true;
    }
}

/* The DID sample `stamp` is fresh at time `t` (latest sample at or before t, within the
 * DID's own stale_after_ms). */
static bool fresh_at(bool has, uint32_t stamp, uint32_t t, uint16_t limit)
{
    const uint32_t age = t - stamp;
    return has && (age <= 0x7FFFFFFFu) && (age <= (uint32_t)limit);
}

/* The coolant fresh at `t` when the machine is at rest, in the idle band and above the
 * minimum (the reference's resting_idle()), else false. */
static bool resting_idle(const derived_ride_acc_t* a, uint32_t t, float* coolant)
{
    bool ok = fresh_at(a->has_speed, a->speed_ms, t, vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms) &&
              fresh_at(a->has_rpm, a->rpm_ms, t, vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms) &&
              fresh_at(a->has_coolant_now, a->coolant_ms, t, vehicle_cl250_dids[VEHICLE_CL250_IDX_COOLANT_TEMP].stale_after_ms);
    if (ok) {
        ok = (a->speed_v <= VEHICLE_CL250_DERIVED_MOVING_SPEED_MIN_KMH) &&
             (a->rpm_v >= VEHICLE_CL250_DERIVED_IDLE_RPM_MIN) && (a->rpm_v <= VEHICLE_CL250_DERIVED_IDLE_RPM_MAX) &&
             (a->coolant_v > VEHICLE_CL250_RIDE_FAN_ON_TEMPERATURE_COOLANT_MIN_DEGC);
    }
    if (ok) {
        *coolant = a->coolant_v;
    }
    return ok;
}

void derived_ride_settle(derived_ride_acc_t* a)
{
    if ((a == NULL) || (!a->batt_pending)) {
        return;
    }
    a->batt_pending = false;
    float c2 = 0.0f;
    const bool rest2 = resting_idle(a, a->batt_pend_ms, &c2);
    const uint32_t gap = a->batt_pend_ms - a->batt_prev_ms;
    if ((!a->fan_found) && a->has_batt_prev && a->batt_prev_rest && rest2 &&
        (gap <= (uint32_t)vehicle_cl250_dids[VEHICLE_CL250_IDX_BATTERY_VOLTAGE].stale_after_ms) &&
        (((double)a->batt_prev_v - (double)a->batt_pend_v) >= (double)VEHICLE_CL250_RIDE_FAN_ON_TEMPERATURE_VOLTAGE_STEP_V)) {
        a->fan_found = true;
        a->fan_temp = c2;
    }
    a->batt_prev_v = a->batt_pend_v;
    a->batt_prev_ms = a->batt_pend_ms;
    a->batt_prev_rest = rest2;
    a->has_batt_prev = true;
}

/* A sample of a DID the fan-on step reads: the pending battery sample settles first when
 * this one is later than it. */
static void settle_before(derived_ride_acc_t* a, uint32_t stamp_ms)
{
    if (a->batt_pending && (stamp_ms != a->batt_pend_ms) && (((uint32_t)(stamp_ms - a->batt_pend_ms)) <= 0x7FFFFFFFu)) {
        derived_ride_settle(a);
    }
}

static void coolant_sample(derived_ride_acc_t* a, float value, uint32_t stamp_ms)
{
    settle_before(a, stamp_ms);
    if (!a->has_coolant) {
        a->coolant_first = value;
        a->has_coolant = true;
        a->first_coolant_warm = value >= VEHICLE_CL250_DERIVED_OPERATING_COOLANT_DEGC;
    }
    a->coolant_last = value;
    a->coolant_v = value;
    a->coolant_ms = stamp_ms;
    a->has_coolant_now = true;
    if ((!a->has_warm) && (value >= VEHICLE_CL250_DERIVED_OPERATING_COOLANT_DEGC)) {
        a->warm_ms = stamp_ms;
        a->has_warm = true;
    }
    if (value < VEHICLE_CL250_DERIVED_COLD_COOLANT_DEGC) {
        a->was_cold = true;
    } else if ((value >= VEHICLE_CL250_DERIVED_OPERATING_COOLANT_DEGC) && a->was_cold) {
        if (a->thermal_cycles < UINT32_MAX) {
            a->thermal_cycles++;
        }
        a->was_cold = false;
    } else {
        /* between the two thresholds the state is kept */
    }
}

void derived_ride_sample(derived_ride_acc_t* a, uint32_t did, float value, uint32_t stamp_ms)
{
    if (a == NULL) {
        return;
    }
    switch (did) {
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_FUEL_RATE:
        fuel_sample(a, value, stamp_ms);
        break;
    case (uint32_t)VEHICLE_CL250_IDX_LONG_TERM_FUEL_TRIM_B1:
        a->ltft_n++;
        a->ltft_sum += (double)value;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_SHORT_TERM_FUEL_TRIM_B1: {
        a->stft_n++;
        const double delta = (double)value - a->stft_mean;
        a->stft_mean += delta / (double)a->stft_n;
        a->stft_m2 += delta * ((double)value - a->stft_mean);
        break;
    }
    case (uint32_t)VEHICLE_CL250_IDX_COOLANT_TEMP:
        coolant_sample(a, value, stamp_ms);
        break;
    case (uint32_t)VEHICLE_CL250_IDX_INTAKE_AIR_TEMP:
        if (!a->has_intake) {
            a->intake_first = value;
            a->has_intake = true;
        }
        break;
    case (uint32_t)VEHICLE_CL250_IDX_ENGINE_SPEED:
        settle_before(a, stamp_ms);
        a->rpm_v = value;
        a->rpm_ms = stamp_ms;
        a->has_rpm = true;
        if ((!a->has_rpm_start) && (value > 0.0f)) {
            a->rpm_start_ms = stamp_ms;
            a->has_rpm_start = true;
        }
        break;
    case (uint32_t)VEHICLE_CL250_IDX_VEHICLE_SPEED:
        settle_before(a, stamp_ms);
        a->speed_v = value;
        a->speed_ms = stamp_ms;
        a->has_speed = true;
        break;
    case (uint32_t)VEHICLE_CL250_IDX_BATTERY_VOLTAGE:
        derived_ride_settle(a); /* an earlier battery sample of the same step */
        a->batt_pend_v = value;
        a->batt_pend_ms = stamp_ms;
        a->batt_pending = true;
        break;
    default:
        break; /* no summary reads this DID */
    }
}

/* Index of a flag-time signal, else DERIVED_RIDE_FLAG_TIMES. */
static uint32_t flag_time_index(uint32_t sig)
{
    uint32_t i = DERIVED_RIDE_FLAG_TIMES;
    if (sig == VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE) {
        i = 0u;
    } else if (sig == VEHICLE_CL250_DERIVED_OVER_REV) {
        i = 1u;
    } else if (sig == VEHICLE_CL250_DERIVED_LUGGING) {
        i = 2u;
    } else {
        /* not a flag-time signal */
    }
    return i;
}

/* One row of a class-A flag output (_flag_seconds): the previous row, when valid with value 1,
 * counts until this one but never longer than the signal's stale_after_ms. */
static void flag_time_row(derived_ride_flag_time_t* f, uint32_t sig, uint32_t stamp_ms, bool valid, float value)
{
    if (f->has_prev && f->prev_one) {
        const uint32_t dt = stamp_ms - f->prev_ms;
        const uint32_t limit = (uint32_t)vehicle_cl250_derived[sig].stale_after_ms;
        f->total_s += (double)((dt < limit) ? dt : limit) / RIDE_MS_PER_S;
    }
    f->prev_ms = stamp_ms;
    f->has_prev = true;
    f->prev_one = valid && (value >= 1.0f) && (value <= 1.0f);
    f->any_valid = f->any_valid || valid;
}

static void keep_max(float* best, bool* has, float value)
{
    if ((!*has) || (value > *best)) {
        *best = value;
    }
    *has = true;
}

static void keep_first(float* kept, bool* has, float value)
{
    if (!*has) {
        *kept = value;
    }
    *has = true;
}

static void apply_row(derived_ride_acc_t* a, uint32_t sig, uint32_t stamp_ms, bool valid, float value)
{
    const uint32_t fi = flag_time_index(sig);
    if (fi < DERIVED_RIDE_FLAG_TIMES) {
        flag_time_row(&a->flag_time[fi], sig, stamp_ms, valid, value);
    }
    if (!valid) {
        return;
    }
    switch (sig) {
    case VEHICLE_CL250_DERIVED_TRIP_DISTANCE:
        a->trip_distance = value;
        a->has_trip_distance = true;
        break;
    case VEHICLE_CL250_DERIVED_TRIP_TIME:
        a->trip_time = value;
        a->has_trip_time = true;
        break;
    case VEHICLE_CL250_DERIVED_IDLE_TIME:
        a->idle_time = value;
        a->has_idle_time = true;
        break;
    case VEHICLE_CL250_DERIVED_STOP_EVENT:
        if (a->stop_count < UINT32_MAX) {
            a->stop_count++;
        }
        break;
    case VEHICLE_CL250_DERIVED_COOLANT_RATE:
        keep_max(&a->coolant_rate_max, &a->has_coolant_rate, value);
        break;
    case VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA:
        keep_max(&a->heat_soak_max, &a->has_heat_soak, value);
        break;
    case VEHICLE_CL250_DERIVED_CRANKING_DIP:
        keep_first(&a->cranking_dip, &a->has_cranking_dip, value);
        break;
    case VEHICLE_CL250_DERIVED_BATTERY_REST_VOLTAGE:
        keep_first(&a->rest_voltage, &a->has_rest_voltage, value);
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_STD:
        keep_max(&a->idle_rpm_std_max, &a->has_idle_rpm_std, value);
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT:
        a->idle_rpm_drift = value;
        a->has_idle_rpm_drift = true;
        break;
    case VEHICLE_CL250_DERIVED_SHIFT_EVENT:
        if (a->shift_count < UINT32_MAX) {
            a->shift_count++;
        }
        break;
    case VEHICLE_CL250_DERIVED_LAUNCH_EVENT:
        if (a->launch_count < UINT32_MAX) {
            a->launch_count++;
        }
        break;
    default:
        break;
    }
}

static uint32_t tail_index(uint32_t sig)
{
    uint32_t i = DERIVED_RIDE_TAILS;
    switch (sig) {
    case VEHICLE_CL250_DERIVED_HEAT_SOAK_DELTA:
        i = 0u;
        break;
    case VEHICLE_CL250_DERIVED_COLD_ENGINE_ABUSE:
        i = 1u;
        break;
    case VEHICLE_CL250_DERIVED_OVER_REV:
        i = 2u;
        break;
    case VEHICLE_CL250_DERIVED_LUGGING:
        i = 3u;
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_STD:
        i = 4u;
        break;
    case VEHICLE_CL250_DERIVED_IDLE_RPM_DRIFT:
        i = 5u;
        break;
    default:
        break;
    }
    return i;
}

bool derived_ride_is_tail_signal(uint32_t sig)
{
    return tail_index(sig) < DERIVED_RIDE_TAILS;
}

_Static_assert(DERIVED_RIDE_TAIL_POOL == 306u, "the waiting-row pool");

/* Caps in the order of tail_index(); the slices follow each other in the pool. */
static const uint16_t tail_caps[DERIVED_RIDE_TAILS] = {
    DERIVED_RIDE_TAIL_CAP_HEAT_SOAK, DERIVED_RIDE_TAIL_CAP_COLD_ABUSE, DERIVED_RIDE_TAIL_CAP_OVER_REV,
    DERIVED_RIDE_TAIL_CAP_LUGGING,   DERIVED_RIDE_TAIL_CAP_IDLE_STD,   DERIVED_RIDE_TAIL_CAP_IDLE_DRIFT,
};

uint32_t derived_ride_tail_cap(uint32_t sig)
{
    const uint32_t ti = tail_index(sig);
    return (ti < DERIVED_RIDE_TAILS) ? (uint32_t)tail_caps[ti] : 0u;
}

uint32_t derived_ride_tail_need(uint32_t sig)
{
    uint32_t need = 0u;
    if (tail_index(sig) < DERIVED_RIDE_TAILS) {
        const vehicle_cl250_derived_t* const spec = &vehicle_cl250_derived[sig];
        uint32_t stale = 0u;
        for (uint32_t k = 0u; (k < (uint32_t)spec->input_count) && (k < VEHICLE_CL250_DERIVED_INPUT_MAX); k++) {
            const uint32_t idx = spec->inputs[k].index;
            uint32_t limit = 0u;
            if (spec->inputs[k].kind == VEHICLE_CL250_DERIVED_INPUT_DID) {
                limit = (idx < VEHICLE_CL250_DID_COUNT) ? (uint32_t)vehicle_cl250_dids[idx].stale_after_ms : 0u;
            } else {
                limit = (idx < VEHICLE_CL250_DERIVED_COUNT) ? (uint32_t)vehicle_cl250_derived[idx].stale_after_ms : 0u;
            }
            stale = (limit > stale) ? limit : stale;
        }
        const uint32_t period = spec->period_ms;
        need = (period == 0u) ? UINT32_MAX : (((stale + period) - 1u) / period) + 2u;
    }
    return need;
}

bool derived_ride_tail_fits(uint32_t sig)
{
    return derived_ride_tail_cap(sig) >= derived_ride_tail_need(sig);
}

static derived_ride_row_t* tail_row(derived_ride_acc_t* a, uint32_t ti, uint32_t offset)
{
    uint32_t base = 0u;
    for (uint32_t i = 0u; i < ti; i++) {
        base += tail_caps[i];
    }
    return &a->tail_pool[base + (((uint32_t)a->tail[ti].head + offset) % (uint32_t)tail_caps[ti])];
}

/* Oldest waiting row out of the ring into the summaries. */
static void tail_pop(derived_ride_acc_t* a, uint32_t ti, uint32_t sig)
{
    derived_ride_tail_t* const t = &a->tail[ti];
    const derived_ride_row_t r = *tail_row(a, ti, 0u);
    t->head = (uint16_t)(((uint32_t)t->head + 1u) % (uint32_t)tail_caps[ti]);
    t->count--;
    const bool valid = isfinite(r.value) != 0;
    apply_row(a, sig, r.tick, valid, valid ? r.value : 0.0f);
}

void derived_ride_advance(derived_ride_acc_t* a, uint32_t sig, uint32_t bound_ms)
{
    const uint32_t ti = (a == NULL) ? DERIVED_RIDE_TAILS : tail_index(sig);
    if (ti < DERIVED_RIDE_TAILS) {
        while ((a->tail[ti].count > 0u) && ((uint32_t)(bound_ms - tail_row(a, ti, 0u)->tick) <= 0x7FFFFFFFu)) {
            tail_pop(a, ti, sig);
        }
    }
}

void derived_ride_class_a(derived_ride_acc_t* a, uint32_t sig, uint32_t stamp_ms, bool valid, float value)
{
    if (a == NULL) {
        return;
    }
    const uint32_t ti = tail_index(sig);
    if (ti < DERIVED_RIDE_TAILS) {
        if (a->tail[ti].count >= tail_caps[ti]) {
            tail_pop(a, ti, sig); /* full: the oldest row is past every input's stale limit */
        }
        derived_ride_row_t* const r = tail_row(a, ti, a->tail[ti].count);
        r->tick = stamp_ms;
        r->value = valid ? value : NAN;
        a->tail[ti].count++;
    } else {
        apply_row(a, sig, stamp_ms, valid, value);
    }
}

void derived_ride_close(derived_ride_t* r, uint32_t now_ms)
{
    if (r == NULL) {
        return;
    }
    for (uint32_t i = 0u; i < DERIVED_RIDE_SLOTS; i++) {
        derived_out_t* const o = &r->prev[i];
        derived_ride_finalize(&r->acc, (uint32_t)slot_signal[i], o);
        o->stamp_ms = now_ms;
        o->seq++;
    }
    r->prev_counter = r->counter;
    r->have_prev = true;
    r->open = false;
    r->closed_count++;
}
