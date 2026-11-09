/*
 * Class-B ride accumulators and closed values (D-070, D-087). See derived_ride.h. The
 * formulas follow vehicle-server src/vehicle_server/derived/summary.py.
 */
#include "features/derived/derived_ride.h"

#include "vehicle_cl250.h"
#include "vehicle_cl250_constants.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__)
#error "derived_ride.c must not be built with -ffast-math"
#endif

/* The reference's unit conversion of the fuel integral: L/h to L/s (summary.py
 * SECONDS_PER_HOUR) and the consumption factor (LITRES_PER_100KM_FACTOR); ms to s. */
#define RIDE_SECONDS_PER_HOUR (3600.0)
#define RIDE_L_PER_100KM (100.0)
#define RIDE_MS_PER_S (1000.0)

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

static bool ride_finite(float x)
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

_Static_assert(DERIVED_RIDE_TAIL_POOL == 388u, "the waiting-row pool");

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

static raw_t raw_of(const derived_ride_acc_t* a, uint32_t sig)
{
    return (sig == VEHICLE_CL250_DERIVED_RIDE_FUEL_CONSUMPTION) ? consumption(a) : raw_simple(a, sig);
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
