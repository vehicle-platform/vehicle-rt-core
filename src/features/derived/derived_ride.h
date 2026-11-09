#ifndef FEATURES_DERIVED_DERIVED_RIDE_H
#define FEATURES_DERIVED_DERIVED_RIDE_H

/*
 * Class-B ride summaries (D-070 item 2, parts 2-3; D-087 item 2). A ride is one D-075 stop
 * epoch of the vehicle tester: the core opens it when the epoch starts and closes it when the
 * epoch ends or another one starts. This file holds the pure accumulators and the closed
 * values; the epoch handling and the class-A reset are in derived_core.c.
 *
 * Computed signals (decided from the gen/ table: class B, rt_core_ready, scalar, inputs only
 * DIDs or computed signals, and a formula here): ride_distance, ride_time, ride_idle_time,
 * ride_stop_count, ride_fuel_used, ride_fuel_consumption, ride_ltft_mean, ride_stft_std,
 * ride_coolant_rate_max, ride_start_coolant_temp, ride_start_intake_temp,
 * ride_end_coolant_temp, and the 13 D-103 summaries (LR21b): ride_heat_soak_max,
 * ride_cranking_dip, ride_rest_voltage, ride_cold_abuse_time, ride_over_rev_time,
 * ride_lugging_time, ride_idle_rpm_std, ride_idle_rpm_drift, ride_shift_count,
 * ride_launch_count, ride_warmup_time, ride_thermal_cycle_count, ride_fan_on_temperature,
 * and the 17 D-079 window summaries (derived_windows.h): the statistics of the first qualifying
 * instance of W1..W5 and the qualifying counts (window_key_on_*, window_crank_count,
 * window_warm_idle_*, window_cruise_count, window_post_stop_*).
 * ride_aggressiveness_score and throttle_torque_lag need every sample of the ride (a p95 and a
 * median over every step): NOT_COMPUTED, DERIVED_NC_UNBOUNDED. Binned, per-gear, sensor-input,
 * the 12 window signals the reference does not compute and the other class-B signals are NOT_COMPUTED with a reason.
 *
 * The state is a handful of scalars for the open ride plus one closed record (the previous
 * ride) with one output row per computed signal: no heap, no unbounded loop. The time
 * integral and the means keep `double` accumulators (a long ride would lose small
 * increments in float32; the M7 has a double-precision FPU); results are float32.
 *
 * Semantics follow the vehicle-server reference (derived/summary.py): "last valid" signals take
 * the last valid class-A output of the ride; means and the first/last values are over every
 * DID sample of the ride; the fuel integral holds a sample until the next one but never
 * beyond that DID's own stale_after_ms (D-087), and the segment still open at the close adds
 * nothing. A power-down close is not handled here (no persistence yet, D-045): the epoch
 * flag is the only ride signal.
 */

#include "features/derived/derived_types.h"
#include "features/derived/derived_windows.h"
#include "vehicle_cl250_derived.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DERIVED_RIDE_SLOTS (42u)
#define DERIVED_RIDE_FLAG_TIMES (3u) /* cold abuse, over rev, lugging */
/* Grid class-A outputs whose rows wait for the reference's grid end (see derived_ride_advance). */
#define DERIVED_RIDE_TAILS (6u)
/* Waiting-row capacity per signal. A row can be valid only while every input is fresh, so a
 * signal needs ceil(max over its inputs of stale_after_ms / period_ms) + 2 rows (the +2 as in
 * the DERIVED_WIN_*_CAP): an older row is past every input's stale limit, which makes it an
 * invalid row in the reference too. gen/ exports period and stale only as table data, so
 * derived_ride_tail_fits() checks the caps against it at init (a too small cap makes the
 * signal NOT_COMPUTED, DERIVED_NC_SIZING). Values for defs v0.24.0:
 *   heat_soak_delta     intake / ambient 15000 ms / 110 ms -> 137 + 2 = 139
 *   cold_engine_abuse   oil temp 15000 ms / 110 ms         -> 139
 *   over_rev            rpm 330 ms / 110 ms                -> 3 + 2 = 5
 *   lugging             engine load 6000 ms / 110 ms      -> 55 + 2 = 57 (gear_estimate: 330 ms)
 *   idle_rpm_std/drift  coolant 2400 ms / 110 ms           -> 22 + 2 = 24 */
#define DERIVED_RIDE_TAIL_CAP_HEAT_SOAK (139u)
#define DERIVED_RIDE_TAIL_CAP_COLD_ABUSE (139u)
#define DERIVED_RIDE_TAIL_CAP_OVER_REV (5u)
#define DERIVED_RIDE_TAIL_CAP_LUGGING (57u)
#define DERIVED_RIDE_TAIL_CAP_IDLE_STD (24u)
#define DERIVED_RIDE_TAIL_CAP_IDLE_DRIFT (24u)
/* One shared pool of rows; each signal owns a consecutive slice of its cap. */
#define DERIVED_RIDE_TAIL_POOL                                                                      \
    (DERIVED_RIDE_TAIL_CAP_HEAT_SOAK + DERIVED_RIDE_TAIL_CAP_COLD_ABUSE + DERIVED_RIDE_TAIL_CAP_OVER_REV + \
     DERIVED_RIDE_TAIL_CAP_LUGGING + DERIVED_RIDE_TAIL_CAP_IDLE_STD + DERIVED_RIDE_TAIL_CAP_IDLE_DRIFT)
#define DERIVED_RIDE_NO_SLOT (0xFFu)

/* One waiting row; a NaN value is an invalid row (a valid row is always finite). */
typedef struct {
    uint32_t tick;
    float value;
} derived_ride_row_t;

typedef struct {
    uint16_t head; /* a ring over the signal's slice of the pool, oldest row first */
    uint16_t count;
} derived_ride_tail_t;

/* The time a class-A flag output is 1 (the reference's _flag_seconds): every row counts. */
typedef struct {
    double total_s;
    uint32_t prev_ms; /* time of the previous row */
    bool has_prev;
    bool prev_one; /* the previous row was valid with value 1 */
    bool any_valid;
} derived_ride_flag_time_t;

typedef struct {
    float trip_distance; /* last valid class-A outputs of the ride */
    float trip_time;
    float idle_time;
    float coolant_rate_max;
    float coolant_first;
    float coolant_last;
    float intake_first;
    uint32_t stop_count;
    uint32_t ltft_n;
    uint32_t stft_n;
    double ltft_sum;
    double stft_mean; /* Welford */
    double stft_m2;
    double fuel_prefix; /* closed segments of the fuel integral, litres */
    double fuel_rate;   /* L/s of the open segment */
    uint32_t fuel_seg_start_ms;
    int32_t fuel_seg_remaining_ms;
    bool fuel_seg_active;
    bool fuel_seen;
    bool has_trip_distance;
    bool has_trip_time;
    bool has_idle_time;
    bool has_coolant_rate;
    bool has_coolant;
    bool has_intake;
    /* D-103 class-A outputs of the ride: max / first / last / counts */
    float heat_soak_max;
    float cranking_dip;
    float rest_voltage;
    float idle_rpm_std_max;
    float idle_rpm_drift;
    uint32_t shift_count;
    uint32_t launch_count;
    bool has_heat_soak;
    bool has_cranking_dip;
    bool has_rest_voltage;
    bool has_idle_rpm_std;
    bool has_idle_rpm_drift;
    derived_ride_flag_time_t flag_time[DERIVED_RIDE_FLAG_TIMES];
    derived_ride_tail_t tail[DERIVED_RIDE_TAILS];
    derived_ride_row_t tail_pool[DERIVED_RIDE_TAIL_POOL];
    /* ride_warmup_time: first rpm > 0, first coolant, first coolant >= operating */
    uint32_t rpm_start_ms;
    uint32_t warm_ms;
    bool has_rpm_start;
    bool first_coolant_warm;
    bool has_warm;
    /* ride_thermal_cycle_count */
    uint32_t thermal_cycles;
    bool was_cold;
    /* ride_fan_on_temperature: latest speed / rpm / coolant sample, the previous battery
     * sample with its resting-idle coolant, the battery sample waiting for the step to settle */
    float speed_v;
    float rpm_v;
    float coolant_v;
    uint32_t speed_ms;
    uint32_t rpm_ms;
    uint32_t coolant_ms;
    bool has_speed;
    bool has_rpm;
    bool has_coolant_now;
    float batt_prev_v;
    uint32_t batt_prev_ms;
    float batt_prev_coolant; /* the resting-idle coolant at batt_prev, valid when batt_prev_rest */
    bool has_batt_prev;
    bool batt_prev_rest;
    float batt_pend_v;
    uint32_t batt_pend_ms;
    bool batt_pending;
    float fan_temp;
    bool fan_found;
    /* D-079 health windows (W1-W5) and the statistics of the 17 window summaries */
    derived_windows_t win;
} derived_ride_acc_t;

typedef struct {
    derived_ride_acc_t acc; /* the open ride's running state */
    derived_out_t prev[DERIVED_RIDE_SLOTS]; /* the last closed ride, per slot */
    uint32_t last_now_ms; /* time of the last core step (stamps CURRENT reads) */
    uint32_t closed_count;
    uint16_t counter; /* epoch counter of the open ride */
    uint16_t prev_counter; /* epoch counter of the last closed ride (valid once have_prev) */
    bool open;
    bool have_prev;
} derived_ride_t;

/* Slot of a class-B signal this file computes (0..DERIVED_RIDE_SLOTS-1), else NO_SLOT. */
uint32_t derived_ride_slot(uint32_t sig);

/* Boot: no ride, no closed record. */
void derived_ride_init(derived_ride_t* r);

/* A new ride: fresh accumulators. */
void derived_ride_begin(derived_ride_t* r, uint16_t counter);

/* A new DID sample of the open ride (once per sample, in time order). */
void derived_ride_sample(derived_ride_acc_t* a, uint32_t did, float value, uint32_t stamp_ms);

/* Every evaluated class-A row of the open ride (the signals the summaries read; others
 * ignored): its time, whether it is valid (no flag) and its value. The flag-time summaries
 * need the invalid rows too. */
void derived_ride_class_a(derived_ride_acc_t* a, uint32_t sig, uint32_t stamp_ms, bool valid, float value);

/* True for the grid class-A outputs whose rows wait (heat soak delta, cold engine abuse, over
 * rev, lugging, idle RPM std and drift). The reference evaluates a grid signal only up to the
 * last time every input has a sample (engine.py: end = min of the inputs' last times), so a
 * row counts toward a summary only once every input has a sample at or after its tick; rows
 * after the final such time are dropped at the close. Each such signal holds its not yet
 * confirmed rows in a bounded ring of DERIVED_RIDE_TAIL_CAP_* rows. When the ring is full the
 * oldest row is past every input's stale limit (an invalid row in the reference too) and is
 * confirmed early. */
bool derived_ride_is_tail_signal(uint32_t sig);

/* Rows `sig` can hold (0 for a signal that does not wait), and the rows it needs per the gen/
 * table: ceil(max input stale_after_ms / period_ms) + 2. fits = cap >= need (true for a signal
 * that does not wait). */
uint32_t derived_ride_tail_cap(uint32_t sig);
uint32_t derived_ride_tail_need(uint32_t sig);
bool derived_ride_tail_fits(uint32_t sig);

/* Confirms the waiting rows of `sig` with tick <= bound_ms (wrap-safe) into the summaries. */
void derived_ride_advance(derived_ride_acc_t* a, uint32_t sig, uint32_t bound_ms);

/* End of one ingest: settles the battery sample that arrived in it (ride_fan_on_temperature). */
void derived_ride_settle(derived_ride_acc_t* a);

/* The value `sig` would close with now. status VALID or INVALID (flag), unranged, value;
 * stamp and seq are the caller's. Not a computed class-B slot: NOT_COMPUTED. */
void derived_ride_finalize(const derived_ride_acc_t* a, uint32_t sig, derived_out_t* out);

/* Closes the open ride into `prev` at now_ms and keeps its epoch counter (the D-098
 * header counter of the closed ride, not the counter of a ride opened later). */
void derived_ride_close(derived_ride_t* r, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_RIDE_H */
