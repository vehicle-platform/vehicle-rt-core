#ifndef FEATURES_DERIVED_DERIVED_CORE_H
#define FEATURES_DERIVED_DERIVED_CORE_H

/*
 * Pure derived-signal core of rt-core (D-070 items 1, 2 and 5, part 1: class A signals
 * whose inputs are DIDs or other computed derived signals). No HAL, RTOS, services or clock
 * access: the glue (derived.c) hands in the DID samples (value, stamp, present) and the
 * time; the core evaluates every computed signal on its gen/ period grid and keeps the last
 * output per signal.
 *
 * QM information (D-070 item 5): nothing here feeds 0x020, safety-node, the heartbeat or
 * the anomaly safety net. The module is not safety-critical, but it shares a CPU with the
 * EKF: no blocking, every loop is bounded by a compile-time size, static memory only.
 *
 * Sampling rule (D-070 item 1, the vehicle-server reference engine.py / context.py): an
 * input's value at time t is its last sample at or before t, and counts only while its age
 * is within its own stale_after_ms: for a DID input the DID's (vehicle_cl250_dids[], D-070),
 * for a derived input the producer's (vehicle_cl250_derived[]). The event signal is judged
 * the same way (D-087). A held value is never
 * presented as fresh. The output of a signal is on its own period_ms grid, which starts at
 * the latest first-sample time of its inputs (a derived input: the producer's grid start).
 * A result outside the gen/ [min, max] is invalid (OUT_OF_RANGE), never clamped.
 *
 * Which signals are computed is decided at init from the gen/ table (class A, rt_core_ready,
 * inputs only DIDs or computed derived signals earlier in the table, a formula in
 * derived_core.c, window buffers large enough); the others report NOT_COMPUTED with a reason.
 * Evaluation order is the gen/ index order (every derived input has a lower index).
 *
 * Time: evaluation happens at `now_ms`; the output is stamped with the grid tick it
 * answers (the latest tick at or before now). Called every millisecond (the tests) it is
 * the reference to the last tick; called every few ms (the glue) a tick is evaluated up to a
 * step late, and a step that spans several ticks of one signal evaluates the latest one only
 * (bounded work, no catch-up). Likewise at most one new sample per DID is seen per step.
 */

#include "features/derived/derived_ride.h"
#include "features/derived/derived_types.h"
#include "vehicle_cl250.h"
#include "vehicle_cl250_derived.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Compile-time sizing (static RAM). Capacity = window_ms / poll period + 2 samples of the
 * DID (or of the derived producer). gen/ does not export window or period as constant
 * expressions, so the numbers below are checked against the gen/ table at init: a signal
 * whose window needs more is NOT_COMPUTED (DERIVED_NC_SIZING), never silently truncated.
 * The caps derive from the gen/ table fields `poll_period_ms` (vehicle_cl250_dids[]; for the
 * accel ring `period_ms` of the producer) and `window_ms` (vehicle_cl250_derived[]) and are
 * checked against them at init (SIZING). Values for the pinned defs: SPEED 60000/110+2 = 547, FUEL_RATE 60000/2000+2 = 32,
 * COOLANT 60000/800+2 = 77, OIL 60000/5000+2 = 14, VEHICLE_ACCEL output 440/110+2 = 6.
 */
#define DERIVED_WIN_SPEED_CAP (548u)
#define DERIVED_WIN_FUEL_RATE_CAP (32u)
#define DERIVED_WIN_COOLANT_CAP (77u)
#define DERIVED_WIN_OIL_CAP (14u)
#define DERIVED_WIN_ACCEL_CAP (8u)
/* D-103 windows: largest window / poll + 2. ENGINE_SPEED poll 110 ms: idle drift 60 s
 * = 545 + 2 = 547; BATTERY_VOLTAGE poll 800 ms: charging plateau 10 s = 12 + 2 = 14;
 * FUEL_LEVEL poll 5000 ms: fuel level filtered 30 s = 6 + 2 = 8; VEHICLE_ACCEL period
 * 110 ms: the 30 s acceleration history = 272 + 2 = 274. */
#define DERIVED_WIN_RPM_HIST_CAP (548u)
#define DERIVED_WIN_BATT_HIST_CAP (15u)
#define DERIVED_WIN_FUEL_HIST_CAP (8u)
#define DERIVED_WIN_ACCEL_HIST_CAP (277u)
_Static_assert(DERIVED_WIN_RPM_HIST_CAP >= ((60u * 1000u) / 110u) + 2u, "idle drift window");
_Static_assert(DERIVED_WIN_ACCEL_HIST_CAP >= ((30u * 1000u) / 110u) + 2u, "fuel filter window");

/*
 * Algorithm parameters and unit conversions ported from the reference
 * (vehicle-server src/vehicle_server/derived/formulas.py and context.py). Vehicle facts come
 * from gen/ vehicle_cl250_constants.h only. Each "algorithm parameter" is provisional
 * (D-070 item 6); the others are unit conversions or physics constants.
 */
#define DERIVED_PI (3.14159265f)                       /* mathematical constant */
#define DERIVED_KMH_TO_MPS (0.27777778f)               /* 1 / 3.6, formulas.py:39 */
#define DERIVED_RPM_TO_RAD_PER_S (0.10471976f)         /* 2 pi / 60, formulas.py:40 */
#define DERIVED_R_DRY_AIR (287.058f)                   /* J/(kg K), formulas.py:43 */
#define DERIVED_KELVIN_OFFSET (273.15f)                /* formulas.py:44 */
#define DERIVED_STANDARD_GRAVITY (9.80665f)            /* formulas.py:47 */
#define DERIVED_MG_PER_KG (1.0e6f)                     /* formulas.py:49 */
#define DERIVED_G_PER_KG (1000.0f)                     /* formulas.py:50 */
#define DERIVED_SECONDS_PER_MINUTE (60.0f)             /* formulas.py:51 */
#define DERIVED_GPS_TO_KG_PER_H (3.6f)                 /* g/s -> kg/h, formulas.py:395 */
#define DERIVED_SECONDS_PER_HOUR (3600.0f)             /* km/h -> km/s, formulas.py:296 */
#define DERIVED_PA_PER_KPA (1000.0f)                   /* formulas.py:155 */
#define DERIVED_PERCENT (100.0f)                       /* formulas.py:342 */
#define DERIVED_MS_PER_S (1000.0f)                     /* formulas.py:168 */
#define DERIVED_M3_PER_CM3 (1.0e-6f)                   /* cm3 -> m3, d103_windows.py */
#define DERIVED_R_AIR_VE (287.05f)                     /* J/(kg K), d103_windows.py */
/* SAE J1349 reference conditions of the power correction: algorithm parameters, provisional
 * (D-070 item 6), formulas.py:56-57. */
#define DERIVED_J1349_REFERENCE_PRESSURE_KPA (99.0f)
#define DERIVED_J1349_REFERENCE_TEMPERATURE_C (25.0f)
/* A slope or a window mean needs at least this many samples: algorithm parameter,
 * provisional (D-070 item 6), formulas.py:160 and :417. */
#define DERIVED_MIN_WINDOW_SAMPLES (2u)

typedef struct {
    float value;
    uint32_t stamp_ms;
    uint32_t first_ms; /* stamp of the first sample seen */
    bool present;
    bool seen;
    bool fresh_now;    /* a new sample arrived in this step */
    bool lapsed;       /* older than every limit: stays unusable until a new sample */
    bool ignore_valid; /* a ride began: the sample with ignore_stamp_ms is from the old epoch */
    uint32_t ignore_stamp_ms;
} derived_did_t;

typedef struct {
    uint32_t start_ms;
    uint32_t next_ms;
    bool armed;
    bool computed;
    derived_nc_reason_t reason;
} derived_sched_t;

/* Integral of a rate along the time axis (trip distance, trip time, idle time). */
typedef struct {
    double prefix;
    double rate;
    uint32_t seg_start_ms;
    int32_t seg_remaining_ms;
    bool seg_active;
} derived_integral_t;

typedef struct {
    bool has_prev;
    bool moving;
    bool started;
    uint32_t prev_ms;
    uint32_t start_ms;
} derived_stop_t;

typedef struct {
    bool shift_has_prev;
    bool shift_floor_set;
    uint32_t shift_prev_ms;
    uint32_t shift_floor_ms;
    bool launch_active;
    bool launch_has_prev;
    bool launch_prev_valid;
    uint32_t launch_start_ms;
    uint32_t launch_prev_ms;
    float launch_prev_kmh;
    bool braking_active;
    bool braking_has_prev;
    uint32_t braking_run_ms;
    uint32_t braking_prev_ms;
    uint32_t braking_seq;
    float braking_peak_mps2;
    bool refuel_has_moving;
    bool refuel_rested;
    bool refuel_fuel_ok;
    uint32_t refuel_moving_ms;
    uint32_t refuel_fuel_stamp_ms;
    float refuel_fuel_pct;
    bool rest_has_prev;
    bool rest_zero_valid;
    uint32_t rest_prev_ms;
    uint32_t rest_zero_ms;
    float rest_zero_v;
    bool crank_has_prev;
    bool crank_zero_valid;
    bool crank_active;
    bool crank_min_ok;
    bool crank_end_pending;
    uint32_t crank_prev_ms;
    uint32_t crank_zero_ms;
    uint32_t crank_start_ms;
    uint32_t crank_end_ms;
    float crank_min_v;
    bool idle_baseline_ok;
    uint32_t idle_baseline_end_ms;
    float idle_baseline_sum;
    uint32_t idle_baseline_n;
} derived_d103_state_t;

typedef struct {
    derived_out_t out[VEHICLE_CL250_DERIVED_COUNT];
    derived_sched_t sched[VEHICLE_CL250_DERIVED_COUNT];
    derived_did_t did[VEHICLE_CL250_DID_COUNT];
    derived_integral_t integral[3];
    derived_stop_t stop;
    derived_d103_state_t d103;
    float baro_ref;  /* first positive barometric sample: the altitude reference */
    bool baro_ref_set;
    uint32_t max_stale_ms; /* longest limit of any computed signal: the lapse bound */
    uint32_t computed_count;
    uint32_t win_mask;     /* D-079 windows (bit w) with a computed summary and inputs/sizing that fit */
    derived_ride_t ride; /* class-B ride summaries (derived_ride.h) */
    derived_ring_t ring[9];
    derived_sample_t speed_win[DERIVED_WIN_SPEED_CAP];
    derived_sample_t fuel_rate_win[DERIVED_WIN_FUEL_RATE_CAP];
    derived_sample_t coolant_win[DERIVED_WIN_COOLANT_CAP];
    derived_sample_t oil_win[DERIVED_WIN_OIL_CAP];
    derived_sample_t accel_win[DERIVED_WIN_ACCEL_CAP];
    derived_sample_t rpm_hist[DERIVED_WIN_RPM_HIST_CAP];
    uint8_t rpm_warm[DERIVED_WIN_RPM_HIST_CAP];
    derived_sample_t batt_hist[DERIVED_WIN_BATT_HIST_CAP];
    derived_sample_t fuel_hist[DERIVED_WIN_FUEL_HIST_CAP];
    derived_sample_t accel_hist[DERIVED_WIN_ACCEL_HIST_CAP];
    uint8_t accel_hist_valid[DERIVED_WIN_ACCEL_HIST_CAP];
    uint32_t evals_step; /* formula evaluations (and event/breakpoint updates) in the last step */
    uint32_t evals_max;  /* the largest evals_step so far */
} derived_core_t;

/* Boot: decides the computed set from the gen/ table and clears every state. */
void derived_core_init(derived_core_t* c);

/*
 * One step at now_ms with the current DID samples (in[VEHICLE_CL250_DID_COUNT]). Bounded:
 * a fixed pass over the DIDs and over the gen/ table, every window loop limited by its
 * buffer. NULL arguments do nothing.
 */
void derived_core_step(derived_core_t* c, uint32_t now_ms, const derived_in_t* in);

/*
 * Like derived_core_step() with the tester's stop epoch (D-087 item 2): a ride is one epoch.
 * When the counter changes (any jump, wrap-safe) or `open` goes false, the open ride is
 * closed first, from what was accumulated up to the previous step; the inputs of this
 * step are never added to it. When an epoch is open and no ride is, a ride begins: the
 * class-A state (grids, integrals, windows, the altitude reference, the stop event) is
 * reset to a fresh ride-local state and the DID samples the core already holds are ignored
 * until a new sample arrives. epoch NULL or not known: no ride, class A only.
 */
void derived_core_step_epoch(derived_core_t* c, uint32_t now_ms, const derived_in_t* in,
                             const derived_epoch_t* epoch);

/* A class-B signal of the open ride (CURRENT, as it would close now) or of the last closed
 * ride (PREVIOUS). NO_DATA while there is no such ride, NOT_COMPUTED for a signal that is
 * not a computed class-B signal. False for a bad argument or index. */
bool derived_core_ride_out(const derived_core_t* c, derived_ride_which_t which, uint32_t sig,
                           derived_out_t* out);

/* The last closed ride (D-098 item 4): true with its stop-epoch counter in *counter once a
 * ride has closed since boot; false before (or for a NULL argument). */
bool derived_core_ride_closed(const derived_core_t* c, uint16_t* counter);

/* The last output of signal `sig` (a VEHICLE_CL250_DERIVED_*), or NULL when out of range. */
const derived_out_t* derived_core_out(const derived_core_t* c, uint32_t sig);

bool derived_core_is_computed(const derived_core_t* c, uint32_t sig);

/* DERIVED_NC_NONE for a computed signal, else the reason; DERIVED_NC_GEN for a bad index. */
derived_nc_reason_t derived_core_reason(const derived_core_t* c, uint32_t sig);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_CORE_H */
