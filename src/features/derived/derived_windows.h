#ifndef FEATURES_DERIVED_DERIVED_WINDOWS_H
#define FEATURES_DERIVED_DERIVED_WINDOWS_H

/*
 * D-079 health windows of the ride summaries (ISSUES E-30 (11) part 2): the pure state
 * machine of the five windows W1..W5 of gen/ `vehicle_cl250_window[]` and the statistics of the
 * 17 class-B window summaries over the ride's FIRST qualifying instance of each window.
 * No HAL, RTOS, clock, heap; every loop is bounded by a compile-time size. The rules are the
 * vehicle-server reference (derived/health_windows.py, health_window_summaries.py):
 *
 *  - A window is evaluated on the merged sample times of its condition, stable and trigger
 *    inputs plus, for every input, the first time its last sample is stale (sample +
 *    stale_after_ms + 1) when no newer sample arrived by then and some window sample
 *    exists at or after that time ("a lost input closes the instance at its stale instant").
 *    That time is only known once a later sample arrives, so an expiry is processed when the
 *    next window-input sample comes (lazily) and never at the ride end.
 *  - At every time each input has its latest sample unless it is older than its own
 *    stale_after_ms (then it is unknown and a condition with an unknown input is not met).
 *  - Opening: `condition` windows open when the conditions and the stable inputs are available
 *    and re-arm only after the conditions were not met once; `key_on` opens at the first
 *    point only; `engine_start` / `engine_stop` open on the next transition of ENGINE_SPEED
 *    (consecutive samples no further apart than the DID's stale_after_ms) when the
 *    conditions hold. Closing: max duration (clamped to open + max), conditions, a stable band
 *    exceeded, engine zero / unknown (engine_start) and the ride end at the last point.
 *  - An instance qualifies by max duration or once it lasted min_duration_ms.
 *
 * The statistics of the first qualifying instance are accumulated while it is open from the
 * DID samples inside [open, close) (a close by max duration or by the ride end keeps its
 * boundary sample). A stat sample waits in a small pending list until the window point at or
 * after it is evaluated, because the point may close the instance (the sample at the closing
 * point is outside) and a stat sample after the last window sample is only inside once a
 * later window sample proves the expiry real.
 *
 * Everything comes from the gen/ window table; a window whose sizing (pending list, coolant
 * buffer) or inputs (a derived channel that is not computed, a sensor) do not fit is not
 * enabled and its signals are NOT_COMPUTED, never truncated.
 */

#include "features/derived/derived_types.h"
#include "vehicle_cl250.h"
#include "vehicle_cl250_derived.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DERIVED_WINDOWS_COUNT (VEHICLE_CL250_WINDOW_COUNT)
/* Distinct inputs of one window: conditions + stable + the trigger. */
#define DERIVED_WINDOWS_CH_MAX (VEHICLE_CL250_WINDOW_CONDITION_MAX + VEHICLE_CL250_WINDOW_STABLE_MAX + 1u)
/* Pending stat samples per window. Needed per DID: ceil((longest channel stale + 1) / poll) + 1,
 * summed over the window's stat DIDs (derived_windows_sizing_ok() checks it against gen/):
 * W1 12, W3 6, W5 4. */
#define DERIVED_WINDOWS_PEND_CAP (16u)
/* New window events of one step: at most one sample per wanted DID plus one row per derived
 * window input (derived_windows_sizing_ok() checks it against gen/). */
#define DERIVED_WINDOWS_EVENT_CAP (16u)
/* Post-stop coolant samples (W5 tau; 4 B each, exact: delta time + raw DID integer): max_duration_ms / coolant poll + 2 = 1 800 000 / 800 + 2.
 * gen/ exports both only as table data, so derived_windows_sizing_ok() checks it at init. */
#define DERIVED_WIN_POST_STOP_COOLANT_CAP (2252u)
#define DERIVED_WINDOWS_NONE (0xFFu)

/* Latest sample of one window input and its pending expiry. */
typedef struct {
    uint32_t t;
    float v;
    uint32_t exp_t; /* t + stale_after_ms + 1, pending until a newer sample or a later point */
    uint32_t stale_ms;
    uint8_t kind; /* VEHICLE_CL250_DERIVED_INPUT_DID / _DERIVED */
    uint8_t index;
    bool has;
    bool valid; /* a derived row can be invalid (unknown); a DID sample is always valid */
    bool exp_pending;
} derived_win_chan_t;

/* A stored post-stop coolant sample (4 B): the time since the previous stored sample (a gap
 * above uint16 is `broken`) and the DID's raw integer; the float is rebuilt exactly like
 * vehicle_cl250_decode() does. */
typedef struct {
    uint16_t dt_ms;
    uint16_t raw;
} derived_win_cool_t;

/* A stat sample waiting for its window point. */
typedef struct {
    uint32_t t;
    float v;
    uint8_t did;
} derived_win_item_t;

/* Mean over DID samples (the reference's sum / len, double). */
typedef struct {
    double sum;
    uint32_t n;
} derived_win_mean_t;

#define DERIVED_WIN_MEAN_BATTERY (0u)
#define DERIVED_WIN_MEAN_COOLANT (1u)
#define DERIVED_WIN_MEAN_INTAKE (2u)
#define DERIVED_WIN_MEAN_AMBIENT (3u)
#define DERIVED_WIN_MEAN_MAP (4u)
#define DERIVED_WIN_MEAN_BARO (5u)
#define DERIVED_WIN_MEANS (6u)

/* Statistics of the instance being collected (the first qualifying one once frozen). */
typedef struct {
    derived_win_mean_t mean[DERIVED_WIN_MEANS];
    float map_min; /* warm-idle MAP spread */
    float map_max;
    double stft_mean; /* Welford, sample std (n - 1) */
    double stft_m2;
    uint32_t stft_n;
    float coolant_first; /* post-stop coolant */
    float coolant_last;
    uint32_t coolant_n; /* samples in the shared buffer */
    uint32_t coolant_t0; /* stamp of the first stored sample */
    uint32_t coolant_tl; /* stamp of the last stored sample */
    /* warm-idle engine speed, computed from the core's RPM history when the instance closes */
    double rpm_spread;
    double rpm_drift;
    bool has_map;
    bool has_coolant;
    bool rpm_spread_ok;
    bool rpm_drift_ok;
} derived_win_stats_t;

typedef struct {
    derived_win_chan_t ch[DERIVED_WINDOWS_CH_MAX];
    derived_win_item_t pend[DERIVED_WINDOWS_PEND_CAP];
    derived_win_stats_t st;
    double lo[VEHICLE_CL250_WINDOW_STABLE_MAX]; /* stable bands of the open instance */
    double hi[VEHICLE_CL250_WINDOW_STABLE_MAX];
    double prev_v; /* previous trigger sample */
    double trig_v;
    uint32_t open_ms;
    uint32_t staged_t;
    uint32_t last_t; /* the last evaluated point */
    uint32_t prev_t;
    uint32_t count; /* qualifying instances */
    uint32_t first_open_ms;
    uint32_t first_close_ms;
    uint8_t first_reason;
    uint8_t cond_ch[VEHICLE_CL250_WINDOW_CONDITION_MAX];
    uint8_t stable_ch[VEHICLE_CL250_WINDOW_STABLE_MAX];
    uint8_t trig_ch;
    uint8_t nch;
    uint8_t pend_n;
    bool enabled;
    bool open;
    bool staged;
    bool trig_has;
    bool has_last;
    bool can_open;
    bool key_on_seen;
    bool has_prev;
    bool has_first;
    bool broken; /* a bound was exceeded at run time: the summaries are MATH_ERROR, never wrong */
} derived_win_t;

/* One new sample (a DID) or row (a derived signal) of the step, before it is dispatched. */
typedef struct {
    uint32_t t;
    float v;
    uint8_t kind; /* VEHICLE_CL250_DERIVED_INPUT_DID / _DERIVED */
    uint8_t index;
    bool valid;
} derived_win_event_t;

typedef struct {
    derived_win_t w[DERIVED_WINDOWS_COUNT];
    derived_win_cool_t coolant_buf[DERIVED_WIN_POST_STOP_COOLANT_CAP]; /* W5 tau fit */
    derived_win_event_t ev[DERIVED_WINDOWS_EVENT_CAP];
    const derived_sample_t* rpm_data; /* the core's ENGINE_SPEED history (warm-idle drift) */
    const derived_ring_t* rpm_ring;
    uint32_t rpm_cap;
    uint8_t ev_n;
} derived_windows_t;

/* ---- gen/ checks, used at init ---------------------------------------------------------- */

/* The window `sig` reports in (its record DID is the window's), else DERIVED_WINDOWS_NONE. */
uint32_t derived_windows_of_signal(uint32_t sig);

/* True for a class-B signal this module computes the value of. */
bool derived_windows_has_summary(uint32_t sig);

/* The pending list, the step event buffer, (W3) the core's ENGINE_SPEED history of
 * `rpm_hist_cap` samples and (W5) the coolant buffer fit the gen/ table for window `w`. */
bool derived_windows_sizing_ok(uint32_t w, uint32_t rpm_hist_cap);

/* Derived signals window `w` reads as inputs (DID inputs need nothing), into out[]; returns
 * the count, or DERIVED_WINDOWS_CH_MAX + 1 when `w` has a sensor input (never computed). */
uint32_t derived_windows_derived_inputs(uint32_t w, uint8_t out[DERIVED_WINDOWS_CH_MAX]);

/* ---- the ride ------------------------------------------------------------------------------ */

/* A ride begins: clears everything, wires the windows of `enabled_mask` (bit w) from gen/ and
 * remembers the core's ENGINE_SPEED history. */
void derived_windows_begin(derived_windows_t* a, uint32_t enabled_mask, const derived_sample_t* rpm_data,
                           const derived_ring_t* rpm_ring, uint32_t rpm_cap);

/* New sample of DID `did` / new row of derived signal `sig` of the open ride (queued; ignored
 * when no enabled window reads it). */
void derived_windows_push_did(derived_windows_t* a, uint32_t did, float value, uint32_t stamp_ms);
void derived_windows_push_derived(derived_windows_t* a, uint32_t sig, bool valid, float value, uint32_t stamp_ms);

/* Dispatches the queued events in time order (window inputs before stat samples of the same
 * time). */
void derived_windows_run(derived_windows_t* a);

/* The ride ends: evaluates the last point and closes an open instance there. */
void derived_windows_end(derived_windows_t* a);

/* The value of class-B window signal `sig` (as the ride would close now). Returns the flag
 * (DERIVED_FLAG_NONE with *out set, else why it is invalid). */
derived_flag_t derived_windows_value(const derived_windows_t* a, uint32_t sig, double* out);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_WINDOWS_H */
