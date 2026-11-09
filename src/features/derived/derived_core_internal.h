#ifndef FEATURES_DERIVED_DERIVED_CORE_INTERNAL_H
#define FEATURES_DERIVED_DERIVED_CORE_INTERNAL_H

/*
 * Private interface between derived_core.c (scheduler, grid, ring storage, class-A point and
 * integral formulas, output store) and derived_d103.c (the D-103 class-A formulas, their
 * sample history and event state machines). Only the files in features/derived may include
 * it (CI layering check); everything else uses derived_core.h.
 */

#include "features/derived/derived_core.h"
#include "vehicle_cl250_constants.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sample ring ids (index into derived_core_t.ring). */
#define RING_SPEED (0u)
#define RING_FUEL_RATE (1u)
#define RING_COOLANT (2u)
#define RING_OIL (3u)
#define RING_ACCEL (4u)
#define RING_RPM_HIST (5u)
#define RING_BATT_HIST (6u)
#define RING_FUEL_HIST (7u)
#define RING_ACCEL_HIST (8u)
#define RING_NONE (0xFFu)

/* What a formula or an event step sees: the core, the evaluation time, the signal and its spec. */
typedef struct {
    derived_core_t* c;
    uint32_t now;
    uint32_t sig;
    const vehicle_cl250_derived_t* spec;
} derived_ctx_t;

/* ---- provided by derived_core.c --------------------------------------------------------- */

/* Wrap-safe time helpers and the finite check. */
uint32_t derived_core_age_of(uint32_t now, uint32_t stamp);
bool derived_core_at_or_after(uint32_t a, uint32_t b);
bool derived_core_is_finite_f(float x);

/* Ring storage: the sample array and capacity of ring `id`, and a push. */
derived_sample_t* derived_core_ring_data(derived_core_t* c, uint32_t id, uint32_t* cap);
void derived_core_ring_push(derived_core_t* c, uint32_t id, float value, uint32_t stamp);

/* Declared inputs: a fresh DID, and a fresh valid derived output. */
derived_flag_t derived_core_did_value(const derived_ctx_t* x, uint32_t did, float* v);
derived_flag_t derived_core_derived_value(const derived_ctx_t* x, uint32_t sig, float* v);

/* Stores an output (range check, status, sequence, ride accumulation). */
void derived_core_store(derived_core_t* c, uint32_t sig, uint32_t stamp, derived_flag_t flag, float value);

/* ---- provided by derived_d103.c --------------------------------------------------------- */

/* Records the D-103 sample history (RPM, battery, fuel, event baselines) after the ingest. */
void derived_d103_record(derived_core_t* c);

/* Evaluates the D-103 point formula of `x->sig`; MATH_ERROR for any other signal. */
derived_flag_t derived_d103_eval_point(const derived_ctx_t* x, float* out);

/* Runs the D-103 event state machine of `sig`; does nothing for any other signal. */
void derived_d103_event_step(derived_core_t* c, uint32_t sig);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_CORE_INTERNAL_H */
