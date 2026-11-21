#ifndef FEATURES_DERIVED_DERIVED_RIDE_INTERNAL_H
#define FEATURES_DERIVED_DERIVED_RIDE_INTERNAL_H

/*
 * Private interface between derived_ride.c (accumulators) and derived_ride_final.c (closed values).
 * Moved out of one file, same code (ride_finite lost its `static`). Only the files in features/derived
 * may include it.
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ms to s. */
#define RIDE_MS_PER_S (1000.0)

/* True for a finite float (the accumulators and the summaries reject NaN and infinity). */
bool ride_finite(float x);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_RIDE_INTERNAL_H */
