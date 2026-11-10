#ifndef FEATURES_DERIVED_DERIVED_GEAR_H
#define FEATURES_DERIVED_DERIVED_GEAR_H

/*
 * Gear estimate from a measured gearbox ratio (D-070, catalog section 5): pure, no state.
 * Kept in its own file so it can move to services/ when a second feature needs a gear
 * (architecture-guard MAJOR on the C3 plan); until then it is private to features/derived.
 * The ratios come from gen/ vehicle_cl250_constants.h only.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* The gear 1..VEHICLE_CL250_GEAR_COUNT whose ratio is nearest to `measured_ratio`
 * (relative distance, the lower gear on a tie), or 0 when none is within gen/
 * VEHICLE_CL250_DERIVED_GEAR_MATCH_TOLERANCE
 * or the ratio is not a finite positive number. */
uint8_t derived_gear_estimate(float measured_ratio);

/* The gen/ ratio of gear 1..VEHICLE_CL250_GEAR_COUNT, or 0 for any other gear. */
float derived_gear_ratio(uint8_t gear);

/* The gen/ gear ratio nearest to `measured_ratio` (relative distance, the lower gear on a
 * tie), with no tolerance: the D-103 clutch rule compares the measured ratio with it. */
float derived_gear_nearest_ratio(float measured_ratio);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_GEAR_H */
