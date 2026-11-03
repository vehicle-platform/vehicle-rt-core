#ifndef SERVICES_RIDE_RECORDS_CORE_H
#define SERVICES_RIDE_RECORDS_CORE_H

/*
 * Pure encoder of the ride-summary record DIDs 0xFD40-0xFD4E (D-098, C3 part 2b): the
 * generic writer over the generated layout (gen/ platform_uds.h: PLATFORM_UDS_IDX_RIDE_*,
 * *_LENGTH, the per-record header *_BYTE / *_MASK, platform_uds_ride_fields[]). Nothing of
 * the layout is typed here (invariant 2); 0xFD42 RIDE_OPERATING_POINTS has no gen/ entry
 * and is not served (D-098 item 1).
 *
 * The records of one ride live in one flat buffer of RIDE_RECORDS_BYTES, in gen/ table
 * order. Every record starts with the 4-byte header (D-098 item 4): the layout version, the
 * big-endian stop-epoch counter of the closed ride, and the flags byte (bit 0 RIDE_CLOSED,
 * the other bits zero). A field is raw = round half to even of (value - range_min) /
 * resolution in float32 (`lrintf`, the default FE_TONEAREST; never roundf/lroundf, D-098
 * item 5), unsigned big-endian. Its all-ones value (`na`) is NOT_AVAILABLE: no valid value,
 * not finite, or a rounded raw value outside 0..raw_max (never clamped, D-070). No ride
 * closed: counter 0, RIDE_CLOSED 0 and every field NOT_AVAILABLE.
 *
 * It sees no derived type (services sit below features/): the caller hands the values over
 * through ride_records_value_fn, one call per row of platform_uds_ride_fields[]. No heap, no
 * recursion, loops bounded by the gen/ tables. QM (D-070 item 5).
 */

#include "platform_uds.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bytes of all served ride records together (the gen/ record lengths). */
#define RIDE_RECORDS_BYTES                                                                         \
    (PLATFORM_UDS_DID_RIDE_USAGE_LENGTH + PLATFORM_UDS_DID_RIDE_FUELING_LENGTH +                   \
     PLATFORM_UDS_DID_RIDE_THERMAL_LENGTH + PLATFORM_UDS_DID_RIDE_ELECTRICAL_LENGTH +              \
     PLATFORM_UDS_DID_RIDE_ENGINE_STRESS_LENGTH + PLATFORM_UDS_DID_RIDE_DRIVETRAIN_LENGTH +        \
     PLATFORM_UDS_DID_RIDE_CALIBRATION_LENGTH + PLATFORM_UDS_DID_RIDE_RIDER_STYLE_LENGTH +         \
     PLATFORM_UDS_DID_RIDE_ECU_HEALTH_LENGTH + PLATFORM_UDS_DID_RIDE_WINDOW_KEY_ON_LENGTH +        \
     PLATFORM_UDS_DID_RIDE_WINDOW_CRANK_LENGTH + PLATFORM_UDS_DID_RIDE_WINDOW_WARM_IDLE_LENGTH +   \
     PLATFORM_UDS_DID_RIDE_WINDOW_CRUISE_LENGTH + PLATFORM_UDS_DID_RIDE_WINDOW_POST_STOP_LENGTH)

/* The value of derived signal `signal` (a VEHICLE_CL250_DERIVED_* index) in the closed
 * ride: true with *value for a valid one, false for none (not computed, no data, invalid). */
typedef bool (*ride_records_value_fn)(uint32_t signal, float* value);

/* The raw value of one field: the field's `na` unless valid, finite and 0..raw_max after
 * rounding. NULL field: 0. */
uint32_t ride_records_field_raw(const platform_uds_ride_field_t* field, bool valid, float value);

/* Writes every served record into buf[RIDE_RECORDS_BYTES]. closed false (or value NULL):
 * the no-ride records (counter 0, RIDE_CLOSED 0, all fields NOT_AVAILABLE). buf NULL: no-op. */
void ride_records_encode(ride_records_value_fn value, bool closed, uint16_t counter, uint8_t* buf);

/* Copies the record of platform UDS DID index did_idx out of buf. False if did_idx is not a
 * served ride record, len differs from its gen/ length, or a pointer is NULL. */
bool ride_records_copy(const uint8_t* buf, uint32_t did_idx, uint8_t* out, uint16_t len);

/* True if did_idx is a served ride record (one of the gen/ PLATFORM_UDS_IDX_RIDE_*). */
bool ride_records_is_record(uint32_t did_idx);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_RIDE_RECORDS_CORE_H */
