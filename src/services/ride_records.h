#ifndef SERVICES_RIDE_RECORDS_H
#define SERVICES_RIDE_RECORDS_H

/*
 * Derived to comms hand-over of the ride-summary records 0xFD40-0xFD4E (D-098 item 6, C3
 * part 2b), the 0xFD03 pattern in the other direction: the derived task encodes the records
 * of the last closed ride (services/ride_records_core) straight into the write slot of a
 * services/snapshot, and the comms task takes one copy that the UDS server reads.
 *
 *   producer  ride_records_publish()   derived task, ONE call site: features/derived/derived.c,
 *                                      only when a ride closed (closed_count changed)
 *   consumer  ride_records_take()      comms task, ONE call site: app/comms.c, right before
 *                                      uds_server_step(); the single snapshot_read_index(),
 *                                      which copies a new record into the comms copy in the
 *                                      same call (second-reader rule, services/snapshot.h)
 *   reader    ride_records_read()      comms task, the UDS server: the comms copy only, never
 *                                      the snapshot, no slot index kept
 *
 * Boot: ride_records_init() before either task runs writes the no-ride records (counter 0,
 * RIDE_CLOSED 0, every field NOT_AVAILABLE) into the three slots and the comms copy, so a
 * 0x22 before the first close answers "no ride closed since boot". Until it ran, publish
 * does nothing (counted), take does nothing and read fails. After a producer fault nothing
 * is published again; after a consumer fault the comms copy keeps the last record taken
 * (the server keeps answering it; the faults are visible through the getters).
 *
 * Last closed ride only, no persistence, read-only, platform bus only. QM information
 * (D-070 item 5): nothing safety-relevant reads it. Static storage, no heap; no
 * hal_atomic.h here (the snapshot service holds the atomics).
 */

#include "services/ride_records_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot only, before the derived task and the comms task run. */
void ride_records_init(void);

/* Derived task: encodes the records of the closed ride (closed false: the no-ride records)
 * into the write slot and publishes them. False if it did nothing (before init, or after a
 * producer fault). Bounded: one pass over the gen/ field table, one atomic exchange. */
bool ride_records_publish(ride_records_value_fn value, bool closed, uint16_t counter);

/* Comms task, once per pass before the UDS server: copies a newly published record into the
 * comms copy (nothing to copy: no work beyond one atomic load). */
void ride_records_take(void);

/* Comms task (UDS server): the record of platform UDS DID index did_idx from the comms copy.
 * False if not initialised, not a served ride record, or len differs from the gen/ length. */
bool ride_records_read(uint32_t did_idx, uint8_t* out, uint16_t len);

/* Publish calls that did nothing because ride_records_init() never ran (saturates).
 * Derived task only. */
uint32_t ride_records_skipped_count(void);

/* Latched snapshot faults (services/snapshot): producer side (derived task) / consumer side
 * (comms task). */
bool ride_records_producer_fault(void);
bool ride_records_consumer_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_RIDE_RECORDS_H */
