#ifndef SERVICES_DERIVED_INPUTS_H
#define SERVICES_DERIVED_INPUTS_H

/*
 * Cross-task hand-over of the DID samples to the derived-signal task (D-070, architecture-
 * guard BLOCKER on the C3 plan). services/vehicle_signals is main-loop / single-task only:
 * the UDS client (comms task) writes it, so the derived task must never read it. This
 * service copies every DID sample out of vehicle_signals in the comms task and hands the
 * copy to the derived task through services/snapshot (SPSC triple buffer, no lock, no
 * wait, no retry).
 *
 *   producer  derived_inputs_publish(now_ms)   comms task, ONE call site: app/comms.c,
 *                                              right after uds_client_step()
 *   consumer  derived_inputs_read(&record)     derived task, ONE call site: features/derived
 *
 * One consumer call site (snapshot second-reader rule): the read copies the whole record
 * out in the same call, so no slot index or pointer outlives it.
 *
 * Record: for every DID index of vehicle_cl250_dids[] the value, the sample stamp and the
 * state vehicle_signals reported at copy time, the tester's stop epoch (services/tester_epoch:
 * open flag and counter, the ride boundary of the class-B summaries) and a pass counter (one
 * per publish, so a test can tell a torn record: every entry of one record comes from the
 * same pass).
 *
 * D-104 (data_quality_mask): the record also carries the tester's per-DID quality state, the
 * vehicle-bus bus-off count and the tester's step-gap window (services/tester_quality), valid
 * flag included, all stamped with published_ms. The window is TAKEN AND CLEARED from the
 * service by every publish (the reset belongs to this sampler, not to the UDS glue). The
 * derived task reads only the newest record and the comms pass publishes far more often than
 * it reads, so a gap seen in one publish would be overwritten before any 110 ms tick of the
 * derived task could read it: the publish therefore holds the largest taken gap in the record
 * for DERIVED_INPUTS_GAP_HOLD_MS (twice the signal's period, so a tick always sees it) and
 * then lets it go; a larger gap taken meanwhile replaces it.
 * Entries keep the sample's own stamp, so a stalled producer makes them age out in the
 * consumer: the record says "newest", not "fresh".
 *
 * Boot: derived_inputs_init() before either task runs (all three slots INVALID, state NONE).
 * Until it ran, publish does nothing. Static storage, no heap.
 */

#include "vehicle_cl250.h"
#include "vehicle_cl250_derived.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* How long a taken step gap stays in the published records (D-104): two periods of the
 * data_quality_mask grid, so at least one tick of the derived task reads it. */
#define DERIVED_INPUTS_GAP_HOLD_MS (2u * VEHICLE_CL250_PERIOD_MS_DATA_QUALITY_MASK)

typedef struct {
    float value;
    uint32_t stamp_ms;
    uint8_t state; /* vehicle_signal_state_t: NONE (0), VALID, STALE */
} derived_input_entry_t;

typedef struct {
    uint32_t pass;         /* 0 = the boot record, no publish yet */
    uint32_t published_ms; /* the comms task's time of the copy */
    uint32_t bus_off_count;     /* services/tester_quality: vehicle-bus bus-offs since boot (D-104) */
    uint32_t step_gap_window_ms; /* longest tester poll gap in the hold window (D-104, see below) */
    uint16_t epoch_counter; /* services/tester_epoch at the copy: changes at every new epoch (D-087) */
    uint8_t epoch_open;     /* 1 while a stop epoch is running (the ride is open) */
    uint8_t quality_valid;  /* 1 when services/tester_quality was valid at the copy; 0 = unknown */
    uint8_t did_quality[VEHICLE_CL250_DID_COUNT]; /* tester_quality_state_t per DID (D-104) */
    derived_input_entry_t did[VEHICLE_CL250_DID_COUNT];
} derived_inputs_record_t;

/* Boot only, before the comms task and the derived task run. */
void derived_inputs_init(void);

/* Comms task: copies vehicle_signals (as of now_ms) into the write slot and publishes it.
 * Bounded: one pass over the DID table, one atomic exchange. Does nothing before
 * derived_inputs_init() or after a producer fault. */
void derived_inputs_publish(uint32_t now_ms);

/* Derived task: copies the newest record into *out. True if it is new since the last call;
 * false keeps the previous record (an all-NONE boot record before the first publish). */
bool derived_inputs_read(derived_inputs_record_t* out);

/* Publish calls that did nothing because derived_inputs_init() never ran (a missing boot
 * call on the H7 shows up here; saturates at UINT32_MAX). Comms task only. */
uint32_t derived_inputs_skipped_count(void);

/* Latched snapshot faults (see services/snapshot): producer side / consumer side. */
bool derived_inputs_producer_fault(void);
bool derived_inputs_consumer_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_DERIVED_INPUTS_H */
