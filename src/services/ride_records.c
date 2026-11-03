#include "services/ride_records.h"

#include "services/snapshot.h"

#include <stddef.h>
#include <string.h>

static snapshot_t rr_snap;
static uint8_t rr_slots[SNAPSHOT_SLOTS][RIDE_RECORDS_BYTES];
static uint8_t rr_comms_copy[RIDE_RECORDS_BYTES]; /* comms task only: what the server answers */
static bool rr_inited;      /* set at boot, before the tasks run */
static uint32_t rr_skipped; /* producer only: publishes before init */

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void ride_records_init(void)
{
    ride_records_encode(NULL, false, 0u, rr_comms_copy); /* no ride closed since boot */
    for (uint32_t i = 0u; i < SNAPSHOT_SLOTS; i++) {
        (void)memcpy(rr_slots[i], rr_comms_copy, sizeof rr_comms_copy);
    }
    snapshot_init(&rr_snap);
    rr_skipped = 0u;
    rr_inited = true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by features/derived (derived task) and the tests
bool ride_records_publish(ride_records_value_fn value, bool closed, uint16_t counter)
{
    if (!rr_inited) {
        if (rr_skipped < UINT32_MAX) {
            rr_skipped++; /* a missing ride_records_init() at boot must be visible */
        }
        return false;
    }
    if (snapshot_producer_fault(&rr_snap)) {
        return false; /* after a producer fault the write slot may be the consumer's front */
    }
    ride_records_encode(value, closed, counter, rr_slots[snapshot_write_index(&rr_snap)]);
    snapshot_publish(&rr_snap);
    return true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by app/comms (comms task) and the tests
void ride_records_take(void)
{
    if (!rr_inited) {
        return;
    }
    uint8_t idx = 0u;
    if (snapshot_read_index(&rr_snap, &idx)) {
        (void)memcpy(rr_comms_copy, rr_slots[idx], sizeof rr_comms_copy); /* no index outlives this call */
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by features/uds (comms task) and the tests
bool ride_records_read(uint32_t did_idx, uint8_t* out, uint16_t len)
{
    return rr_inited && ride_records_copy(rr_comms_copy, did_idx, out, len);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
uint32_t ride_records_skipped_count(void)
{
    return rr_skipped;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
bool ride_records_producer_fault(void)
{
    return snapshot_producer_fault(&rr_snap);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
bool ride_records_consumer_fault(void)
{
    return snapshot_consumer_fault(&rr_snap);
}
