#include "services/derived_inputs.h"

#include "services/snapshot.h"
#include "services/tester_epoch.h"
#include "services/tester_quality.h"
#include "services/vehicle_signals.h"

#include <stddef.h>
#include <string.h>

static snapshot_t snap;
static derived_inputs_record_t slots[SNAPSHOT_SLOTS];
static bool inited;       /* set at boot, before the tasks run */
static uint32_t pass_no;  /* producer only */
static uint32_t skipped;  /* producer only: publishes before init */
static uint32_t held_gap_ms;    /* producer only: the gap the records carry (D-104) */
static uint32_t held_since_ms;  /* producer only: when held_gap_ms was taken */

/* D-104: the gap the record carries. A taken gap larger than the held one replaces it; a held
 * one is dropped when it is older than DERIVED_INPUTS_GAP_HOLD_MS and then the taken one (if
 * any) takes its place. */
static uint32_t hold_gap(uint32_t taken_ms, uint32_t now_ms)
{
    const bool expired = (uint32_t)(now_ms - held_since_ms) >= DERIVED_INPUTS_GAP_HOLD_MS;
    if ((taken_ms > held_gap_ms) || expired) {
        held_gap_ms = taken_ms;
        held_since_ms = now_ms;
    }
    return held_gap_ms;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void derived_inputs_init(void)
{
    (void)memset(slots, 0, sizeof slots); /* state NONE, pass 0: INVALID */
    snapshot_init(&snap);
    pass_no = 0u;
    skipped = 0u;
    held_gap_ms = 0u;
    held_since_ms = 0u;
    inited = true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by app/comms (comms task) and the tests
void derived_inputs_publish(uint32_t now_ms)
{
    if (!inited) {
        if (skipped < UINT32_MAX) {
            skipped++; /* a missing derived_inputs_init() at boot must be visible */
        }
        return;
    }
    if (snapshot_producer_fault(&snap)) {
        return; /* after a producer fault the write slot may be the consumer's front */
    }
    derived_inputs_record_t* const rec = &slots[snapshot_write_index(&snap)];
    pass_no++;
    rec->pass = pass_no;
    rec->published_ms = now_ms;
    const tester_epoch_t epoch = tester_epoch_get(); /* the same record as the DIDs: one channel */
    rec->epoch_counter = epoch.counter;
    rec->epoch_open = epoch.open ? 1u : 0u;
    const tester_quality_t quality = tester_quality_take(); /* takes and clears the latched gap */
    rec->quality_valid = quality.valid ? 1u : 0u;
    rec->bus_off_count = quality.bus_off_count;
    rec->step_gap_window_ms = hold_gap(quality.step_gap_max_ms, now_ms);
    (void)memcpy(rec->did_quality, quality.did, sizeof rec->did_quality);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        vehicle_signal_sample_t s;
        (void)memset(&s, 0, sizeof s); /* NONE */
        (void)vehicle_signals_get(i, now_ms, &s);
        rec->did[i].value = s.physical;
        rec->did[i].stamp_ms = s.timestamp_ms;
        rec->did[i].state = (uint8_t)s.state;
    }
    snapshot_publish(&snap);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by features/derived (derived task) and the tests
bool derived_inputs_read(derived_inputs_record_t* out)
{
    if (out == NULL) {
        return false;
    }
    uint8_t idx = 0u;
    const bool fresh = snapshot_read_index(&snap, &idx);
    *out = slots[idx]; /* the one copy; no index outlives this call */
    return fresh;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
uint32_t derived_inputs_skipped_count(void)
{
    return skipped;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
bool derived_inputs_producer_fault(void)
{
    return snapshot_producer_fault(&snap);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
bool derived_inputs_consumer_fault(void)
{
    return snapshot_consumer_fault(&snap);
}
