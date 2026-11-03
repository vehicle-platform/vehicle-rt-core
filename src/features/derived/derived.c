#include "features/derived/derived.h"

#include "services/derived_inputs.h"
#include "services/ride_records.h"
#include "services/vehicle_signals.h"

#include <stddef.h>

static derived_core_t core;
static derived_in_t inputs[VEHICLE_CL250_DID_COUNT];
static derived_inputs_record_t latest; /* the newest record read */
static uint32_t published_closed;      /* closed_count of the last ride-record publish */

/* ride_records_value_fn over the last closed ride (D-098): VALID only; NOT_COMPUTED,
 * NO_DATA and INVALID read as no value (NOT_AVAILABLE on the wire). */
static bool closed_ride_value(uint32_t signal, float* value)
{
    derived_out_t o;
    const bool ok = derived_core_ride_out(&core, DERIVED_RIDE_PREVIOUS, signal, &o) &&
                    (o.status == DERIVED_VALID);
    if (ok) {
        *value = o.value;
    }
    return ok;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the boot setup (host program, H7) and the tests
void derived_init(void)
{
    derived_core_init(&core);
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        inputs[i].value = 0.0f;
        inputs[i].stamp_ms = 0u;
        inputs[i].present = false;
        latest.did[i].value = 0.0f;
        latest.did[i].stamp_ms = 0u;
        latest.did[i].state = (uint8_t)VEHICLE_SIGNAL_NONE;
    }
    latest.pass = 0u;
    latest.published_ms = 0u;
    latest.epoch_counter = 0u;
    latest.epoch_open = 0u;
    published_closed = 0u; /* the boot records (no ride) come from ride_records_init() */
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (host program, H7) and the tests
void derived_step(uint32_t now_ms)
{
    (void)derived_inputs_read(&latest); /* the one consumer call site */
    for (uint32_t i = 0u; i < VEHICLE_CL250_DID_COUNT; i++) {
        /* The core judges every DID by its own stale_after_ms from the sample stamp, the
         * limit vehicle_signals uses for STALE too, so a sample counts as present whenever
         * one exists (and a sticky STALE only ever coincides with the core's verdict). */
        inputs[i].present = latest.did[i].state != (uint8_t)VEHICLE_SIGNAL_NONE;
        inputs[i].value = latest.did[i].value;
        inputs[i].stamp_ms = latest.did[i].stamp_ms;
    }
    const derived_epoch_t epoch = {true, latest.epoch_open != 0u, latest.epoch_counter};
    derived_core_step_epoch(&core, now_ms, inputs, &epoch);
    /* D-098 item 6: the records go to the comms task only when a ride closed, encoded here
     * (the derived task), so the comms pass only copies bytes. One publish per close; a
     * publish that did nothing (no init, producer fault) is not retried. */
    if (core.ride.closed_count != published_closed) {
        published_closed = core.ride.closed_count;
        uint16_t counter = 0u;
        const bool closed = derived_core_ride_closed(&core, &counter);
        (void)ride_records_publish(closed_ride_value, closed, counter); /* the one producer call site */
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (future logger) and the tests
bool derived_get(uint32_t sig, derived_out_t* out)
{
    const derived_out_t* const o = derived_core_out(&core, sig);
    if ((o == NULL) || (out == NULL)) {
        return false;
    }
    *out = *o;
    return true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the derived task (future logger) and the tests
bool derived_ride_get(derived_ride_which_t which, uint32_t sig, derived_out_t* out)
{
    return derived_core_ride_out(&core, which, sig, out);
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: tests and diagnostics
const derived_core_t* derived_core(void)
{
    return &core;
}
