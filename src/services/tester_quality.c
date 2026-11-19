#include "services/tester_quality.h"

#include <stddef.h>
#include <string.h>

static tester_quality_t held; /* not valid, no gap */

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the UDS client glue and the tests
void tester_quality_set(bool valid, const uint8_t* did_state, uint32_t bus_off_count,
                        uint32_t step_gap_window_ms)
{
    if (!valid) {
        (void)memset(&held, 0, sizeof held);
    } else if (did_state != NULL) {
        (void)memcpy(held.did, did_state, sizeof held.did);
        held.bus_off_count = bus_off_count;
        if (step_gap_window_ms > held.step_gap_max_ms) {
            held.step_gap_max_ms = step_gap_window_ms; /* latched until the sampler takes it */
        }
        held.valid = true;
    } else {
        /* no state to store */
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by services/derived_inputs and the tests
tester_quality_t tester_quality_take(void)
{
    const tester_quality_t copy = held;
    held.step_gap_max_ms = 0u; /* the sampler's reset */
    return copy;
}
