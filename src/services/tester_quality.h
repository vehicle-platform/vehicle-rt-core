#ifndef SERVICES_TESTER_QUALITY_H
#define SERVICES_TESTER_QUALITY_H

/*
 * The vehicle tester's data-quality state as a service (D-104, data_quality_mask): per DID
 * whether the tester's last read ended in a negative response, or the DID is stopped (D-068)
 * or skipped (cooldown), the vehicle-bus bus-off count, and the longest tester poll gap
 * (D-053) since the sampler last took it. It copies the contract of services/tester_epoch.
 *
 * Single writer: the UDS client glue (features/uds/uds_client, comms task) and nothing else
 * (CI greps the includes of this service and of services/derived_inputs; any other call of
 * tester_quality_set() is a review finding). After every client step while the client is open
 * it passes the per-DID states, the vehicle port's bus-off count (the value of 0xFD02
 * VEHICLE_BUS_OFFS before its clamp) and the core's step-gap window (the longest poll gap
 * since the previous step); the service max-merges the window into its LATCHED gap. It is
 * not refreshed while the client is closed, and uds_client_open() resets it to "not valid"
 * (tester_quality_set(false, ...)), as the epoch service is reset to closed.
 *
 * "Not valid" is what a never-set or a closed tester reads as: the consumer takes it as
 * unknown, and unknown counts as bad (D-104). valid is also false in the boot state.
 *
 * Reader: services/derived_inputs (derived_inputs_publish, comms task) only. It calls
 * tester_quality_take(), which returns the state and clears the latched gap: the reset of the
 * window belongs to the sampler, not to the writer.
 *
 * The state enum is this header's own (not the client core's); the glue maps. Comms task
 * only, like services/vehicle_signals (no atomics, not ISR-safe). Includes no feature header.
 */

#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TESTER_QUALITY_OK = 0,      /* no negative response to the last read, not stopped, not skipped */
    TESTER_QUALITY_NRC = 1,     /* the last read got a negative response (other than 0x78) */
    TESTER_QUALITY_STOPPED = 2, /* an unverified DID stopped by the tester (D-068 item 2) */
    TESTER_QUALITY_SKIPPED = 3  /* in skip cooldown */
} tester_quality_state_t;

typedef struct {
    bool valid;                                 /* false: never set, or the tester was reopened */
    uint8_t did[VEHICLE_CL250_DID_COUNT];       /* tester_quality_state_t per vehicle_cl250_dids[] index */
    uint32_t bus_off_count;                     /* vehicle-bus bus-offs since boot (saturating source) */
    uint32_t step_gap_max_ms;                   /* longest poll gap since the previous take */
} tester_quality_t;

/*
 * Writer: the UDS client glue. valid false resets everything (the arguments are ignored).
 * valid true stores did_state[VEHICLE_CL250_DID_COUNT] and bus_off_count and max-merges
 * step_gap_window_ms into the latched gap. A NULL did_state with valid true is a no-op.
 */
void tester_quality_set(bool valid, const uint8_t* did_state, uint32_t bus_off_count,
                        uint32_t step_gap_window_ms);

/* Reader: derived_inputs_publish. Returns the state and clears the latched gap. All zero and
 * not valid before any set. */
tester_quality_t tester_quality_take(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_TESTER_QUALITY_H */
