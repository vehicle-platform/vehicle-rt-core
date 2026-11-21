#ifndef FEATURES_SAFETY_HB_H
#define FEATURES_SAFETY_HB_H

/*
 * Glue of the safety-heartbeat monitor (D-118 item 6): registers the receiver for the
 * safety MCU's 0x082 on the platform bus (services/can_if, boot only, before the filters),
 * feeds the frames to safety_hb_core and reports the DTC SAFETY_HEARTBEAT_LOST
 * (services/diag, diagnostic only) every pass. It sends nothing, and no other module reads
 * its state except diagnostics (the link counters): see safety_hb_core.h.
 */

#include "features/safety_hb/safety_hb_core.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    safety_hb_core_t core;
    bool opened; /* the receiver is registered: the only state that monitors */
} safety_hb_t;

/* Resets m (the start-up grace starts at the first safety_hb_step()) and registers the
 * 0x082 receiver on the platform port. False (monitor stays closed) if can_if refused it
 * (sealed, listed already, full). Boot only, before comms_apply_filters(). */
bool safety_hb_open(safety_hb_t* m);

/* Every comms pass, after can_if_dispatch(PLATFORM): reports the lost verdict as the test
 * result of DTC SAFETY_HEARTBEAT_LOST (level-triggered). Nothing unless opened. */
void safety_hb_step(safety_hb_t* m);

/* The link-health counters (diagnostics, tests); all zero for NULL. */
safety_hb_link_t safety_hb_link(const safety_hb_t* m);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_SAFETY_HB_H */
