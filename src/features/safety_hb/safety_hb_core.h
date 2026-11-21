#ifndef FEATURES_SAFETY_HB_CORE_H
#define FEATURES_SAFETY_HB_CORE_H

/*
 * The pure logic of the safety-heartbeat monitor (D-118 item 6, defs v0.30.0): rt-core's
 * receiver side of the safety MCU's 0x082 HeartbeatSafety, for the diagnostic-only DTC
 * SAFETY_HEARTBEAT_LOST (U0167-00) and a link-health counter set.
 *
 * Diagnostic input only. The received frame is E2E-checked and counted, never decoded: no
 * signal of it leaves this module, so it can feed neither 0x020, the EKF, rt-core's own
 * 0x081 (NODE_MODE, ERROR_COUNT) nor anything sent to the safety MCU. The only output is
 * the lost verdict (lost -> one DTC report) and the counters.
 *
 * Lost means silence, nothing else: the generated check_timeout reports NO_NEW_DATA when no
 * frame advanced the E2E state for more than PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS (300
 * ms = three 100 ms cycles). A single E2E error (WRONG_SEQUENCE, REPEATED, WRONG_CRC,
 * WRONG_LENGTH, BAD_ARGUMENT) goes to the link counters only; a sender that keeps
 * repeating its counter or sending bad frames still ends up silent for the check, which is
 * the generated semantics. Before the first valid frame the monitor allows the same
 * timeout from its first safety_hb_core_lost() call, i.e. the first comms pass (start-up
 * grace: the safety MCU boots on its own schedule, and the boot between open and the
 * scheduler start may be long), then reads lost. Static storage, no heap, no ISR use.
 */

#include "platform_e2e.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Link-health counters of 0x082, all saturating at UINT32_MAX. */
typedef struct {
    uint32_t frames;         /* every frame handed to safety_hb_core_rx() */
    uint32_t resyncs;        /* valid CRC, E2E (re)started: first frame or after a timeout */
    uint32_t sequence_lost;  /* valid, counter jump within max delta (OK_SOME_LOST) */
    uint32_t wrong_sequence; /* counter jump beyond max delta */
    uint32_t repeated;       /* same counter again */
    uint32_t wrong_crc;      /* corrupted or wrong DataID */
    uint32_t wrong_length;   /* DLC is not PLATFORM_HEARTBEAT_SAFETY_E2E_LENGTH (WRONG_LENGTH) */
    uint32_t other_errors;   /* BAD_ARGUMENT or an unknown status */
    uint32_t lost_events;    /* passed -> lost transitions of the verdict */
} safety_hb_link_t;

typedef struct {
    vehicle_e2e_rx_state_t e2e;
    uint32_t start_ms;  /* the first verdict call: the start-up grace runs from here */
    bool armed;         /* start_ms is set */
    bool seen;          /* a frame passed the CRC since init (grace is over for good) */
    bool lost;          /* the last verdict */
    safety_hb_link_t link;
} safety_hb_core_t;

/* Resets the state. The start-up grace is anchored later, at the first verdict call. */
void safety_hb_core_init(safety_hb_core_t* c);

/* One received 0x082 frame (payload and DLC as read from the bus) at now_ms. Runs the
 * generated platform_heartbeat_safety_e2e_check() and sorts the status into the counters.
 * Never changes the verdict by itself (only the timeout does). NULL c or data: ignored. */
void safety_hb_core_rx(safety_hb_core_t* c, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/* The verdict at now_ms, every pass: true exactly when the generated
 * platform_heartbeat_safety_e2e_check_timeout() reports NO_NEW_DATA (outside the start-up
 * grace). A NULL c reads as lost. */
bool safety_hb_core_lost(safety_hb_core_t* c, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_SAFETY_HB_CORE_H */
