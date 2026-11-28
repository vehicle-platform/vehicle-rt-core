#ifndef FEATURES_POWER_KEYOFF_CORE_H
#define FEATURES_POWER_KEYOFF_CORE_H

/*
 * Pure key-off power-down state machine of rt-core (D-138 item 2): no HAL, RTOS or CAN port
 * access. The glue (a later card, T13) feeds it the KL15 sense, the battery sample and the
 * Pi 5 acknowledgement (platform 0x122), and applies its outputs: the HOLD pin, the request
 * frame (platform 0x121) and "close the logs".
 *
 * Contract:
 *  - HOLD is asserted in RUNNING and SHUTTING_DOWN and released only in DONE, so a zeroed
 *    or never-initialised state keeps the supply on (fail-safe for the filesystem).
 *  - The step is O(1), has no loop and never blocks: the comms task may call it every pass.
 *  - A hung Pi never holds the supply past KEYOFF_TIMEOUT_MS: the supply is cut at the
 *    deadline with or without an acknowledgement.
 *  - A key-off needs a valid KL15 low held for KEYOFF_KEY_DEBOUNCE_MS with the engine not
 *    running; an invalid sense never starts one and never cancels one (the timeout bounds it).
 *  - The low-voltage path only runs with KL15 on and the engine known to be stopped, so a
 *    weak charge at idle or a cranking dip never halts the Pi while riding.
 *  - HOLD is also asserted while KL15 is valid and on, whatever the phase; only a real key
 *    cycle (KL15 off for the debounce, then on for the debounce, after the sequence started)
 *    returns DONE to RUNNING (logs reopen); for low voltage also an engine start. A corrupted phase value is treated as a
 *    new bounded shutdown.
 *  - Hardware must still cap HOLD (retriggerable) so an rt-core reset loop cannot hold the
 *    supply indefinitely: glue/hardware card, not this module.
 *  - The safety MCU does not depend on any of this (invariant 3, D-138 item 3).
 *
 * Every frame value comes from gen/ (platform.h); this file names no CAN literal.
 */

#include "platform.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bounded wait for the Pi 5 between the request and the supply cut. PROVISIONAL (D-138 item 2:
 * 20-30 s, to be measured on the bike, T73). */
#define KEYOFF_TIMEOUT_MS 25000u

/* Low-voltage cut-off (D-138 item 4: below 12.2 V at rest) and how long the battery sample
 * must stay below it, so a starter dip or a load step never starts a shutdown. The debounce
 * is PROVISIONAL. */
#define KEYOFF_LOW_VOLTAGE_MV 12200u
#define KEYOFF_LOW_VOLTAGE_DEBOUNCE_MS 10000u
/* A low-voltage sequence is cancelled before HALTING once the battery recovers to this level
 * (hysteresis above KEYOFF_LOW_VOLTAGE_MV; provisional until T73). */
#define KEYOFF_LOW_VOLTAGE_RECOVER_MV 12600u

/* A KL15 level must hold this long before it counts (connector glitch, cranking brown-out).
 * PROVISIONAL. */
#define KEYOFF_KEY_DEBOUNCE_MS 500u

typedef enum {
    KEYOFF_PHASE_RUNNING = 0, /* zero state: HOLD asserted, no request */
    KEYOFF_PHASE_SHUTTING_DOWN = 1,
    KEYOFF_PHASE_DONE = 2 /* HOLD released; terminal until the supply is cut */
} keyoff_phase_t;

typedef struct {
    keyoff_phase_t phase;
    uint8_t reason;           /* PLATFORM_SHUTDOWN_REQUEST_SHUTDOWN_REASON_*_CHOICE */
    uint8_t request_id;       /* incremented once per shutdown sequence, 4 bit */
    bool halting_seen;        /* the Pi acknowledged HALTING for this sequence */
    bool seen_off;            /* KL15 read valid and off since the sequence started: a later
                               * debounced key-on is a real key cycle (edge), not a level */
    bool low_active;          /* the battery sample is below the cut-off */
    bool off_active;          /* KL15 valid and low, engine not running (key-off start) */
    bool cyc_off_active;      /* KL15 valid and low (the off half of a key cycle, no veto) */
    bool on_active;           /* KL15 valid and high */
    uint32_t off_since_ms;
    uint32_t cyc_off_since_ms;
    uint32_t on_since_ms;
    uint32_t sequence_ms;     /* time the current sequence started */
    uint32_t low_since_ms;    /* time the sample first fell below the cut-off */
} keyoff_t;

typedef struct {
    bool kl15_valid;          /* false until the KL15 sense is measured: never starts or cancels
                               * a sequence; in DONE an invalid sense releases HOLD (the real
                               * KL15 still holds the supply through the diode-OR) */
    bool kl15_on;
    bool engine_running;      /* engine speed valid and above zero: vetoes a key-off */
    bool engine_stopped;      /* engine speed valid and zero; unknown = false: no low-voltage trip */
    bool battery_valid;
    uint16_t battery_mv;
    uint8_t ack_state;        /* PLATFORM_SHUTDOWN_ACK_SHUTDOWN_ACK_STATE_*_CHOICE, 0 if none */
    uint8_t ack_id;
    bool ack_fresh;           /* an acknowledgement frame passed its timeout and checks */
} keyoff_in_t;

typedef struct {
    bool hold;                /* HOLD output */
    uint8_t request;          /* SHUTDOWN_REQUEST choice value */
    uint8_t reason;           /* SHUTDOWN_REASON choice value */
    uint8_t request_id;
    uint8_t deadline_s;       /* seconds left until the supply is cut, saturated at 255 */
    bool close_logs;          /* true from the start of the sequence */
} keyoff_out_t;

/* Zero state with the request counter at 0. */
void keyoff_init(keyoff_t* k);

/* One pass at timebase now_ms (wrap-safe). */
void keyoff_step(keyoff_t* k, const keyoff_in_t* in, uint32_t now_ms, keyoff_out_t* out);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_POWER_KEYOFF_CORE_H */
