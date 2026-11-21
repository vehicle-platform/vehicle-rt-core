#ifndef APP_HOST_SIM_SAFETY_H
#define APP_HOST_SIM_SAFETY_H

/*
 * Simulated safety MCU for the host SIL (not firmware): a platform-bus node that sends the
 * safety node's 0x082 HeartbeatSafety at its generated cycle with a valid E2E (core
 * vehicle_e2e_protect() and the generated DataID), so rt-core's 0x082 monitor
 * (features/safety_hb, D-118 item 6) sees a live peer. The payload is all zero: rt-core
 * never decodes it. In-process only: it takes a vbus_t and has no SocketCAN path (a CI
 * grep keeps it that way), so it can never put a frame on a real or vcan interface. Silence can be switched on to rehearse SAFETY_HEARTBEAT_LOST.
 */

#include "hal/host/vbus.h"
#include "vehicle_e2e.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    vbus_t* bus;
    uint8_t node;
    vehicle_e2e_tx_state_t tx;
    uint32_t next_ms;
    bool started;
    bool silent; /* send nothing */
    uint32_t sent;
} sim_safety_t;

/* Attaches the peer to the bus. False if the bus is full. */
bool sim_safety_init(sim_safety_t* s, vbus_t* bus);

/* Sends 0x082 when its cycle is due (the first one at once). */
void sim_safety_step(sim_safety_t* s, uint32_t now_ms);

#endif /* APP_HOST_SIM_SAFETY_H */
