#include "app/host/sim_safety.h"

#include "platform.h"
#include "platform_e2e.h"

#include <string.h>

bool sim_safety_init(sim_safety_t* s, vbus_t* bus)
{
    (void)memset(s, 0, sizeof *s);
    s->bus = bus;
    vehicle_e2e_tx_init(&s->tx);
    return vbus_attach(bus, &s->node);
}

void sim_safety_step(sim_safety_t* s, uint32_t now_ms)
{
    if (s->silent) {
        return;
    }
    if (s->started && ((int32_t)(now_ms - s->next_ms) < 0)) {
        return;
    }
    can_frame_t f;
    (void)memset(&f, 0, sizeof f);
    f.id = PLATFORM_HEARTBEAT_SAFETY_FRAME_ID;
    f.extended = PLATFORM_HEARTBEAT_SAFETY_IS_EXTENDED != 0;
    f.dlc = (uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH;
    if (vehicle_e2e_protect(PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID, f.data,
                            PLATFORM_HEARTBEAT_SAFETY_LENGTH, &s->tx) != VEHICLE_E2E_OK) {
        return;
    }
    if (vbus_send(s->bus, s->node, &f) == CAN_PORT_OK) {
        s->sent++;
    }
    s->next_ms = s->started ? (s->next_ms + PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS)
                            : (now_ms + PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS);
    s->started = true;
}
