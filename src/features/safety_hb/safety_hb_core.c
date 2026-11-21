#include "features/safety_hb/safety_hb_core.h"

#include "platform.h"

#include <stddef.h>

static void bump(uint32_t* counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

void safety_hb_core_init(safety_hb_core_t* c, uint32_t start_ms)
{
    if (c == NULL) {
        return;
    }
    vehicle_e2e_rx_init(&c->e2e);
    c->start_ms = start_ms;
    c->seen = false;
    c->lost = false;
    c->link.frames = 0u;
    c->link.resyncs = 0u;
    c->link.sequence_lost = 0u;
    c->link.wrong_sequence = 0u;
    c->link.repeated = 0u;
    c->link.wrong_crc = 0u;
    c->link.wrong_length = 0u;
    c->link.other_errors = 0u;
    c->link.lost_events = 0u;
}

void safety_hb_core_rx(safety_hb_core_t* c, const uint8_t* data, uint8_t dlc, uint32_t now_ms)
{
    if ((c == NULL) || (data == NULL)) {
        return;
    }
    bump(&c->link.frames);
    /* The generated wrapper checks the length first (v0.30.0): a wrong DLC leaves the E2E
     * state untouched and reads WRONG_LENGTH, which is invalid like every error status. */
    switch (platform_heartbeat_safety_e2e_check(data, (size_t)dlc, &c->e2e, now_ms)) {
    case VEHICLE_E2E_OK:
        c->seen = true;
        break;
    case VEHICLE_E2E_OK_SOME_LOST:
        c->seen = true;
        bump(&c->link.sequence_lost);
        break;
    case VEHICLE_E2E_INITIAL:
        c->seen = true;
        bump(&c->link.resyncs);
        break;
    case VEHICLE_E2E_REPEATED:
        bump(&c->link.repeated);
        break;
    case VEHICLE_E2E_WRONG_SEQUENCE:
        bump(&c->link.wrong_sequence);
        break;
    case VEHICLE_E2E_WRONG_CRC:
        bump(&c->link.wrong_crc);
        break;
    case VEHICLE_E2E_WRONG_LENGTH:
        bump(&c->link.wrong_length);
        break;
    case VEHICLE_E2E_NO_NEW_DATA:
    case VEHICLE_E2E_BAD_ARGUMENT:
    default:
        bump(&c->link.other_errors);
        break;
    }
}

bool safety_hb_core_lost(safety_hb_core_t* c, uint32_t now_ms)
{
    if (c == NULL) {
        return true;
    }
    bool lost = false;
    if (platform_heartbeat_safety_e2e_check_timeout(&c->e2e, now_ms) == VEHICLE_E2E_NO_NEW_DATA) {
        /* never seen: the same timeout from init is the start-up grace */
        lost = c->seen ||
               ((uint32_t)(now_ms - c->start_ms) > (uint32_t)PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS);
    }
    if (lost && !c->lost) {
        bump(&c->link.lost_events);
    }
    c->lost = lost;
    return lost;
}
