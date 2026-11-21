#include "features/safety_hb/safety_hb.h"

#include "platform.h"
#include "platform_uds.h"
#include "services/can_if.h"
#include "services/diag.h"
#include "services/timebase.h"

#include <stddef.h>
#include <string.h>

static void safety_hb_on_rx(void* ctx, const can_frame_t* frame)
{
    // cppcheck-suppress misra-c2012-11.5 ; DEV-003: ctx is the monitor this file registered
    safety_hb_t* const m = (safety_hb_t*)ctx;
    if ((m != NULL) && (frame != NULL)) {
        safety_hb_core_rx(&m->core, frame->data, frame->dlc, timebase_now_ms());
    }
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by the host program and the H7 setup
bool safety_hb_open(safety_hb_t* m)
{
    if (m == NULL) {
        return false;
    }
    (void)memset(m, 0, sizeof *m);
    safety_hb_core_init(&m->core, timebase_now_ms());
    m->opened = (can_if_register_rx(CAN_PORT_PLATFORM, PLATFORM_HEARTBEAT_SAFETY_FRAME_ID,
                                    PLATFORM_HEARTBEAT_SAFETY_IS_EXTENDED != 0,
                                    safety_hb_on_rx,
                                    m) == CAN_IF_OK);
    return m->opened;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: called by app/comms and the tests
void safety_hb_step(safety_hb_t* m)
{
    if ((m == NULL) || !m->opened) {
        return;
    }
    diag_dtc_report((uint32_t)PLATFORM_UDS_DTC_IDX_SAFETY_HEARTBEAT_LOST,
                    safety_hb_core_lost(&m->core, timebase_now_ms()));
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: diagnostics and the tests
safety_hb_link_t safety_hb_link(const safety_hb_t* m)
{
    safety_hb_link_t zero;
    (void)memset(&zero, 0, sizeof zero);
    return (m != NULL) ? m->core.link : zero;
}
