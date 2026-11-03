/*
 * Ride-summary record encoder (D-098). See ride_records_core.h.
 */
#include "services/ride_records_core.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__)
#error "ride_records_core.c must not be built with -ffast-math"
#endif

typedef struct {
    uint8_t did_idx;      /* platform_uds_did_index_t */
    uint8_t length;       /* data record bytes */
    uint8_t version_byte; /* header: layout version */
    uint8_t counter_byte; /* header: closed ride's stop-epoch counter, big-endian */
    uint8_t counter_len;
    uint8_t flags_byte;   /* header: RIDE_CLOSED (other bits reserved, zero) */
    uint8_t closed_mask;
} ride_record_desc_t;

/* The served records, gen/ table order, gen/ symbols only (invariant 2). */
static const ride_record_desc_t records[] = {
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_USAGE, (uint8_t)PLATFORM_UDS_DID_RIDE_USAGE_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_USAGE_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_FUELING, (uint8_t)PLATFORM_UDS_DID_RIDE_FUELING_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_FUELING_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_FUELING_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_FUELING_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_FUELING_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_FUELING_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_THERMAL, (uint8_t)PLATFORM_UDS_DID_RIDE_THERMAL_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_THERMAL_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_THERMAL_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_THERMAL_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_THERMAL_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_THERMAL_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_ELECTRICAL, (uint8_t)PLATFORM_UDS_DID_RIDE_ELECTRICAL_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_ELECTRICAL_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_ELECTRICAL_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_ELECTRICAL_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_ELECTRICAL_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_ELECTRICAL_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_ENGINE_STRESS, (uint8_t)PLATFORM_UDS_DID_RIDE_ENGINE_STRESS_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_ENGINE_STRESS_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_ENGINE_STRESS_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_ENGINE_STRESS_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_ENGINE_STRESS_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_ENGINE_STRESS_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_DRIVETRAIN, (uint8_t)PLATFORM_UDS_DID_RIDE_DRIVETRAIN_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_DRIVETRAIN_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_DRIVETRAIN_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_DRIVETRAIN_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_DRIVETRAIN_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_DRIVETRAIN_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_CALIBRATION, (uint8_t)PLATFORM_UDS_DID_RIDE_CALIBRATION_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_CALIBRATION_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_CALIBRATION_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_CALIBRATION_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_CALIBRATION_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_CALIBRATION_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_RIDER_STYLE, (uint8_t)PLATFORM_UDS_DID_RIDE_RIDER_STYLE_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_RIDER_STYLE_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_RIDER_STYLE_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_RIDER_STYLE_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_RIDER_STYLE_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_RIDER_STYLE_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_ECU_HEALTH, (uint8_t)PLATFORM_UDS_DID_RIDE_ECU_HEALTH_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_ECU_HEALTH_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_ECU_HEALTH_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_ECU_HEALTH_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_ECU_HEALTH_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_ECU_HEALTH_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_WINDOW_KEY_ON, (uint8_t)PLATFORM_UDS_DID_RIDE_WINDOW_KEY_ON_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_KEY_ON_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_KEY_ON_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_KEY_ON_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_KEY_ON_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_KEY_ON_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_WINDOW_CRANK, (uint8_t)PLATFORM_UDS_DID_RIDE_WINDOW_CRANK_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRANK_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRANK_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRANK_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRANK_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRANK_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_WINDOW_WARM_IDLE, (uint8_t)PLATFORM_UDS_DID_RIDE_WINDOW_WARM_IDLE_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_WARM_IDLE_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_WARM_IDLE_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_WARM_IDLE_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_WARM_IDLE_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_WARM_IDLE_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_WINDOW_CRUISE, (uint8_t)PLATFORM_UDS_DID_RIDE_WINDOW_CRUISE_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRUISE_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRUISE_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRUISE_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRUISE_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_CRUISE_RIDE_CLOSED_MASK},
    {(uint8_t)PLATFORM_UDS_IDX_RIDE_WINDOW_POST_STOP, (uint8_t)PLATFORM_UDS_DID_RIDE_WINDOW_POST_STOP_LENGTH,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_POST_STOP_LAYOUT_VERSION_BYTE, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_POST_STOP_RIDE_COUNTER_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_POST_STOP_RIDE_COUNTER_LENGTH, (uint8_t)PLATFORM_UDS_RIDE_WINDOW_POST_STOP_RIDE_CLOSED_BYTE,
     (uint8_t)PLATFORM_UDS_RIDE_WINDOW_POST_STOP_RIDE_CLOSED_MASK},
};

#define RIDE_RECORD_COUNT ((uint32_t)(sizeof(records) / sizeof(records[0])))

/* Offset of record did_idx in the flat buffer and its descriptor; false if not served. */
static bool find_record(uint32_t did_idx, uint32_t* offset, const ride_record_desc_t** desc)
{
    uint32_t off = 0u;
    bool found = false;
    for (uint32_t i = 0u; (i < RIDE_RECORD_COUNT) && !found; i++) {
        if ((uint32_t)records[i].did_idx == did_idx) {
            *offset = off;
            *desc = &records[i];
            found = true;
        } else {
            off += (uint32_t)records[i].length;
        }
    }
    return found;
}

static void ride_put_be(uint8_t* p, uint32_t n, uint32_t v)
{
    for (uint32_t i = 0u; i < n; i++) {
        const uint32_t shift = 8u * (n - 1u - i);
        p[i] = (uint8_t)((v >> shift) & 0xFFu);
    }
}

uint32_t ride_records_field_raw(const platform_uds_ride_field_t* field, bool valid, float value)
{
    if (field == NULL) {
        return 0u;
    }
    uint32_t raw = field->na;
    if (valid && (isfinite(value) != 0) && (field->resolution > 0.0f)) {
        /* float32 like vehicle-server's encoder (D-098 item 5). The bounds keep lrintf
         * defined (its long is 32 bit on the M7): one step beyond 0..raw_max either way
         * can only round to a value outside it, which is NOT_AVAILABLE anyway. */
        const float x = (value - field->range_min) / field->resolution;
        const float above = (float)field->raw_max + 1.0f;
        if ((isfinite(x) != 0) && (x > -1.0f) && (x < above) && (x < (float)LONG_MAX)) {
            const long r = lrintf(x); /* round half to even, never roundf/lroundf */
            if ((r >= 0L) && ((uint32_t)r <= field->raw_max)) {
                raw = (uint32_t)r;
            }
        }
    }
    return raw;
}

void ride_records_encode(ride_records_value_fn value, bool closed, uint16_t counter, uint8_t* buf)
{
    if (buf == NULL) {
        return;
    }
    const bool have = closed && (value != NULL);
    (void)memset(buf, 0, RIDE_RECORDS_BYTES);
    uint32_t off = 0u;
    for (uint32_t i = 0u; i < RIDE_RECORD_COUNT; i++) {
        const ride_record_desc_t* const d = &records[i];
        uint8_t* const rec = &buf[off];
        rec[d->version_byte] = (uint8_t)PLATFORM_UDS_RIDE_RECORD_LAYOUT_VERSION;
        ride_put_be(&rec[d->counter_byte], (uint32_t)d->counter_len, have ? (uint32_t)counter : 0u);
        rec[d->flags_byte] = have ? d->closed_mask : 0u;
        off += (uint32_t)d->length;
    }
    for (uint32_t i = 0u; i < PLATFORM_UDS_RIDE_FIELD_COUNT; i++) {
        const platform_uds_ride_field_t* const f = &platform_uds_ride_fields[i];
        uint32_t rec_off = 0u;
        const ride_record_desc_t* d = NULL;
        /* a row outside its record cannot come from codegen; skipped, never written past */
        if (find_record((uint32_t)f->did_idx, &rec_off, &d) &&
            (((uint32_t)f->byte + (uint32_t)f->length) <= (uint32_t)d->length)) {
            float v = 0.0f;
            const bool valid = have && value((uint32_t)f->signal, &v);
            ride_put_be(&buf[rec_off + (uint32_t)f->byte], (uint32_t)f->length,
                   ride_records_field_raw(f, valid, v));
        }
    }
}

bool ride_records_copy(const uint8_t* buf, uint32_t did_idx, uint8_t* out, uint16_t len)
{
    uint32_t off = 0u;
    const ride_record_desc_t* d = NULL;
    if ((buf == NULL) || (out == NULL) || !find_record(did_idx, &off, &d) ||
        ((uint32_t)len != (uint32_t)d->length)) {
        return false;
    }
    (void)memcpy(out, &buf[off], (size_t)len);
    return true;
}

// cppcheck-suppress misra-c2012-8.7 ; DEV-002: the tests and future diagnostics
bool ride_records_is_record(uint32_t did_idx)
{
    uint32_t off = 0u;
    const ride_record_desc_t* d = NULL;
    return find_record(did_idx, &off, &d);
}
