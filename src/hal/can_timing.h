#ifndef HAL_CAN_TIMING_H
#define HAL_CAN_TIMING_H

/*
 * FDCAN nominal bit-timing arithmetic (pure, HAL-free, no heap, header-only static inline
 * like features/uds/dtc_snapshot_core.h, so no external linkage). T13 calls can_timing_ok()
 * with the values it is about to program (or reads back from NBTP) before HAL_FDCAN_Start.
 *
 * Bit time = (1 + seg1 + seg2) time quanta of (prescaler / kernel_hz) seconds; the sync
 * segment is one quantum and the sample point sits at the end of seg1. Inputs are the
 * decoded values (NBTP field + 1), limited to the FDCAN nominal ranges (RM0433 FDCAN_NBTP):
 * prescaler 1..512, seg1 1..256, seg2 1..128, sjw 1..128 and sjw <= seg2 (ISO 11898-1).
 * Inside these ranges the divisor prescaler * quanta <= 512 * 385 fits 32 bits, and the
 * rate check multiplies in 64 bits, so no product can wrap.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sample point window accepted by can_timing_ok(), percent of the bit time (CiA 301). */
#define CAN_TIMING_SAMPLE_MIN_PCT 75u
#define CAN_TIMING_SAMPLE_MAX_PCT 90u

/* FDCAN nominal bit-timing field ranges (decoded values). */
#define CAN_TIMING_PRESCALER_MAX 512u
#define CAN_TIMING_SEG1_MAX 256u
#define CAN_TIMING_SEG2_MAX 128u

/* prescaler * (1 + seg1 + seg2), or 0 when a value is outside its FDCAN range. */
static inline uint32_t can_timing_divisor(uint32_t prescaler, uint32_t seg1, uint32_t seg2)
{
    uint32_t divisor = 0u;

    if ((prescaler >= 1u) && (prescaler <= CAN_TIMING_PRESCALER_MAX) && (seg1 >= 1u) &&
        (seg1 <= CAN_TIMING_SEG1_MAX) && (seg2 >= 1u) && (seg2 <= CAN_TIMING_SEG2_MAX)) {
        divisor = prescaler * (1u + seg1 + seg2); /* <= 512 * 385, no wrap */
    }
    return divisor;
}

/*
 * Resulting bit rate in bit/s: kernel_hz / (prescaler * (1 + seg1 + seg2)), integer
 * division (truncated). Returns 0 for kernel_hz == 0 or any value outside its range.
 */
static inline uint32_t can_timing_bitrate(uint32_t kernel_hz, uint32_t prescaler,
                                          uint32_t seg1, uint32_t seg2)
{
    const uint32_t divisor = can_timing_divisor(prescaler, seg1, seg2);
    uint32_t bps = 0u;

    if (divisor != 0u) {
        bps = kernel_hz / divisor;
    }
    return bps;
}

/*
 * True only if every value is in its range, 1 <= sjw <= seg2, the bit rate is exactly
 * expected_bps (no truncation, expected_bps != 0) and the sample point
 * (1 + seg1) / (1 + seg1 + seg2) lies in [CAN_TIMING_SAMPLE_MIN_PCT, CAN_TIMING_SAMPLE_MAX_PCT].
 */
static inline bool can_timing_ok(uint32_t kernel_hz, uint32_t prescaler, uint32_t seg1,
                                 uint32_t seg2, uint32_t sjw, uint32_t expected_bps)
{
    const uint32_t divisor = can_timing_divisor(prescaler, seg1, seg2);
    bool ok = false;

    if ((divisor != 0u) && (sjw >= 1u) && (sjw <= seg2) && (kernel_hz != 0u) &&
        (expected_bps != 0u) && ((uint64_t)kernel_hz == ((uint64_t)expected_bps * divisor))) {
        const uint32_t quanta = 1u + seg1 + seg2;
        const uint32_t sample = 1u + seg1;
        ok = ((sample * 100u) >= (CAN_TIMING_SAMPLE_MIN_PCT * quanta)) &&
             ((sample * 100u) <= (CAN_TIMING_SAMPLE_MAX_PCT * quanta));
    }
    return ok;
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_TIMING_H */
