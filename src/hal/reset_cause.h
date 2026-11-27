#ifndef HAL_RESET_CAUSE_H
#define HAL_RESET_CAUSE_H

/*
 * Reset-cause decoding of the STM32H7 RCC_RSR register (RM0433), pure and HAL-free, header-only
 * static inline like hal/can_timing.h. The register read, the RMVF clear and the storage live in
 * cubemx/User/boot_checks.c (it reads RCC->RSR before anything else clears it); this header only
 * turns the captured value into one cause.
 *
 * Several flags are set at once on most resets (every reset also drives the NRST pin flag, a
 * power-on reset also sets the BOR flag), so the decode is a fixed priority, most specific
 * first: IWDG, WWDG, CPU lockup/reset, low power, software, POR, BOR, pin. A value with none of
 * the known flags (including 0) is RESET_CAUSE_UNKNOWN. The bit positions are the RM0433
 * RCC_RSR positions; the cubemx build asserts them against the CMSIS RCC_RSR_*_Msk macros.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RCC_RSR flag bits (RM0433). */
#define RESET_CAUSE_RSR_RMVF 0x00010000u     /* write 1: clear all flags */
#define RESET_CAUSE_RSR_CPURSTF 0x00020000u  /* CPU reset (lockup) */
#define RESET_CAUSE_RSR_D1RSTF 0x00080000u   /* D1 domain power switch reset */
#define RESET_CAUSE_RSR_D2RSTF 0x00100000u   /* D2 domain power switch reset */
#define RESET_CAUSE_RSR_BORRSTF 0x00200000u  /* brown-out reset */
#define RESET_CAUSE_RSR_PINRSTF 0x00400000u  /* NRST pin reset */
#define RESET_CAUSE_RSR_PORRSTF 0x00800000u  /* power-on / power-down reset */
#define RESET_CAUSE_RSR_SFTRSTF 0x01000000u  /* software reset (SYSRESETREQ) */
#define RESET_CAUSE_RSR_IWDG1RSTF 0x04000000u
#define RESET_CAUSE_RSR_WWDG1RSTF 0x10000000u
#define RESET_CAUSE_RSR_LPWRRSTF 0x40000000u /* illegal Stop/Standby entry */

typedef enum {
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_IWDG,
    RESET_CAUSE_WWDG,
    RESET_CAUSE_LOCKUP,
    RESET_CAUSE_LOW_POWER,
    RESET_CAUSE_SOFTWARE,
    RESET_CAUSE_POWER_ON,
    RESET_CAUSE_BROWN_OUT,
    RESET_CAUSE_PIN
} reset_cause_t;

/* Decode an RCC_RSR value (the RMVF bit itself is ignored). */
static inline reset_cause_t reset_cause_decode(uint32_t rsr)
{
    const uint32_t flags = rsr & ~RESET_CAUSE_RSR_RMVF;
    reset_cause_t cause = RESET_CAUSE_UNKNOWN;

    if ((flags & RESET_CAUSE_RSR_IWDG1RSTF) != 0u) {
        cause = RESET_CAUSE_IWDG;
    } else if ((flags & RESET_CAUSE_RSR_WWDG1RSTF) != 0u) {
        cause = RESET_CAUSE_WWDG;
    } else if ((flags & RESET_CAUSE_RSR_CPURSTF) != 0u) {
        cause = RESET_CAUSE_LOCKUP;
    } else if ((flags & (RESET_CAUSE_RSR_LPWRRSTF | RESET_CAUSE_RSR_D1RSTF |
                       RESET_CAUSE_RSR_D2RSTF)) != 0u) {
        cause = RESET_CAUSE_LOW_POWER;
    } else if ((flags & RESET_CAUSE_RSR_SFTRSTF) != 0u) {
        cause = RESET_CAUSE_SOFTWARE;
    } else if ((flags & RESET_CAUSE_RSR_PORRSTF) != 0u) {
        cause = RESET_CAUSE_POWER_ON;
    } else if ((flags & RESET_CAUSE_RSR_BORRSTF) != 0u) {
        cause = RESET_CAUSE_BROWN_OUT;
    } else if ((flags & RESET_CAUSE_RSR_PINRSTF) != 0u) {
        cause = RESET_CAUSE_PIN;
    } else {
        /* no known flag: stays UNKNOWN */
    }
    return cause;
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_RESET_CAUSE_H */
