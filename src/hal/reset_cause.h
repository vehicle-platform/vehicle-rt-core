#ifndef HAL_RESET_CAUSE_H
#define HAL_RESET_CAUSE_H

/*
 * Reset-cause decoding of the STM32H7 RCC_RSR register (RM0433), pure and HAL-free, header-only
 * static inline like hal/can_timing.h. The register read, the RMVF clear and the storage live in
 * cubemx/User/boot_checks.c (it reads RCC->RSR before anything else clears it); this header only
 * turns the captured value into one cause.
 *
 * Per the RM0433 reset-source table, several flags are set at once: CPURSTF is set by every
 * system reset, PINRSTF by every reset that pulls NRST (all but a domain standby exit), BORRSTF
 * by a power-on reset, and D1RSTF/D2RSTF also by a power-on reset. So the decode is a fixed
 * priority, most specific source first: IWDG, WWDG, low-power violation, software, POR, BOR,
 * pin, then a domain-only standby exit (D1/D2 without a system reset source), then a bare CPU
 * reset. A value with none of the known flags (including 0) is RESET_CAUSE_UNKNOWN. The bit
 * positions are the RM0433 RCC_RSR positions; the cubemx build asserts them against the CMSIS
 * RCC_RSR_*_Msk macros.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RCC_RSR flag bits (RM0433). */
#define RESET_CAUSE_RSR_RMVF 0x00010000u     /* write 1: clear all flags */
#define RESET_CAUSE_RSR_CPURSTF 0x00020000u  /* CPU reset (set by every system reset) */
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
    RESET_CAUSE_LOW_POWER,
    RESET_CAUSE_SOFTWARE,
    RESET_CAUSE_POWER_ON,
    RESET_CAUSE_BROWN_OUT,
    RESET_CAUSE_PIN,
    RESET_CAUSE_DOMAIN_STANDBY,
    RESET_CAUSE_CPU_RESET
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
    } else if ((flags & RESET_CAUSE_RSR_LPWRRSTF) != 0u) {
        cause = RESET_CAUSE_LOW_POWER;
    } else if ((flags & RESET_CAUSE_RSR_SFTRSTF) != 0u) {
        cause = RESET_CAUSE_SOFTWARE;
    } else if ((flags & RESET_CAUSE_RSR_PORRSTF) != 0u) {
        cause = RESET_CAUSE_POWER_ON;
    } else if ((flags & RESET_CAUSE_RSR_BORRSTF) != 0u) {
        cause = RESET_CAUSE_BROWN_OUT;
    } else if ((flags & RESET_CAUSE_RSR_PINRSTF) != 0u) {
        cause = RESET_CAUSE_PIN;
    } else if ((flags & (RESET_CAUSE_RSR_D1RSTF | RESET_CAUSE_RSR_D2RSTF)) != 0u) {
        cause = RESET_CAUSE_DOMAIN_STANDBY;
    } else if ((flags & RESET_CAUSE_RSR_CPURSTF) != 0u) {
        cause = RESET_CAUSE_CPU_RESET;
    } else {
        /* no known flag: stays UNKNOWN */
    }
    return cause;
}

/*
 * Fault record kept across the fault-handler reset (cubemx/User/boot_checks.c, a .noinit
 * struct written stack-free by the naked fault entry): word 0 is the fault kind, word 3 is
 * kind XOR RESET_CAUSE_FAULT_MAGIC. A record whose check word does not match, or whose kind is
 * out of range, is not a fault (random RAM after power-up, a reset from Error_Handler).
 */
#define RESET_CAUSE_FAULT_MAGIC 0xFA17C0DEu

typedef enum {
    RESET_FAULT_NONE = 0,
    RESET_FAULT_HARD = 1,
    RESET_FAULT_MEMMANAGE = 2,
    RESET_FAULT_BUS = 3,
    RESET_FAULT_USAGE = 4
} reset_fault_t;

/*
 * False when the RAM behind the fault record cannot be trusted after this reset: a power-on or
 * brown-out reset (random contents), or a domain power-switch reset (D1/D2, the RAM was off).
 * Reading it then must be skipped (it may also hold uninitialised ECC RAM).
 */
static inline bool reset_fault_record_trusted(uint32_t rsr)
{
    const uint32_t untrusted = RESET_CAUSE_RSR_PORRSTF | RESET_CAUSE_RSR_BORRSTF |
                               RESET_CAUSE_RSR_D1RSTF | RESET_CAUSE_RSR_D2RSTF;

    return (rsr & untrusted) == 0u;
}

static inline reset_fault_t reset_fault_decode(uint32_t kind, uint32_t check)
{
    reset_fault_t fault = RESET_FAULT_NONE;

    if (((kind ^ RESET_CAUSE_FAULT_MAGIC) == check) && (kind >= 1u) && (kind <= 4u)) {
        fault = (reset_fault_t)kind;
    }
    return fault;
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_RESET_CAUSE_H */
