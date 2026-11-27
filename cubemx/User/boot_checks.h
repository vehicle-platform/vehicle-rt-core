/*
 * Boot-time checks and the fault safe state of the CubeMX image (T77). Not generated: this file
 * and boot_checks.c are ours and survive a CubeMX regeneration; the generated files only call
 * into them from USER CODE blocks. Red-zone, invariant-8 code (reset handling, fault paths).
 */
#ifndef BOOT_CHECKS_H
#define BOOT_CHECKS_H

#include "main.h"

#include "hal/reset_cause.h"
#include "vehicle_cl250.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Safe state, then reset. Stack-free: IRQs masked, the SPI1 slave chip selects driven high
 * through GPIOD BSRR (the GPIOD clock is forced on so the write cannot fault), then a system
 * reset request. It is static inline with always_inline on purpose: a call would push the
 * return address and a frame onto a stack that may be exhausted (a stack-overflow fault), and a
 * second fault there is a lockup. Inlined, it is a handful of register writes. Before
 * MX_GPIO_Init the pins are still analog, so the BSRR write only preloads ODR and the lines
 * rely on the external pull-ups (h743-pin-plan hardware item). It never returns; the wait
 * after the reset request is bounded by the reset itself (IWDG1 backs it up once started).
 */
static inline __attribute__((always_inline, noreturn)) void boot_safe_state(void)
{
    __disable_irq();
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    (void)RCC->AHB4ENR;
    GPIOD->BSRR = (uint32_t)(CS_EGT_Pin | CS_ADXL_ARM_Pin | CS_ADXL_ENG_Pin);
    __DSB();
    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk;
    __DSB();
    for (;;) {
        __NOP();
    }
}

/*
 * Fault record, kept across the reset in a .noinit struct (outside .bss, so the startup code
 * does not clear it). Word 0 = fault kind (reset_fault_t value 1..4), 1 = CFSR, 2 = HFSR,
 * 3 = kind XOR RESET_CAUSE_FAULT_MAGIC. Written by BOOT_FAULT_ENTRY() with registers only.
 */
typedef struct {
    uint32_t kind;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t check;
} boot_fault_record_t;

extern boot_fault_record_t boot_fault_record;

/*
 * Entry of HardFault, MemManage, BusFault and UsageFault: a naked handler (no prologue, so no
 * push) that loads MSP with the top of the stack region (_estack, the linker symbol), saves the
 * fault kind (tag 1..4), CFSR and HFSR into boot_fault_record with registers only, and branches
 * to boot_fault_safe_state(). The handler runs on MSP; if the fault was MSP exhaustion (an ISR
 * chain that overran the stack) the old MSP is not trusted, and the fault is never resumed, so
 * restarting the stack at the top is safe. Use only as the whole body of a function declared
 * __attribute__((naked)). The next boot reads the record with boot_fault_kind().
 */
#define BOOT_FAULT_ENTRY(tag)                                  \
    __asm volatile("ldr r0, =_estack\n"                        \
                   "msr msp, r0\n"                             \
                   "isb\n"                                     \
                   "ldr r1, =boot_fault_record\n"              \
                   "ldr r2, =0xE000ED28\n" /* SCB->CFSR */     \
                   "ldr r3, [r2]\n"                            \
                   "str r3, [r1, #4]\n"                        \
                   "ldr r3, [r2, #4]\n" /* SCB->HFSR */        \
                   "str r3, [r1, #8]\n"                        \
                   "movs r3, #" #tag "\n"                      \
                   "str r3, [r1, #0]\n"                        \
                   "ldr r2, =0xFA17C0DE\n"                     \
                   "eors r3, r3, r2\n"                         \
                   "str r3, [r1, #12]\n"                       \
                   "dsb\n"                                     \
                   "b boot_fault_safe_state\n")

/* Target of BOOT_FAULT_ENTRY(); a function (not inline) because the asm branches to it by name. */
void boot_fault_safe_state(void) __attribute__((noreturn, used));

/*
 * Reset cause. boot_reset_cause_capture() reads RCC->RSR, stores it, then clears the flags
 * with RMVF; call it first in main(), before any peripheral init (nothing earlier reads or
 * clears RSR: Reset_Handler and SystemInit leave it alone). The raw value and the decoded
 * cause stay readable afterwards. No CAN report yet: defs has no reset-cause signal
 * (defs follow-up).
 */
void boot_reset_cause_capture(void);
uint32_t boot_reset_cause_raw(void);
reset_cause_t boot_reset_cause(void);

/*
 * Whether the reset that led to this boot came from a fault handler (not Error_Handler, not a
 * watchdog): the kind, CFSR and HFSR saved by BOOT_FAULT_ENTRY(). Valid after
 * boot_reset_cause_capture(); RESET_FAULT_NONE after a power-on reset, whose RAM is random. The
 * record is cleared at capture, so it reports the previous reset only.
 */
reset_fault_t boot_fault_kind(void);
uint32_t boot_fault_cfsr(void);
uint32_t boot_fault_hfsr(void);

/*
 * FDCAN nominal bit-timing check before HAL_FDCAN_Start: the kernel clock is read from the RCC
 * at run time (HAL_RCCEx_GetPeriphCLKFreq) and the NBTP register is read back; both go to
 * can_timing_nbtp_ok() (hal/can_timing.h) with the expected bit rate. False for a wrong
 * clock tree, a wrong or unprogrammed NBTP, or an unreadable clock.
 *
 * The vehicle bus uses the generated VEHICLE_CL250_BITRATE. The platform bus has no generated
 * bit-rate constant (defs follow-up); both buses are 500 kbit/s (D-009, src/hal/README.md), so
 * FDCAN2 is checked against the same value until defs has its own.
 */
#define BOOT_FDCAN1_EXPECTED_BPS VEHICLE_CL250_BITRATE
#define BOOT_FDCAN2_EXPECTED_BPS VEHICLE_CL250_BITRATE

bool boot_fdcan_timing_ok(const FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps);

/*
 * The only way the image starts an FDCAN controller: on a failed timing check it goes to the
 * safe state (reset) and the controller is not started; otherwise it returns HAL_FDCAN_Start().
 */
HAL_StatusTypeDef boot_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_CHECKS_H */
