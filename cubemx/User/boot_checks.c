/* Boot-time checks and the fault safe state, see boot_checks.h (T77). */
#include "boot_checks.h"

#include "hal/can_timing.h"

/* The pure decode (src/hal/reset_cause.h) carries its own bit positions; they must match the CMSIS ones. */
_Static_assert(RESET_CAUSE_RSR_RMVF == RCC_RSR_RMVF_Msk, "RMVF");
_Static_assert(RESET_CAUSE_RSR_CPURSTF == RCC_RSR_CPURSTF_Msk, "CPURSTF");
_Static_assert(RESET_CAUSE_RSR_D1RSTF == RCC_RSR_D1RSTF_Msk, "D1RSTF");
_Static_assert(RESET_CAUSE_RSR_D2RSTF == RCC_RSR_D2RSTF_Msk, "D2RSTF");
_Static_assert(RESET_CAUSE_RSR_BORRSTF == RCC_RSR_BORRSTF_Msk, "BORRSTF");
_Static_assert(RESET_CAUSE_RSR_PINRSTF == RCC_RSR_PINRSTF_Msk, "PINRSTF");
_Static_assert(RESET_CAUSE_RSR_PORRSTF == RCC_RSR_PORRSTF_Msk, "PORRSTF");
_Static_assert(RESET_CAUSE_RSR_SFTRSTF == RCC_RSR_SFTRSTF_Msk, "SFTRSTF");
_Static_assert(RESET_CAUSE_RSR_IWDG1RSTF == RCC_RSR_IWDG1RSTF_Msk, "IWDG1RSTF");
_Static_assert(RESET_CAUSE_RSR_WWDG1RSTF == RCC_RSR_WWDG1RSTF_Msk, "WWDG1RSTF");
_Static_assert(RESET_CAUSE_RSR_LPWRRSTF == RCC_RSR_LPWRRSTF_Msk, "LPWRRSTF");
_Static_assert(CAN_TIMING_NBTP_NSJW_POS == FDCAN_NBTP_NSJW_Pos, "NSJW pos");
_Static_assert(CAN_TIMING_NBTP_NBRP_POS == FDCAN_NBTP_NBRP_Pos, "NBRP pos");
_Static_assert(CAN_TIMING_NBTP_NTSEG1_POS == FDCAN_NBTP_NTSEG1_Pos, "NTSEG1 pos");
_Static_assert(CAN_TIMING_NBTP_NTSEG2_POS == FDCAN_NBTP_NTSEG2_Pos, "NTSEG2 pos");
_Static_assert((CAN_TIMING_NBTP_NSJW_MASK << CAN_TIMING_NBTP_NSJW_POS) == FDCAN_NBTP_NSJW_Msk, "NSJW mask");
_Static_assert((CAN_TIMING_NBTP_NBRP_MASK << CAN_TIMING_NBTP_NBRP_POS) == FDCAN_NBTP_NBRP_Msk, "NBRP mask");
_Static_assert((CAN_TIMING_NBTP_NTSEG1_MASK << CAN_TIMING_NBTP_NTSEG1_POS) == FDCAN_NBTP_NTSEG1_Msk, "NTSEG1 mask");
_Static_assert((CAN_TIMING_NBTP_NTSEG2_MASK << CAN_TIMING_NBTP_NTSEG2_POS) == FDCAN_NBTP_NTSEG2_Msk, "NTSEG2 mask");

/* .noinit orphan section (NOBITS): placed after .bss by the linker, outside the startup clear.
 * The trailing '@' makes the assembler read the rest of the section directive as a comment. */
boot_fault_record_t boot_fault_record __attribute__((section(".noinit,\"aw\",%nobits @")));

static uint32_t s_reset_rsr;
static reset_fault_t s_fault_kind = RESET_FAULT_NONE;
static uint32_t s_fault_cfsr;
static uint32_t s_fault_hfsr;
static reset_cause_t s_reset_cause = RESET_CAUSE_UNKNOWN;

void boot_fault_safe_state(void)
{
    boot_safe_state();
}

void boot_reset_cause_capture(void)
{
    s_reset_rsr = RCC->RSR;
    s_reset_cause = reset_cause_decode(s_reset_rsr);
    RCC->RSR |= RCC_RSR_RMVF;
    RCC->RSR &= ~RCC_RSR_RMVF; /* RMVF is software set/clear on the H7, not self-clearing */

    /* After a power-on reset the RAM is random (and uninitialised ECC RAM must not be read), so
     * the fault record only counts after any other reset. It is written afresh every boot. */
    if ((s_reset_rsr & RCC_RSR_PORRSTF) == 0u) {
        s_fault_kind = reset_fault_decode(boot_fault_record.kind, boot_fault_record.check);
        s_fault_cfsr = boot_fault_record.cfsr;
        s_fault_hfsr = boot_fault_record.hfsr;
    }
    boot_fault_record.kind = 0u;
    boot_fault_record.cfsr = 0u;
    boot_fault_record.hfsr = 0u;
    boot_fault_record.check = 0u;
}

reset_fault_t boot_fault_kind(void)
{
    return s_fault_kind;
}

uint32_t boot_fault_cfsr(void)
{
    return s_fault_cfsr;
}

uint32_t boot_fault_hfsr(void)
{
    return s_fault_hfsr;
}

uint32_t boot_reset_cause_raw(void)
{
    return s_reset_rsr;
}

reset_cause_t boot_reset_cause(void)
{
    return s_reset_cause;
}

bool boot_fdcan_timing_ok(const FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps)
{
    const uint32_t kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    return (hfdcan != NULL) && (hfdcan->Instance != NULL) &&
           can_timing_nbtp_ok(kernel_hz, hfdcan->Instance->NBTP, expected_bps);
}

HAL_StatusTypeDef boot_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps)
{
    if (!boot_fdcan_timing_ok(hfdcan, expected_bps)) {
        boot_safe_state();
    }
    return HAL_FDCAN_Start(hfdcan);
}
