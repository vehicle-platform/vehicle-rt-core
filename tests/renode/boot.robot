*** Settings ***
Documentation     Boot test of the CubeMX-generated rt-core image (STM32H743ZI, FreeRTOS CMSIS-V2) in Renode.
...
...               The probes are installed by rt-core.resc and log "PROBE <name>" lines. Nothing in the
...               firmware is modified or instrumented. See tests/renode/README.md for what is real and
...               what is stubbed.
Suite Setup       Setup
Suite Teardown    Teardown
Test Setup        Reset Emulation
Test Teardown     Test Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${RESC}           ${CURDIR}/rt-core.resc
${ELF}            ${CURDIR}/../../cubemx/build/Debug/rt-core.elf
# RCC_CFGR (0x58024410) bits [5:3] = SWS, 0b011 = PLL1 selected as system clock.
${RCC_CFGR}       0x58024410
# The default task refreshes the 0.5 s IWDG1 every 100 ms (osDelay(100)).
${RUN_SECONDS}    2

*** Keywords ***
Load Image
    [Documentation]    Load the ELF with the probes and attach the log tester (emulation stays paused;
    ...                time only advances in the Run keywords below).
    Execute Command    $elf=@${ELF}
    Execute Command    include @${RESC}
    Create Log Tester  0

Run For
    [Arguments]        ${seconds}
    Execute Command    emulation RunFor "${seconds}"

Probe Should Appear
    [Arguments]        ${probe}
    Wait For Log Entry    PROBE ${probe}    timeout=0

Watchdog Refresh Should Happen Within
    [Documentation]    One-shot probe on HAL_IWDG_Refresh: armed now, it must fire within the given
    ...                emulated time. The tag keeps every probe line unique.
    [Arguments]        ${tag}    ${seconds}
    Execute Command    sysbus.cpu AddHook `sysbus GetSymbolAddress "HAL_IWDG_Refresh"` "self.WarningLog('PROBE IWDG_REFRESH ${tag}'); self.RemoveHooksAt(self.PC)"
    Run For            ${seconds}
    Probe Should Appear    IWDG_REFRESH ${tag}

*** Test Cases ***
Core Reaches Main And Completes Clock Init
    [Documentation]    HSE bypass + PLL1 + VOS (PWR stub of the stock platform) complete without Error_Handler.
    Load Image
    Run For            0.05
    Probe Should Appear    Reset_Handler
    Probe Should Appear    main
    # MX_GPIO_Init is the first call after SystemClock_Config(); reaching it means every
    # HAL_RCC_OscConfig / HAL_RCC_ClockConfig status check passed.
    Probe Should Appear    clock_init_done
    ${cfgr}=           Execute Command    sysbus ReadDoubleWord ${RCC_CFGR}
    ${sws}=            Evaluate    (int($cfgr.strip(), 16) >> 3) & 0x7
    Should Be Equal As Integers    ${sws}    3    SYSCLK is not PLL1 (RCC_CFGR=${cfgr})

FDCAN Nominal Bit Timing Is Programmed
    [Documentation]    Prescaler 10, TSEG1 13, TSEG2 2, SJW 2 in HAL terms = NBRP 9, NTSEG1 12, NTSEG2 1, NSJW 1.
    Load Image
    Run For            0.05
    Probe Should Appear    NBTP fdcan1 nbrp=9 ntseg1=12 ntseg2=1 nsjw=1
    Probe Should Appear    NBTP fdcan2 nbrp=9 ntseg1=12 ntseg2=1 nsjw=1

Kernel Starts And Default Task Refreshes The Watchdog
    Load Image
    Run For            0.2
    Probe Should Appear    osKernelStart
    Probe Should Appear    StartDefaultTask
    # Five consecutive refreshes, each within one task period plus margin.
    FOR    ${i}    IN RANGE    5
        Watchdog Refresh Should Happen Within    ${i}    0.15
    END

No Reset And No Fault Handler Within Two Seconds
    [Documentation]    Any entry into a fault/error handler or a machine reset (IWDG expiry, system reset)
    ...                fails the test; the watchdog refresh must still be running at the end.
    Load Image
    Run For            ${RUN_SECONDS}
    Should Not Be In Log    PROBE FAULT
    Watchdog Refresh Should Happen Within    end    0.15
