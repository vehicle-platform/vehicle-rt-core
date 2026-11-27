# Renode boot test of the CubeMX image (T67)

Boots the CubeMX-generated image of `cubemx/` (STM32H743ZI, FreeRTOS CMSIS-V2) in the [Renode](https://renode.io) emulator and checks that it comes up cleanly. It runs in CI (job `renode-boot`) before any board is on the desk. It does **not** replace hardware bring-up: Renode models registers, not electrical behaviour, clock accuracy or timing margins.

Pinned versions: Renode **1.17.0** (released 2026-09-07; portable Linux build, SHA-256 `4ba7c68b59e2447f188ef4b4b112fcccd2802582c460beedceb63b84e5605a5f`), Robot Framework 6.1.

## What is covered (`boot.robot`)

| Test | Asserts |
|---|---|
| Core Reaches Main And Completes Clock Init | `Reset_Handler`, `main` and the first call after `SystemClock_Config()` are reached, so HSE bypass + PLL1 + VOS and every HAL status check passed; `RCC_CFGR.SWS` shows PLL1 as the system clock |
| FDCAN Nominal Bit Timing Is Programmed | `FDCAN_NBTP` is written with NBRP 9, NTSEG1 12, NTSEG2 1, NSJW 1 on FDCAN1 and FDCAN2 (HAL prescaler 10, seg1 13, seg2 2, SJW 2) |
| Kernel Starts And Default Task Refreshes The Watchdog | `osKernelStart` and `StartDefaultTask` are reached; five consecutive `HAL_IWDG_Refresh` calls, each within one 100 ms task period (plus margin) |
| No Reset And No Fault Handler Within Two Seconds | 2 s of emulated time with no entry into `Error_Handler`, `HardFault_Handler`, `NMI_Handler`, `MemManage_Handler`, `BusFault_Handler`, `UsageFault_Handler`, `Default_Handler` or `vApplicationStackOverflowHook`, and no machine reset (IWDG1 expiry or system reset); the refresh is still running at the end |

The probes are Renode CPU hooks on those symbols (`rt-core.resc`); the firmware is not modified.

## Real versus stubbed

Used as shipped by Renode (`platforms/cpus/stm32h743.repl`): Cortex-M7 + NVIC + SysTick, the H7 RCC model (HSE/PLL ready flags, `SWS`), PWR with its voltage-scaling-ready stub (VOSRDY/ACTVOSRDY), IWDG1 (it really resets the machine when not refreshed), FDCAN1/2 (Bosch M_CAN), GPIO, USART/UART, I2C, SPI, TIM, DMA, ADC.

Added or worked around in `rt-core.repl` / `rt-core.resc`:

- `rcc.hseFrequency = 8 MHz` (ST-LINK MCO). The model reports `HSERDY`/`PLLRDY` itself.
- A read-only DBGMCU stub (`IDCODE`), because `HAL_GetREVID()` reads it and the stock platform leaves it unmapped.
- **NBTP is decoded at write time** by a Renode peripheral-write hook, because the stock M_CAN model does not store the register (a read returns the reset value). So the test proves the firmware programs the right value, not that a bus with that timing works. FDCAN is not started by the image (`HAL_FDCAN_Start` is not called), so no frames are exchanged.
- The image runs from flash at `0x08000000`: `VectorTableOffset` is set and re-applied by a `reset` macro.

Not modelled and therefore **not** tested: real clock speeds and timing margins (the emulated core runs the instruction stream, not the 480 MHz timing), HSE bypass electrical behaviour, CSS (clock security system) reactions, MPU/cache effects, real FDCAN bus traffic, power modes, flash wait states.

## Run locally

Install the pinned Renode release (macOS: the arm64 portable `.app` from the same GitHub release; Linux: the portable tarball). Then:

```sh
# 1. Build the image (arm-none-eabi-gcc and ninja on PATH)
(cd cubemx && cmake --preset Debug && cmake --build --preset Debug)

# 2. Python deps in a venv (Python 3.11-3.13), then run from a scratch directory (Robot writes its logs to the cwd)
python3 -m venv /tmp/rvenv && /tmp/rvenv/bin/pip install -r tests/renode/requirements.txt
mkdir -p /tmp/renode-out && cd /tmp/renode-out
PATH=/tmp/rvenv/bin:$PATH /path/to/renode/renode-test "$OLDPWD/tests/renode/boot.robot"
```

Interactive: `renode tests/renode/rt-core.resc`, then `start` (or `emulation RunFor "2"`). Set `$elf` first to load another ELF.

CI uploads `log.html`, `report.html`, `robot_output.xml` and the Renode logs as the `renode-boot-rt-core` artifact.

## Updating Renode

Bump `RENODE_VERSION` and `RENODE_SHA256` in `.github/workflows/ci.yml` (use a release that is at least two weeks old; the checksum is the `digest` GitHub shows for the tarball) and the versions above, then re-run the job. Probe messages and register decoding depend on the Renode model; re-check them on a bump.
