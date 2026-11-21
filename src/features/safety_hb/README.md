# features/safety_hb

rt-core's monitor of the safety MCU's heartbeat 0x082 `HeartbeatSafety` (D-118 item 6, D-005, D-026; defs v0.30.0). It is the only place where rt-core receives a frame of the safety partition, and the frame is **diagnostic input only**.

## Responsibility

- Registers a receiver for 0x082 on the platform port (`can_if_register_rx()`, boot, before `comms_apply_filters()`), so the acceptance filter lets it through. The ID, length, cycle, DataID and timeout come from gen/ (`PLATFORM_HEARTBEAT_SAFETY_*`, `platform_heartbeat_safety_e2e_check()` and `_check_timeout()`).
- Reports the DTC `SAFETY_HEARTBEAT_LOST` (U0167-00, `diagnostic_only`) every comms pass through `diag_dtc_report()` (level-triggered).
- Keeps link-health counters (`safety_hb_link()`): frames, resyncs, sequence_lost, wrong_sequence, repeated, wrong_crc, wrong_length, other_errors, lost_events. They are read by tests and the host summary; there is no DID for them (none is invented).

## Semantics

- **The DTC fails only on silence**: the generated `check_timeout` returns NO_NEW_DATA when no frame advanced the E2E state for more than `PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS` (300 ms, three 100 ms cycles). One lost frame does not set it. Before the first valid frame the same timeout counted from `safety_hb_open()` is the start-up grace (the safety MCU boots on its own schedule); after it the DTC reads failed.
- **Single E2E errors are link errors, not the DTC**: WRONG_SEQUENCE, REPEATED, WRONG_CRC, WRONG_LENGTH (and BAD_ARGUMENT) only increment a counter. A sender that stays wrong never refreshes the timeout, so it ends up as silence, which is the generated semantics. WRONG_LENGTH (a DLC other than `PLATFORM_HEARTBEAT_SAFETY_E2E_LENGTH`) leaves the E2E state untouched and is invalid like every non-OK status.
- **Never decoded, never fed back**: the payload is not read. Nothing from the frame reaches 0x020, the EKF, rt-core's own 0x081 (the DTC's generated `DIAGNOSTIC_ONLY` flag keeps it out of `diag_fault_active()` and `diag_fault_onsets()`), or any frame sent to the safety MCU (it is not a safety input, invariant 3). CI greps restrict the 0x082 ID, the `platform_heartbeat_safety_*` calls and `safety_hb_step()` to this module and the comms pass.

## Wiring

`safety_hb_open(&m)` at boot (host: `app/host/main.c`; H7: with the other opens) and `comms_pass(..., &m)`: `safety_hb_step()` runs right after `vehicle_republish_step()`, from the frames `can_if_dispatch(PLATFORM)` just delivered; the comms task is the only caller, so no lock. `m == NULL` or closed means no monitor and no DTC report.

Files: `safety_hb_core.[ch]` (pure: state, E2E status sort, verdict; in `vehicle_rtcore_logic`), `safety_hb.[ch]` (glue: can_if + diag + timebase; in `vehicle_rtcore_fw`). Static storage only, about 56 B of state (`safety_hb_t`).

## Tests

`test_safety_hb_core` (verdict timing incl. wrap, grace, link counters, WRONG_LENGTH, saturation), `test_safety_hb` (bus, can_if and diag: DTC after three silent cycles and not after one lost frame, 0x081 NODE_MODE/ERROR_COUNT unchanged, wrong DLC), `test_platform_tx` (real `comms_pass()`: 0x081 and 0x020 byte-identical with the monitor alive, silent or absent). The host SIL plays the safety MCU with `app/host/sim_safety`.
