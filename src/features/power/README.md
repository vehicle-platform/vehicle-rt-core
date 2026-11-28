# features/power

Key-off power-down of rt-core (D-138 option B). `keyoff_core` is the pure state machine; the glue is a later card (T13) and wires the KL15 sense, the HOLD pin, the platform frames `ShutdownRequest` (0x121, sent) and `ShutdownAck` (0x122, received) and the log close.

- Key off (KL15 valid and low) or a debounced low battery (< 12.2 V for 10 s) starts a sequence: request out, logs close, HOLD stays asserted.
- HOLD is released when the Pi 5 acknowledges `READY_TO_CUT` for the current request id, or at `KEYOFF_TIMEOUT_MS` (25 s, PROVISIONAL until measured in T73), whichever comes first. A hung Pi never holds the supply past the timeout.
- Only a real key cycle (KL15 read off after the sequence started, then a debounced key-on) cancels a sequence before the Pi acknowledged `HALTING`, or reopens DONE. A low-voltage sequence is also cleared before `HALTING` when the engine runs or the battery recovers to `KEYOFF_LOW_VOLTAGE_RECOVER_MV` (12.6 V, PROVISIONAL hysteresis); its DONE is latched until a key cycle or an engine start. After `HALTING` a sequence runs to completion; the halted Pi stays down until the next power cycle (hardware follow-up).
- An invalid KL15 sense never starts or cancels a sequence (in DONE it releases HOLD; the real KL15 still holds the supply through the diode-OR), an invalid battery sample never starts a shutdown, a zeroed state keeps HOLD asserted.
- The safety MCU does not depend on HOLD or on these frames (invariant 3).

Safety review (D-138, invariant 8 scope): KL15-off is debounced (500 ms, PROVISIONAL) and vetoed while the engine runs; the low-voltage trip needs KL15 on and the engine known to be stopped (a weak charge at idle must never halt the Pi mid-ride; D-138 item 4 amendment, user approved 2026-11-28); an invalid KL15 sense never starts or cancels a sequence; HOLD is also asserted while KL15 is valid and on; round 2 (MAJOR-1): the KL15 level that was on all along no longer cancels a low-voltage sequence or reopens DONE (key-cycle edge only).

**Requirements for the glue/hardware card (T13 + Board A, from the safety reviews):**
1. The HOLD diode-OR is a stated hardware safety requirement and must never back-feed the vehicle's KL15 line (HOLD would otherwise power the ECU/ignition); measured on the HIL bench.
2. HOLD needs a maximum on-time bound: in hardware (a timer on the switch) or in software (KL15 invalid longer than N s with the engine not running → a bounded shutdown), so neither an rt-core reset loop nor a stuck KL15 sense or `engine_running` can hold the supply until the battery is flat.
3. `engine_running` goes false once the engine-speed DID turns STALE (at key-off the ECU loses power; a held VALID sample would veto the key-off forever), with a glue test.
4. `ack_fresh` uses the generated `decode_checked`, the DLC check and a 3-cycle timeout (0x122 has no E2E; a spoofed READY only risks the filesystem, QM).
5. A Pi restart path (Pi 5 power-button/RUN pin) for a halted Pi with KL15 on again.
