# features/power

Key-off power-down of rt-core (D-138 option B). `keyoff_core` is the pure state machine; the glue is a later card (T13) and wires the KL15 sense, the HOLD pin, the platform frames `ShutdownRequest` (0x121, sent) and `ShutdownAck` (0x122, received) and the log close.

- Key off (KL15 valid and low) or a debounced low battery (< 12.2 V for 10 s) starts a sequence: request out, logs close, HOLD stays asserted.
- HOLD is released when the Pi 5 acknowledges `READY_TO_CUT` for the current request id, or at `KEYOFF_TIMEOUT_MS` (25 s, PROVISIONAL until measured in T73), whichever comes first. A hung Pi never holds the supply past the timeout.
- Key back on before the Pi acknowledged `HALTING` cancels the sequence. After `HALTING` the sequence runs to completion; if KL15 is on again the halted Pi stays down until the next power cycle (hardware follow-up).
- An invalid KL15 sense reads as key-on, an invalid battery sample never starts a shutdown, a zeroed state keeps HOLD asserted.
- The safety MCU does not depend on HOLD or on these frames (invariant 3).

Safety review (D-138, invariant 8 scope): KL15-off is debounced (500 ms, PROVISIONAL) and vetoed while the engine runs; the low-voltage trip needs KL15 on and the engine known to be stopped (a weak charge at idle must never halt the Pi mid-ride; this refines D-138 item 4 "resting", user may revise); an invalid KL15 sense never starts or cancels a sequence; HOLD is also asserted while KL15 is valid and on; a debounced key-on returns DONE to RUNNING. Open for the glue/hardware card: the diode-OR must be a stated hardware safety requirement, and HOLD needs a hardware time cap so an rt-core reset loop cannot hold the supply indefinitely.
