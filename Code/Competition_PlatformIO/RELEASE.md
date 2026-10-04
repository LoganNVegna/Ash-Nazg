# Competition 1.4 acceptance status

**Hardware test candidate.** Implements the 2026-10-04 post-competition requirements. Prior 1.2 operator sign-off does not apply. No robot upload or physical operation has been performed by the assistant.

Earlier tests established command bounds and low-speed smoothness, not useful fighting-speed translation or impact resilience. Acceptance now reflects those failures. Original V5 and release tags are preserved; the existing remote `competition-v1.3` tag points at the 1.2 commit and is not modified.

## Implemented

- Sensor clipping/staleness/configuration/calibration errors, main-loop delays, NVS failures, failed optional workers and recoverable output errors do not disarm.
- Five continuous seconds without live controller commands disarms all modes. Short gaps hold commands; explicit RF-loss held frames do not refresh the timer. Fresh CH5 HIGH-to-LOW recovers without reset or sensor-health prerequisite.
- Manual CH5 and intentional stopped OTA remain. Aborted OTA leaves maintenance/retry/rearming available. Wi-Fi precedes peripheral workers and has allocation-failure fallback.
- Stronger translation profiles retain corrected heading; independent phase/time lead, fractional trim, submitted-command LEDs, compact diagnostics and optional captures are included.

## Software verification

Target and host/downloader checks are detailed in VALIDATION.md. Fault injection uses actual runtime steps, including sensor absent at boot and one-channel faults. Legal commands, CRC and original DShot encoding/startup packet content remain. Fault-stop and universal no-reversal assertions no longer define production policy; archived 1.2 behavior is tested separately.

## Physical observations still required

1. OTA installation, boot/manual CH5/repeat arming and used tank/unstick/reverse modes work as documented.
2. A second OTA from 1.4 succeeds; an interrupted upload leaves another upload and deliberate CH5 recovery possible. Preserve the older working project for OTA rollback.
3. Actual transmitter loss holds drive through the grace period, then disarms around five seconds after live-command evidence ceases. Reconnect alone cannot restart motion; CH5 HIGH-to-LOW recovers. Receiver reporting must be checked physically.
4. With two healthy motors, TESTING.md comparisons produce useful heading-aligned translation at intended fighting spin, not only smooth low-speed movement.
5. Representative impacts/contact do not cause sensor-triggered disarm/zero commands or a reset-only lock. Estimation confidence may degrade and recover.
6. Match-duration operation at intended spin/translation remains controllable; CH5 returns Wi-Fi. A compact summary is sufficient unless unexplained behavior needs deeper evidence.

These need the operator and physical robot. Successful compilation, simulation or CI does not mark them complete.
