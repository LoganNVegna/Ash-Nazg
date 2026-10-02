# Competition 1.2 release checklist

## Scope frozen

Keep the operator-approved 1.2 control behavior: full-cosine translation, current strength/headroom/deadband, fresh-sample estimator, continuous heading, fractional trim LEDs, receiver-loss inhibition and Wi-Fi OTA. No additional strength boost, braking/reversal change, peripheral rewrite or radio remapping is part of this release.

## Software verification

- Target build, expected identity, binary size and byte-identical original OTA partition layout pass.
- Driver/reference byte equivalence, actual startup/encoder, 2,181,600 mixer bounds/no-reversal cases, sensor calibration/retries, phase/trim/settings, arm/stop/rearm, log retention and paged CSV regression checks pass.
- Repository CI rebuilds and runs the tests. Compiler/package versions and vendored libraries are recorded; build binaries/captures/generated files are excluded from Git.
- Original V5 remains unchanged. For rollback, use a previously saved working competition source revision and its OTA application upload. Do not upload a bootloader or partition table through this workflow.

## Observed on the robot

- OTA uploads from the previous competition firmware succeeded through Wi-Fi; 1.1 reported valid app0/app1 slots and maintenance access.
- 1.1 sensor startup passed, calibration completed in approximately 0.96 s, and transient CTRL4 readback errors recovered.
- Operator reports 1.2 heading and translation are smooth/controllable, correct-direction and acceptable in speed.
- Earlier captured 1.1 session ended normally on CH5 with no sensor/fatal/DShot API error, and trim 1.008 persisted.

## Physical sign-off still required unless already checked

No routine capture/download is needed for these observations. Use an enclosed test area.

1. **Boot inhibition:** with CH5 LOW or nonzero throttle at startup, no drive begins without the healthy neutral CH5 HIGH-to-LOW arm sequence. Keep the controller stationary during calibration.
2. **Kill and repeat arm:** at the lowest familiar speed, CH5 HIGH stops commanded drive; neutral HIGH-to-LOW rearms deliberately. Check tank/unstick and reverse at low output if those modes will be used.
3. **Radio loss:** turn off the transmitter at low spin; drive commands stop and Wi-Fi returns. Before restoring the transmitter set throttle zero/controls centered/CH5 HIGH. Restoring it must not restart drive. Deliberate neutral rearming should work.
4. **OTA from 1.2:** while stopped and inhibited, a normal application OTA upload must succeed from the current 1.2 image, reboot into maintenance, and remain available after reconnecting. Existing 1.2 can be uploaded again; no motor test is necessary for this check.
5. **Operating envelope:** briefly check the highest spin rate and translation strength intended for competition, within the sensor range. Confirm controllability, heading, CH5 stop and no abnormal heating/reset/fault. Low-speed success does not validate the high-speed behavior that prompted the original review.
6. **Power-cycle persistence:** stopped trim saves successfully; after an intentional restart it is retained and arming is again inhibited at boot.

Mark this revision competition-ready only after these physical behaviors are confirmed. This document records a candidate and does not imply unperformed hardware validation. If a check fails, stop and inspect the summary; obtain a capture only when it will help diagnose the failure.
