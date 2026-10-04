# Software verification — competition 1.4

Target: QT Py ESP32-S3 no PSRAM, Espressif32 6.12.0 / Arduino 2.0.17, original min_spiffs two-slot layout, authenticated application-only espota upload. The image checker compares partitions byte-for-byte with the working OTA reference and checks image identity/size/target. Exact firmware hashes and build sizes are supplied with the deployment package.

Local verification on 2026-10-04 passed the target build, all five host suites (including both runtime scenarios), and the Windows PowerShell download tests. Static RAM: 86,764 bytes / 327,680; flash: 844,465 / 1,966,080; application image: 844,832 bytes. These are software results for the current source, not physical acceptance.

## Executable checks

- Sensor scaling, original 20 ms EMA, calibration timeout/rollover, bounded register retries, continuous phase, fractional trim and saved-record integrity.
- Historical 1.2 mixer bounds separately from new production policy; 8,726,400 new-profile cases across spin directions, stick/throttle values and phase. Legal ±999 outputs remain; intentional reversal is permitted in stronger profiles, bounded-profile headroom is retained.
- Five-second loss boundary, timestamp wraparound, boot neutral sequence, explicit CH5 recovery without neutral/sensor prerequisites, and maintained drive while optional save/export/readiness flags change.
- Prediction through missing/clipped measurements, smooth measurement return, isolated shock/transverse/sign rejection, confirmed genuine changes, phase offset/lead, spin ramp and event/timing counters.
- Actual production channel lifecycle and driver: 2,500 stops and ten direction/ten 3D packets per ESC, signed encoder/checksum/pulse timings, explicit zero terminator, one busy response, sibling continuation and independent recovery.
- Actual production UART/SPI/output/maintenance routines compiled against mocks: near-rail input, missing/bad configuration and automatic return, absent sensor at boot, cooperative operation with all task allocations failing, retries when allocation resumes, explicit RF-loss held frames, CH5 recovery, NVS failure, OTA error callbacks and subsequent maintenance/rearm.
- Existing CSV formatting/ranges/partial writes/stalls/disconnects/rollover and actual PowerShell downloader against paged HTTP responses. Current and legacy build labels, corrupted pages and incompatible builds are checked.

The runtime/adapter/settings/OTA functions are taken directly from production source, not reimplemented in tests. Host compilation uses C++17/MSVC or GCC; MCU compilation uses the pinned toolchain. The driver source is intentionally no longer byte-identical to V5: initialization is explicit and a nonblocking one-packet startup method supports recovery. Encoding and initial packet contents remain verified against original vectors. V5 reference files are unchanged.

## Limits

Mocks test control paths, not real UART timing, SPI wiring, interrupt load, watchdog/power failure, ESC acknowledgment, motor torque, braking, wheel slip, collision physics or useful high-speed translation. Phase/sign conventions require the physical comparisons in TESTING.md. The fallback rate model is unvalidated until measured. Installed receiver loss-reporting and OTA operation from the new image remain physical checks. No build or CI pass is a competition sign-off.
