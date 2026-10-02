# Software validation for competition 1.2

ESP32-S3 build using Espressif 32 6.12.0/Arduino 2.0.17, QT Py no-PSRAM board and min_spiffs layout. The separate deployment project's checked build used 199,932 bytes RAM and 832,753 bytes application flash, with an 833,120-byte OTA binary. The repository build is verified independently; exact binary hashes vary with build metadata.

The production drivers remain byte-identical to the supplied V5 reference. Tests compile the actual startup include and driver (normalizing only a GNU macro for the host compiler) and validate the 2,500 alternating stop pairs, direction/3D settings, telemetry/checksum, packet timing and signed encoder vectors.

Portable production-code suites verify sensor scale, 20 ms filtering, phase continuity/rollover, calibration taking 2.8 seconds, timeout preventing late calibration, transient/persistent register verification, fractional LEDs/trim bounds/storage checksums, receiver CRC/frame freshness, link statistics, stop/rearm guards and OTA/export/flash inhibition.2,181,600 mixer cases cover both spin directions, throttle 11-100, all 101 CH2 positions and 3-degree phase steps. Full cosine obeys paired command/headroom/no-wheel-reversal bounds and opposite-phase symmetry. The ideal directional projection doubles the prior half-wave result; physical force/speed is not measured by this test.

Capture tests preserve driving plus a one-second tail through 20 seconds idle, resume logging on activity and retain the final stop row. Export tests cover formatting, byte slices, partial writes/EAGAIN, stalls, disconnects and timer rollover. Windows mock-server tests exercise the actual downloader with 1.0/1.1/1.2 labels, reject incompatible formats/corrupt pages and preserve previously downloaded evidence.

The firmware image checker verifies target/build identity, size, original partition-table equivalence and espota/authentication. Firmware/reference libraries and tests retain their original licenses. The preserved source reference is for driver integrity and historical comparison.

These tests do not simulate actual UART/SPI/RMT scheduling, ESC acknowledgements, motor torque, traction, RF reporting or hardware/power failures. See RELEASE.md for observed hardware behavior and remaining sign-off. No assistant has uploaded or operated the robot.
