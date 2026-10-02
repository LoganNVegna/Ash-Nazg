# AshNazg competition 1.2

Competition firmware for the existing Adafruit QT Py ESP32-S3 (8 MB flash, no PSRAM), H3LIS331DL accelerometer, Repeat AM32 drive ESCs and existing radio/wiring. This is the 1.2 behavior tested by the operator: stable heading, smooth correct-direction translation and acceptable speed. It does not add the subsequently discussed translation boosts.

`../V5_PlatformIO` is preserved as historical firmware. This project is the operator-approved competition release for the tested setup and conditions. Software checks pass; operator sign-off and the reusable physical checklist are recorded in [RELEASE.md](RELEASE.md). Routine CSV downloads are optional.

## Wi-Fi upload

USB is inaccessible. All supported uploads use the existing OTA path. Keep CH5 HIGH, throttle zero and the robot stationary. Connect to **AshNazg Diagnostics**, password **AshNazgDiag**. From this project directory:

```bat
pio run -e upload_ota -t upload
```

OTA password is `admin`; robot IP is `192.168.4.1`, device UDP port 3232, computer TCP port 3233. Keep the existing Windows static Wi-Fi address `192.168.4.2/24` and upload firewall rule. Ethernet can remain connected. Reconnect after SUCCESS and wait 10 seconds stationary for calibration. Check:

```bat
curl.exe --max-time 10 http://192.168.4.1/summary.txt
```

Expect `AshNazg competition 1.2`, `hardware_ready=1`, `fatal=0`, `sensor_failed=0`, `sensor_saturated=0`, `OTA_slots_valid=1`, `ota_status=Ready`, positive link quality and `link_statistics_seen=1`. Close the robot webpage before arming.

Wi-Fi starts before peripheral initialization and returns after a stop. Motor output is inhibited during maintenance/export/OTA. An OTA error keeps output inhibited until restart. Two original OTA slots and their partition layout are preserved; use application uploads only. Initialization or output faults retain maintenance access where firmware can still execute. Hardware/power failures can prevent software recovery.

## Competition operation

1. Boot with CH5 HIGH, CH3 zero, CH1/CH2/CH4 centered and CH6 normal. Keep stationary until calibration completes.
2. With healthy hardware/radio and neutral controls, hold CH5 HIGH, then switch LOW to arm. Wi-Fi turns off when armed. There is no automatic spin or timed diagnostic stop.
3. CH1 steers heading, CH2 translates, CH3 controls spin, CH4 trims, CH5 stops/arms, CH6 selects reverse/unstick. At zero spin throttle, bounded tank/unstick operation remains available.
4. CH5 HIGH stops drive commands and restores Wi-Fi. Wait for mechanical coasting before handling. Restore neutral controls and deliberately cycle HIGH then LOW to rearm.
5. Receiver loss stops every drive mode. Valid controls expire after 100 ms; reported zero link quality stops immediately; missing link statistics expires after one second. These clocks start at received evidence, so actual RF-loss latency also depends on receiver behavior. Restoring the radio cannot automatically restart motion.

Trim is 0.9-1.1 in 0.004 steps per CH4 excursion below 30 / above 70, with a return to center between steps. Higher CH4 slows heading integration; lower speeds it up. A fractional white fill across nine LEDs shows trim: neutral 4.5 means four full LEDs and the fifth half-bright. Heading display resumes after adjustment. Trim persists after stopped maintenance saves it; confirm `trim_saved=1` before power-off. Legacy V5 trim storage is preserved but not reused.

Blue indicates inhibited maintenance/startup; amber indicates a recoverable stop; red indicates a latched sensor/fatal fault. Red requires checking `/summary.txt`, correcting the cause and restarting. The heading marker is white while spinning, with dim green for translation. Faults override trim display.

## Implementation and limits

- Sensor: existing SPI pins, 1 MHz, 400 Hz, +/-400 g, BDU, 0.195 g/count, Y-axis and 18 mm radius. Startup uses 400 fresh stationary bias samples, with a 10-second deadline. Register verification retries up to three complete reads without rewriting configuration during operation; persistent failure is latched and diagnosed in the summary.
- Fresh samples feed a 20 ms exponential filter. Continuous phase integration is independent of sample delivery; competition trim remains available. An accelerometer does not establish an absolute world heading, and phase can drift.
- Translation uses a signed full cosine, a smooth 40-60% CH2 deadband and differential/headroom bounds. It swaps the stronger wheel on the opposite half-turn without commanding wheel reversal in melty mode. There is no independent clipping that distorts the paired command sum.
- Original V5 DShot driver, wiring, startup stop/direction/3D commands and packet encoding are preserved. One output owner submits both commands; API acceptance is not an ESC acknowledgement. No ESC settings are saved.
- At 18 mm radius, 5,000 RPM requires about 503 g and exceeds the sensor range. The raw-Y near-rail guard inhibits drive before a clipped speed estimate is used. Higher-speed competition operation needs physical verification within the sensor's usable range.

## Optional diagnostics

When diagnosing a fault or timing issue, stop and keep power on, reconnect to Wi-Fi, then:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File .\Download-Run.ps1
```

The helper saves a summary, downloads small pages and verifies SHA-256. It accepts the compatible 1.0/1.1/1.2 export formats (early 1.1 manifests incorrectly said 1.0). Captures stay local and are ignored by Git.

The ring retains up to 1,500 rows, roughly 15 seconds of recorded driving/coast-down. After one second commanding STOP, logging pauses to preserve driving evidence; it resumes with driving. The final kill/fault row is retained. Idle pauses produce intentional timestamp gaps. Rearming, rebooting or uploading clears RAM logs, so download first when a recording matters.

## Developer validation

Dependencies are vendored at the tested versions with their licenses: CRSFforArduino 2025.12.11, Adafruit DotStar 1.2.5 and BusIO 1.17.4. PlatformIO uses Espressif 32 6.12.0 and the original OTA partition scheme.

```bat
pio run -e upload_ota
python tests/run_host_tests.py
python tests/test_download_helper.py
```

The host suite requires Python 3.8+ and a C++17 compiler. On Windows run it from a Visual Studio Developer Command Prompt; the downloader suite also requires Windows PowerShell. Linux CI builds the firmware and host suites; Windows CI checks the actual PowerShell helper. No test commands operate the robot or upload firmware.
