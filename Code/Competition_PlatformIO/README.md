# AshNazg competition 1.4 — hardware test candidate

Recoverable sensor, output-driver, settings and worker failures no longer disarm the robot. Five continuous seconds without live controller commands disarms it; reconnect and deliberately cycle CH5 HIGH to LOW. CH5 remains the intentional stop/Wi-Fi switch. Initial boot arming retains neutral throttle/CH1/CH2; subsequent recovery does not require neutral controls or sensor recovery.

Correct sensor scaling and continuous heading remain. Stronger translation is the default, with selectable comparison mixers, independent phase adjustment, impact rejection and readable LEDs. **Motor force, phase lead, traction and fighting-speed performance are not yet physically verified.** See [TESTING.md](TESTING.md) and [RELEASE.md](RELEASE.md). Original V5 and existing release tags remain unchanged. The remote's existing `competition-v1.3` tag points at the 1.2 commit; this new build uses 1.4 to avoid that ambiguity.

## Wi-Fi upload

USB is inaccessible. CH5 HIGH, throttle zero; connect to **AshNazg Diagnostics**, password **AshNazgDiag**. From this directory:

```bat
pio run -e upload_ota -t upload
curl.exe --max-time 10 http://192.168.4.1/summary.txt
```

Expect `AshNazg competition 1.4`. Allow approximately three seconds for ESC initialization and keep still for stationary calibration. Inspect `left_output_ready`, `right_output_ready`, `heading_quality`, and `sensor_calibrated` separately. Sensor calibration and successful settings saves are not drive prerequisites; `fatal=0` remains only for old summary compatibility.

AP/IP/password, espota password `admin`, device UDP3232, computer TCP3233, Windows static Wi-Fi `192.168.4.2/24`, and both original application slots are unchanged. Application uploads only. Wi-Fi starts before peripheral workers and returns after CH5/controller-loss stop. OTA failure aborts the transaction and retains maintenance with retry and deliberate CH5 recovery available. Successful OTA reboots into stopped maintenance. Verify a second OTA from 1.4 before relying on it.

## Controls and recovery

- CH1 heading steering; CH2 translation; CH3 spin; CH4 trim; CH5 stop/arm/Wi-Fi; CH6 reverse/unstick. Tank/unstick remain independent of the accelerometer.
- Boot: CH3 zero, CH1/CH2 centered, CH5 HIGH then LOW. There is no diagnostic duration limit.
- Short reception gaps hold the last valid commands. Corrupt packets are discarded. Reported RF loss prevents held receiver channels from refreshing the five-second clock. Missing optional statistics alone does not disarm. Actual receiver failsafe reporting still needs a physical check.
- A sensor-independent 2,500-command-units/second ramp controls spin-up/reversal. Operator throttle reductions are immediate. Translation strength fades through that ramp; no accelerometer-derived motor cap remains.
- CH5 HIGH stops commanded drive and restores Wi-Fi. Outputs may mechanically coast afterward.
- Sensor faults affect estimation confidence only. Missing/clipped samples use prediction; usable measurements recover without resetting phase. Sensor startup/configuration is retried in the background.
- Busy/output errors retry per channel. Persistent problems reinstall only the affected RMT channel; the other keeps receiving commands. Physically unavailable hardware cannot transmit until recovered, but no global software fault latch is added.
- Task allocations retry independently; missing workers have cooperative main-loop fallbacks. Settings use valid saved records or RAM defaults and retry persistence while stopped.

## Translation settings

While stopped, open `http://192.168.4.1/tuning`. Save, close the page, and cycle CH5 HIGH to LOW. Tuning needs no rebuild or upload. Settings save automatically; a flash-save error cannot block driving.

| Profile | Behavior |
| --- | --- |
| **Stronger: half turn** — default | Smooth deadband; differential up to 999 × stick strength × gain, independent of base spin. Restores V5-style authority and intentional opposing wheel commands; final outputs clip to ±999. |
| Stronger: both halves | Same stronger requested peak, signed cosine. Compares waveform at the same requested authority; clipping and real motor response can change spin/force. |
| Previous bounded mixer | The 1.2 full-cosine strength/headroom behavior, using the new spin ramp. Differential stays within base/headroom; gain is limited to 1. |
| V5 reference | Original half-wave amplitude/deadband step, corrected heading and reverse/backward sign handling. Uses the new spin ramp, not V5's erroneous RPM scaling/cap. |

Defaults: half-turn profile, gain 1, angle 0°, lead 0 ms, impact rejection on, diagnostic LEDs off, CSV capture off. Strong profiles accept gain 0–1.5. Stronger braking/reversal may add translation but reduce spin, increase slip/current, or expose ESC transition delays. The profiles are comparisons, not measured performance guarantees.

Translation angle changes modulation position relative to white heading; it does not alter heading-rate trim. Positive time lead advances commands along the current spin direction. The lead contribution is bounded to ±150°. At 1,500 RPM, 5 ms is 45°; this example does not establish the actual delay. Physical angle offset mirrors in reverse; a time lead advances in either spin direction.

CH4 retains 0.9–1.1 trim in 0.004 steps per excursion, with return to center between steps. The nine-LED fractional white display remains: unity = 4.5 LEDs. Saved 1.2 trim is retained; adjustment temporarily overrides diagnostic LEDs.

## LEDs

Stopped maintenance is blue. Normal mode shows white heading and short modulation markers rather than almost-continuous green. A small amber pixel indicates predicted heading without replacing heading with a fault screen.

Enable **Diagnostic LEDs** for slow motion. Count from the software's first pixel along the strip; identify the corresponding physical end first.

| LED | Meaning |
| --- | --- |
| 1 | White heading flash, once per estimated turn |
| 2 / 3 | Last successfully submitted left/right command: brightness = magnitude, green = positive, red = negative |
| 4 | Narrow green positive-differential peak; cyan opposite peak for full-wave profiles. Requires recent successful paired submissions with unequal commands. |
| 5 | Dim blue = usable measured rate; amber = prediction/degraded measurement |
| 6 / 7 | Red if the corresponding left/right output channel is unavailable/recovering |

These show API-accepted DShot commands, not motor acknowledgment, wheel speed or force. Mark chassis/wheels for low-speed video. Phone slow motion cannot resolve every high-speed pulse because of aliasing.

## Estimator and protocol details

SPI pins, 1 MHz, 400 Hz, ±400 g/BDU, 0.195 g/count, Y radial axis and 18 mm radius remain. Read retries are bounded; documented configuration is restored in the SPI owner. Shock rejection uses radial sign, saturation, transverse acceleration and three-sample confirmation of large changes, ahead of the existing 20 ms EMA and bounded rate corrections. Impact rejection can be disabled for comparison.

Brief measurement outages retain accepted angular rate. Longer outages use the last accepted command/rate relation, learned slope or configured fallback. Without any trusted rate, the default **4 body RPM per command is an unvalidated prediction**, never a drive cap/stop. Saved stationary bias is reused; new calibration happens only while stopped/neutral. Heading uncertainty is shown, not treated as permission to stop. An accelerometer cannot supply absolute world heading or reconstruct unobserved impacts indefinitely.

At 18 mm, 5,000 body RPM requires approximately 503 g, beyond ±400 g. Clipping degrades measurement and invokes prediction; it does not disarm. [ST datasheet](https://www.st.com/resource/en/datasheet/h3lis331dl.pdf).

DShot600, pins/channels and signed encoding remain. Initial startup sends 2,500 stop packets plus ten direction and ten 3D packets per ESC. Startup is nonblocking and per channel. RMT recovery does not replay mode commands into a running ESC. The constructor's original zero terminator is now explicit and RMT configuration is initialized. UART stays at the original 416666 baud with CRC-validated CRSF parsing; its owner retries initialization/restarts a silent interface. No ESC firmware/settings are written. ESC-side faults require separate diagnosis because the current path has no ESC feedback.

## Optional diagnostics and development

`/summary.txt` reports confidence, tuning, per-channel recoveries and gaps specifically during consecutive driving iterations. `/events.txt` keeps 64 compact arm/stop/sensor/output events. Routine CSV downloads are unnecessary.

Capture is disabled by default. When enabled, the unchanged 32-column export keeps 256 rows at approximately 50 Hz (about five seconds), retaining driving and the stop/idle tail. It freezes after the final stop row. Arming/reboot/upload clears CSV evidence. Download before those actions when it matters:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File .\Download-Run.ps1
```

See [ESC-REVIEW.md](ESC-REVIEW.md) for the saved configuration assessment and [VALIDATION.md](VALIDATION.md) for software results.

```bat
pio run -e upload_ota
python tests/run_host_tests.py
python tests/test_download_helper.py
```

Host tests require a C++17 compiler/Visual Studio Developer Command Prompt. Framework remains Espressif32 6.12.0/Arduino 2.0.17. Tests compile actual runtime/transport/recovery/settings/OTA callbacks against mocks; they do not reproduce electrical motor response or RTOS scheduling.
