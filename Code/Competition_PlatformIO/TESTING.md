# Useful physical tests for 1.4

No routine CSV download is required. Restore two working motors before assessing translation. Keep the older firmware project as an OTA rollback source. Do motor trials in the robot's test enclosure.

## First comparison

1. CH5 HIGH, throttle zero; connect to AshNazg Diagnostics and upload the application. Check `/summary.txt` for 1.4 and both output-ready fields. Keep still for calibration; a sensor warning no longer inhibits arming.
2. At `/tuning`, enable diagnostic LEDs. Keep **Stronger: half turn**, gain 1, angle 0°, lead 0 ms, impact rejection on and capture off. Save and close the page; settings need no new upload.
3. Neutral boot controls, CH5 HIGH then LOW. Establish the familiar low spin used previously. Try a small CH2 excursion, then approximately three-quarter travel for two to three seconds if controllable. Center it and stop with CH5. Note speed and direction relative to white heading. Stronger profiles intentionally permit opposing wheel commands.
4. Film one short clip with visible chassis/wheel marks: one second spinning before translation, then two seconds translating. LED 1 is white heading; 2/3 show submitted left/right magnitude/direction; 4 shows modulation peaks; 5 shows measurement quality. Red wheel-command color means negative command, not a lockout.
5. Select **Previous bounded mixer** and repeat at the same spin/stick input. More movement with strong half implicates strength/braking. Both moving away from white suggests phase. Changing commands with little wheel response suggests drivetrain/ESC response.

## Phase and medium spin

Keep strength/profile fixed while changing angle. Begin with 0°, +45° and -45°; extend through the remaining circle if necessary. Refine near the clearest heading-aligned movement. Repeat the useful setting at medium spin. If best angle changes with RPM, try time lead in small steps, such as 1 ms, instead of modifying heading-rate trim. Negative lead is allowed for diagnosis. Numeric command LEDs alone do not establish actual torque delay.

Once timing is useful, compare **Stronger: both halves** at the same gain. Note displacement, spin loss, hopping or curvature. Check forward/back translation and reverse spin at low output. Keep settings that physically help; software tests do not declare an optimal gain/lead.

## Recovery and OTA

At low spin, turn off the transmitter for more than five seconds. The firmware deliberately holds last commands during the grace period, then disarms. Choose CH5 HIGH before reconnecting; afterward deliberately cycle HIGH to LOW. Reconnection alone must not restart motion. Brief interruption must not disarm. This checks the installed receiver's reporting, which mocks cannot prove.

With CH5 HIGH, verify Wi-Fi returns. Upload the same 1.4 image again; then test an interrupted upload while stopped. Another upload and deliberate CH5 rearming must work without power reset or USB.

Sensor faults are tested with software injection, not unplugged wiring on a moving robot. Representative contact and a match-duration run at intended fighting speed remain physical acceptance tests. For unexplained behavior, preserve power and fetch the small summary/events pages. Use a full capture only for a particular unresolved question.
