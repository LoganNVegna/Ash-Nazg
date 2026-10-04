# Saved ESC configuration assessment

The supplied `esc1_config_AshNazg_good.bin` contains 67 bytes, identifies `REPEAT DRIVE` and encodes firmware version 1.99 / EEPROM layout 1. It is a historical ESC1 file; it does not prove either ESC's current settings, their health, or whether the damaged/replaced motor matches the prior setup.

Using the legacy AM32 field offsets, the file suggests complementary PWM off (byte 20 = 0), variable PWM on (21 = 1), stuck-rotor protection off (22 = 0), nominal 24 kHz (24 = 24), startup power 100 (25 = 100), motor KV 1100 (26 = 27, legacy `27*40+20`), fourteen poles (27 = 14), brake-on-stop off (28 = 0), and running/stopped brake levels 1 (41/42 = 1). The observed header/name and these offsets are consistent with the legacy format. Confirm against the installed version before treating this as an active configuration; newer layouts reuse earlier reserved bytes.

Complementary PWM can affect deceleration when reducing duty; coasting would weaken rapid modulation even when commands and the heading are correct. Braking/reversal changes may improve authority but change current, heat and transient behavior. This is a hypothesis to compare with wheel/video response, not a diagnosed cause. The saved file does not point to enabled stuck-rotor protection as the match lockout explanation, although current ESC state is unknown.

No ESC settings/firmware have been written. ESP32 Wi-Fi OTA updates the controller application, not the ESCs. Do not blindly flash ESC firmware, infer current settings from this old file, or change braking/startup/ramp together. Evaluate controller authority/phase first; review both ESCs' actual settings separately if physical response still fails to follow commands.

Primary references: [legacy AM32 firmware/EEPROM decoding](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/blob/master/Src/main.c), [current EEPROM layout](https://github.com/am32-firmware/AM32/blob/main/Inc/eeprom.h), [AM32 settings explanations](https://github.com/am32-firmware/am32-wiki/blob/main/docs/guides/ESC-Settings-Explained.md).
