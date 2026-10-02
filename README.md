# Beyblade-Melty-Brain-Kit
First 1lb plastic melty brain combat robot kit! This repository will be mostly for the code.

*Please note that I'm a mechanical engineer, so the code may not meet CS standards*

## Current competition firmware

[Competition 1.2](Code/Competition_PlatformIO/README.md) contains the operator-tested smooth melty translation, corrected sensor scaling/sample timing, competition trim display, radio-loss stops and Wi-Fi OTA support. The original [V5 project](Code/V5_PlatformIO) is preserved.

See the [release checklist](Code/Competition_PlatformIO/RELEASE.md) for the remaining physical sign-off. Routine captures are optional. Build and upload from `Code/Competition_PlatformIO`; its default environment uses OTA because the robot's USB port is inaccessible.
