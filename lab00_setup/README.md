# Lab 00 — miniArm Setup and Firmware Verification

## Objectives
Install Arduino IDE, compile/upload Hiwonder's default miniArm firmware, verify neutral and knob-control modes, and document hardware configuration.

## Hardware
Hiwonder miniArm, Arduino Uno, USB Type-B data cable, supplied power source; ESP32-S3-Cam is included but not needed for this lab.

## Procedure
1. Follow the [repository setup guide](../README.md#arduino-uno-setup-and-default-firmware). Use instructor-tested FastLED 3.10.3 and the supplied `miniArm.ino` plus `tone.h`.
2. Remove Bluetooth module before upload; turn battery OFF while connecting USB as specified by Hiwonder.
3. Select Arduino Uno and COM port, Verify, Upload.
4. Power up following manufacturer instructions. Confirm neutral position (white RGB); press K1 and K2 together to enable knob mode (green RGB).
5. Carefully identify S1 claw, S2 wrist, S3 forearm, S4 arm, S5 base/pan control. Record actual motion and safe range.

## Deliverables
Screenshot of successful upload; short video of safe knob control; hardware/software inventory; troubleshooting notes.

## Safety
Keep hands clear of joints; do not force servos against mechanical stops; secure the arm and keep its work area clear.

## References
Hiwonder *3.1 Default Program Download Instruction* and *4.1 Knob Control* (manufacturer tutorial package; not redistributed).
