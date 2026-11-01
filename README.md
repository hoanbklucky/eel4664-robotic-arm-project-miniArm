# EEL4664 Robotic Arm Project — Hiwonder miniArm

Five progressive laboratory activities for the Hiwonder miniArm with Arduino Uno and the included front-mounted ESP32-S3-Cam.

| Lab | Topic | Guide |
| --- | --- | --- |
| 00 | Setup and firmware | [lab00_setup](lab00_setup/README.md) |
| 01 | K1/K2 teach-and-repeat pick/place | [lab01_manual_pick_place](lab01_manual_pick_place/README.md) |
| 02 | Programmed action groups | [lab02_action_groups](lab02_action_groups/README.md) |
| 03 | Python GUI, inverse kinematics and USB serial control | [lab03_inverse_kinematics](lab03_inverse_kinematics/README.md) |
| 04 | Rubber-duck detection, camera calibration, IK and pick/place | [lab04_vision_guided_pick_place](lab04_vision_guided_pick_place/README.md) |

**Implementation note:** Labs 00–02 use manufacturer workflows. Labs 03–04 are detailed implementation plans; custom serial firmware, robot calibration, and camera integration must be tested before classroom deployment. The front-mounted camera moves with the arm, so Lab 04 begins at a repeatable observation pose.

---

## Arduino Uno setup and default firmware

Student guide for compiling the Hiwonder miniArm default firmware using Arduino IDE. Instructor-tested with **Arduino IDE 2.3.10** and **FastLED 3.10.3**.

### 1. Install Arduino IDE

1. Download Arduino IDE 2.x from https://www.arduino.cc/en/software and install it.
2. Open Arduino IDE. In the board selector, choose **Arduino Uno**. Install the **Arduino AVR Boards** platform through Boards Manager if needed.

### 2. Install libraries

Open **Library Manager** (books icon in the left sidebar). Search and install:

- **Servo** by Arduino — controls the six servos. Keep only the intended installed copy.
- **FastLED** by Daniel Garcia — **select version 3.10.3** from the version dropdown, not the latest version. This is the version that successfully compiled in the instructor's environment.
- **PID** — follow the manufacturer's supplied library name/version if the particular firmware requires it; similarly named PID libraries are not necessarily interchangeable.

Avoid installing a second copy of FastLED from the manufacturer's ZIP when a working Library Manager copy is already installed.

### 3. Prepare the firmware folder

Obtain the firmware from the Hiwonder miniArm tutorial package or instructor. Place the files together:

```text
miniArm/
├── miniArm.ino
└── tone.h
```

Open `miniArm.ino` in Arduino IDE. Its includes are:

```cpp
#include <FastLED.h>
#include <EEPROM.h>
#include <Servo.h>
#include "tone.h"
```

`tone.h` is a local header, **not** a library to install separately. Keep it in the same sketch folder.

### 4. Verify compilation

1. Select **Arduino Uno**.
2. Click the **checkmark (Verify)** button.
3. Confirm that the Output panel reports memory usage without a compilation error.

Instructor's successful output:

```text
Sketch uses 14822 bytes (45%) of program storage space. Maximum is 32256 bytes.
Global variables use 1091 bytes (53%) of dynamic memory, leaving 957 bytes for local variables. Maximum is 2048 bytes.
```

Your memory usage may differ with tool versions.

### 5. Upload to the robot

1. **Remove the Bluetooth module** before uploading to avoid serial-port conflict.
2. Switch the **battery box OFF** before connecting the USB Type-B cable; avoid accidental shorts near the expansion board.
3. Connect the Arduino Uno using a **USB Type-B data cable**.
4. Select the correct board and **COM port** (port number varies by computer).
5. Click **Upload** (right-arrow icon) and wait for **Done uploading**.
6. Disconnect and restore normal power according to Hiwonder's instructions before testing; keep clear of moving joints.

The manufacturer describes a startup neutral position with white RGB light; pressing both buttons for about one second enables knob control (green light).

## Troubleshooting

### Multiple Servo folders or `Multiple libraries were found for "Servo.h"`

The message **alone** can be informational: Arduino may report which library it selected. If the wrong copy is selected or compilation fails:

1. Go to **File → Preferences** and note the **Sketchbook location**.
2. Close Arduino IDE; inspect the `libraries` subfolder (often `Documents\Arduino\libraries` on Windows).
3. Identify duplicate sketchbook Servo folders (e.g., `Servo` and `Servo-master`). Back up or move the unintended duplicate **outside** `libraries`; don't delete the built-in board-platform library.
4. Restart Arduino IDE and Verify. Install the official Servo library via Library Manager if necessary.

The instructor fixed a duplicate-Servo-folder problem by removing one extra copy.

### `multiple definition of '__vector_11'`

Example:

```text
Servo.cpp.o: multiple definition of '__vector_11'
FastLED/fl/build/fl.stl+.cpp.o: first defined here
collect2.exe: error: ld returned 1 exit status
Compilation error: exit status 1
```

This is an AVR interrupt-vector linker conflict between Servo and FastLED. **Fix:** Library Manager → FastLED → choose **3.10.3** → Install → Verify again. This worked for the instructor. Keep `Servo.h`; the arm needs servo control.

### `tone.h: No such file or directory`

Place `tone.h` beside `miniArm.ino` in the `miniArm` folder.

### Upload fails even though Verify succeeds

Check board/COM port, USB data cable, and serial-port conflicts. Remove the Bluetooth module before uploading.

## Student checklist

- [ ] Arduino IDE installed
- [ ] Arduino Uno selected
- [ ] Servo, FastLED 3.10.3, and any required PID library installed
- [ ] `miniArm.ino` and `tone.h` in the same folder
- [ ] Verify successful
- [ ] Bluetooth module removed and battery OFF before USB connection
- [ ] Correct COM port selected and Upload successful

## Manufacturer documentation (titles only)

The following PDFs are in the **Hiwonder miniArm tutorial package** and are **not redistributed here**:

- *2.1 Arduino IDE Installation and Introduction*
- *2.2 Arduino IDE Instruction*
- *3.1 Default Program Download Instruction*

This repository contains independently written setup instructions, not Hiwonder's PDFs or firmware. Obtain the manufacturer's files through authorized channels.
