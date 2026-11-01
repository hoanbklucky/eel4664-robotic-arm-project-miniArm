# Lab 03 — Inverse Kinematics with Python GUI and Arduino

## Objectives
Model the miniArm's relevant kinematic chain, calculate joint targets from Cartesian coordinates, and command the arm through a Python GUI over USB serial.

## Proposed architecture
GUI (Python/Tkinter or PySide) → IK solver → servo-angle conversion and limits → pyserial → Arduino serial receiver → servos.

**Implementation status:** architecture and student requirements are specified; working Arduino firmware and kinematic calibration still require bench testing. The manufacturer's default sketch is not assumed to support custom serial commands.

## Tasks
1. Define base frame, link lengths, joint axes, tool-center point, and sign/zero offsets; distinguish physical servo angles from mathematical joint variables.
2. Implement forward kinematics and verify against several measured poses.
3. Implement analytic IK for a simplified planar subchain or numerical IK for the calibrated full model. Document any fixed wrist-orientation assumption.
4. Reject unreachable targets and solutions outside joint limits; prefer solutions near the current pose.
5. Implement a GUI with x/y/z input, Solve, Move, Home, and gripper controls; show computed angles and validation errors.
6. Implement/test a simple line-based serial protocol, e.g. `MOVE,a1,a2,a3,a4,a5\n` with `OK`/`ERR` replies. This is a proposed protocol, **not** a documented Hiwonder firmware feature.
7. Approach the target from a safe height, move slowly, and test multiple marked positions.

## Suggested files
- `python/gui.py`: GUI scaffold
- `python/kinematics.py`: FK/IK, joint limits, calibration
- `python/serial_client.py`: serial command interface
- `arduino/miniarm_serial.ino`: instructor-tested command receiver (to be developed)

## Deliverables
Source code, calibration table, FK/IK validation results, screenshots of GUI, and video of reaching at least three targets.

## Important
Servo-to-joint mapping, actual dimensions, mechanical limits, and the manufacturer servo-control implementation must be verified on the specific miniArm before students command motion.
