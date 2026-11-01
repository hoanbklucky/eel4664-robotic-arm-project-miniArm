# Lab 02 — Arduino-Programmed Action Groups

## Objectives
Represent a robot motion as servo-angle records; modify a programmed action group and compare it with Lab 01's manual teaching.

## Background
Hiwonder *6.2.1 Action Group Control Course* provides `read_mini_actions.ino` to read angles and `mini_actions_group.ino` plus `actions.h` to run stored groups. In the documented action records, the first field indicates validity and fields 1–5 correspond to servo angles; follow the manufacturer's full array layout and required terminator exactly. The `action_count` definition must match the number of groups. The single-action *6.1.1 Brief Control Course* is optional background, not a separate lab.

## Procedure
1. Download the manufacturer's action-group examples from the authorized miniArm tutorial package.
2. Upload the read-action-group example and capture representative servo-angle data using Arduino Serial Monitor.
3. Inspect `actions.h` and identify action records, group boundaries, and termination convention.
4. Build a pick-and-place group. Modify at least one pose or insert a safe intermediate lift pose in code.
5. Upload the group-running firmware and execute at least three trials.
6. Compare with Lab 01: ease of editing, repeatability, and risk of collisions.

## Deliverables
Modified source files or a patch (do not redistribute manufacturer code without permission), a pose table, demonstration video, and a comparison paragraph.

## Safety
Check each servo angle against calibrated limits; use a clear test workspace and stop immediately if the arm approaches a hard stop.

## Reference
Hiwonder *6.2.1 Action Group Control Course*, sections 1–2.
