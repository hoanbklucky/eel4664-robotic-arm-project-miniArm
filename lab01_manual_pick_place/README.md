# Lab 01 — Teach-and-Repeat Pick-and-Place Using K1/K2

## Objectives
Use manual knob control and K1/K2 to teach, save, and replay a pick-and-place action sequence; explain the difference between a robot pose and a sequence of poses.

## Background
Hiwonder *4.1 Knob Control*: S1 controls claw, S2 wrist, S3 forearm, S4 arm, S5 base/pan. After leaving neutral mode, long-press K1 to enter editing and clear prior group; short-press K1 to record successive poses; long-press K1 to save/exit. Short-press K2 to play once; long-press K2 to repeat (avoid repeat until the path is safe). Check LED feedback and manufacturer instructions for your firmware.

## Procedure
1. Place a lightweight graspable object at a marked start location and a destination within reach.
2. Using knobs, teach: home → approach → descend → close claw → lift → transfer → lower → open claw → retreat → home.
3. Record each stable pose with K1; save the action group; test once with K2 while monitoring clearance.
4. Repeat at least three trials. Record failures and improve pose spacing and approach height.

## Deliverables
Short video, numbered pose table with intended purpose, trial outcomes, and a short explanation of teach-and-repeat limitations.

## Assessment
Successful safe pick/place; clear pose sequence; observations about repeatability and joint-space versus Cartesian motion.

## Reference
Hiwonder *4.1 Knob Control*, especially the mode/button table and action-group editing section.
