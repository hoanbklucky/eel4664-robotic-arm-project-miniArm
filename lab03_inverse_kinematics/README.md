# Lab 03 — miniArm 3D inverse kinematics

**Students: follow [STUDENT_GUIDE.md](STUDENT_GUIDE.md).** This is the main handout and the authoritative list of required submissions: a **final report (PDF)** and your measured **`calibration.json`**, both submitted privately through the course submission system. The report includes the DH derivation, GitHub/code link, three demonstration video links (including pick-and-place of at least two randomly placed blocks), results and member contributions. GitHub should contain the code, setup/run instructions and demo video links; do not upload the report or member contributions to GitHub.

Current controller/firmware: **GUI v2.2.1 / firmware v2.1.0**, Ngo Lab. First hardware enable is sequential; later upright moves and XYZ moves use Arduino interpolation. Keep the complete lab directory structure and your own calibration.

## Printable coordinate maps

- [Standard Letter version](maps/miniArm_test_mat_Letter.pdf)
- [School-printer version](maps/miniArm_test_mat_Letter_SCHOOL_PRINTER.pdf) — use this version when printing on the school printer.

Print the standard version at **100% / Actual size**, not Fit to page. The school-printer version is already enlarged to compensate for that printer; do not add further enlargement. Check the printed scale with a ruler: grid squares should be **10 mm** and the marked box **55 × 120 mm**.

Orient the arm to face the large arrow. The top/bottom numbers give **Y (forward)**; side numbers give **X**, positive to the robot's right. Align the map's coordinate origin with the GUI's base-frame origin; account for any offset if they do not coincide. Place blocks at different reachable locations, read X/Y at each block's grasp point, and enter them in the GUI in millimeters. Set **Z separately** to the claw's grasp height; the map only supplies X/Y. The grid is a coordinate aid, not a guarantee that every printed location is reachable.

## Read only what you need

| Document | Purpose | Who needs it? |
|---|---|---|
| [Student guide](STUDENT_GUIDE.md) | Ordered lab steps and report/calibration submission requirements | All students |
| [Calibration guide](CALIBRATION_GUIDE.md) | Why calibration is necessary; angle references; measurement, save/reload and validation | Students using hardware |
| [MATLAB code guide](matlab/README.md) | Assigned frames, DH, each `.m` file, transformations, FK, Jacobian and IK execution flow | Code/reference reading |
| [Expected MATLAB output](matlab/EXPECTED_OUTPUT.md) | Figures and results to compare with a successful simulation | Simulation reference |
| [Python GUI reference](miniarm_controlC/README.md) | Modes, previews, startup, connection troubleshooting and file roles | Hardware/GUI reference |
| [Block-to-cup task](PICK_PLACE.md) | Previewed grasp, lift, transfer, release and retreat | Required task; implementation reference |
| [Motion and startup details](SMOOTH_MOTION.md) | Sequential enable, final-target packets, interpolation, timing and STOP | Optional technical detail |
| [Jacobian walkthrough](matlab/JACOBIAN_WALKTHROUGH.md) | Product-rule partial derivatives and the two code loops | Optional deeper study |
| [Optional Jacobian verification](OPTIONAL_JACOBIAN_VERIFICATION.md) | Numerical derivative checks and an IK iteration exercise | Optional enrichment; not a submission |
| [MATLAB/Python pipeline audit](PYTHON_MATLAB_AUDIT.md) | Cross-language conventions, calibration equations and implementation mapping | Instructor/advanced reference |
| [Collision model](COLLISIONS.md) | Capsule checks and their limitations | Optional implementation detail |

The main handout defines the required work; the reference files explain its steps without adding separate assignments. The pick-and-place video must show at least two randomly placed blocks transferred into the cup; the task reference explains settings and limits. Other optional mathematics/maintenance pages are not additional assignments.

## Maintenance history

Historical [code review](docs/maintenance/CODE_REVIEW.md), [simulation validation](docs/maintenance/VALIDATION.md) and [GUI validation](docs/maintenance/GUI_VALIDATION.md) are grouped under `docs/maintenance/`. They preserve earlier design/test evidence; their old version descriptions are not current operating instructions. Firmware host-test instructions remain in [miniarm_controlC/qa](miniarm_controlC/qa/README.md).
