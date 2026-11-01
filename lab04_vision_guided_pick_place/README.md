# Lab 04 — Vision-Guided Rubber Duck Pick-and-Place

## Objectives
Detect a small yellow rubber duck using the included **front-mounted ESP32-S3-Cam**, estimate its position in the robot base frame, reuse Lab 03 IK/serial control, and pick/place the duck.

## Hardware configuration
The standard miniArm camera is mounted on the **front of the arm**, not above a fixed workspace. Camera pose therefore changes with arm motion. Use a fixed **observation pose** before each detection and calibrate for that pose. Keep the tabletop flat, lighting stable, and the duck stationary. Choose a duck that fits the gripper.

## Manufacturer starting points
- *8.4 Color Sorting*: ESP32-S3-Cam `ColorDetection.ino` and Arduino `MiniArm_color_clamp.ino`; detects blue/green blocks and triggers preprogrammed pick/place.
- *8.5 Color Tracking*: camera detects colored regions and returns a bounding box `[x,y,width,height]` via the supplied I²C interface; Arduino maps image location to servo tracking.
- *8.6 Face Detection*: illustrates a different on-camera recognition pipeline, but is not a duck detector.

These examples **do not** demonstrate a ready-made rubber-duck detector, arbitrary world-coordinate recovery, or Python video streaming. Inspect supplied source before adapting it.

## Recommended two-stage implementation

### Stage A (required baseline): color-based detection
1. At a reproducible observation pose, capture the duck's image. First verify whether ESP32-S3 firmware can stream frames to the laptop over Wi-Fi/USB. If not, adapt the supplied on-camera color detector to detect yellow and return bounding-box coordinates over I²C; alternatively use a temporary USB webcam for the laptop vision prototype.
2. If frames are available on the laptop, use Python/OpenCV: HSV yellow threshold → morphology → contours → largest plausible region → bounding-box center `(u,v)`. Include confidence/area and false-positive checks.
3. Calibrate the mapping from image pixels to tabletop coordinates at the **same camera observation pose**. A homography is appropriate for a fixed planar surface; measure reference points and verify reprojection error. Do not assume pixels are centimeters.
4. Convert tabletop coordinates to robot-base coordinates, choose a calibrated grasp height, and call Lab 03 IK.
5. Use a safe approach → descend → close gripper → lift → move to known drop zone → release → retreat sequence.
6. Measure detection error, grasp success rate, and placement success over multiple trials.

### Stage B (optional): learned detection
Train or fine-tune a small YOLO model on labeled images of the actual ducks and background; run inference on laptop frames. Keep the same calibration and IK pipeline. This is an extension, not required for the robotics learning objectives.

## Camera integration decision gate
Before promising a Python detector, test:
- Can this exact ESP32-S3-Cam board and firmware supply JPEG/video frames over Wi-Fi or USB?
- Can the supplied `ColorDetection.ino` be modified for yellow HSV thresholds and return bounding boxes over I²C?
- Can Arduino forward detection results to the laptop without conflicting with the Lab 03 serial motion interface?
- Does the front camera see the full reachable duck pickup area from a collision-free observation pose?

**Fallback:** use the ESP32-S3-Cam's I²C color-region output with Arduino forwarding centroid coordinates over USB, or prototype Python vision with a USB webcam. A color centroid alone does not prove a duck is present; require a controlled scene.

## Suggested files
- `python/detect_duck.py`: OpenCV segmentation
- `python/calibrate_camera.py`: tabletop calibration
- `python/pick_place.py`: orchestrate perception, IK, and motion
- `arduino/`: tested serial/ESP32 interface
- `calibration/`: measured points and calibration output

## Deliverables
Annotated detection image, calibration accuracy table, robot-frame target estimates, source code, and a video showing at least three successful attempts. Report failed detections/grasp attempts rather than omitting them.

## Safety
No moving-arm detection during initial trials; move to the calibrated observation pose before acquiring an image. Limit servo motion and verify collision-free approach heights. Stop on ambiguous detections or unreachable IK targets.
