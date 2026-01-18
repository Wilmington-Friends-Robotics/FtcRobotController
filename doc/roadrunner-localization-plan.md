# Change Rule
Whenever code changes are made, run ./gradlew build; if it fails, fix and rebuild until it succeeds.

# RoadRunner Localization Rework Plan

## Step 1: Hardware Baseline
- Goal: Confirm new odometry wheel positions, encoder directions, and device names after relocation.
- Actions: Measure offsets from robot center, document encoder polarity, and update wiring notes.
- Validation: Quick telemetry op mode shows raw encoder counts responding in the expected direction when wheels are moved manually.

## Step 2: Pinpoint Driver Configuration
- Goal: Reconfigure the localization hardware (Pinpoint or equivalent) with new offsets and encoder directions.
- Actions: Update DriveConstants with revised X/Y offsets, ticks-per-mm, and encoder polarity; expose tunables if helpful.
- Validation: ./gradlew build passes and telemetry confirms the device reports the new configuration at init.

## Step 3: RoadRunner Pose Estimator
- Goal: Replace the existing pose update logic with RoadRunner's Localizer fed by localization hardware.
- Actions: Implement a custom Localizer that reads millimeter data, converts to inches, and updates RoadRunnerMecanumDrive.
- Validation: When pushing the robot manually, RoadRunner pose mirrors raw localization readings.

## Step 4: Motion Primitives
- Goal: Provide APIs for field-centric movement (trajectories, heading holds, PID slots).
- Actions: Expose followTrajectoryAsync, simple pose PID holds, and heading adjustments that leverage the new localizer.
- Validation: Bench tests on blocks show motor commands respond correctly without control exceptions.

## Step 5: Calibration & Verification
- Goal: Tune localization constants with relocated wheels.
- Actions: Run forward/strafe distance trials and rotation checks; adjust ticks-per-mm, yaw scalar, offsets; log results.
- Validation: Forward/strafe error < 2% and spin drift < ±3 mm after tuning.

## Step 6: Field Navigation Demo
- Goal: Demonstrate autonomous movement to field coordinates.
- Actions: Build a sample op mode that initializes pose, generates trajectories to several waypoints, and executes them.
- Validation: On-field test reaches each waypoint within tolerance with telemetry confirming pose alignment.

## Step 7: Task Integration
- Goal: Plug the new localization/motion primitives into the task controller.
- Actions: Create DriveToFieldPoseTask (or similar) using RoadRunner primitives and queue it in TaskController.
- Validation: Task-based sequences run to completion without the previous PID drift issues.

## Step 8: Documentation & Maintenance
- Goal: Capture calibration steps and future maintenance notes.
- Actions: Update docs with new offsets, tuning scripts, and operator checklists.
- Validation: Self review ensures the steps are reproducible.
