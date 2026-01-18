# Change Rule
Whenever code changes are made, run the project build immediately; if it fails, fix the issues and rebuild until it succeeds.

# Team Context
- This code is only to be used by our FTC team
- Legacy op modes from previous seasons are for reference only—new TeleOp and autonomous routines will be authored specifically for this field-localized system.

# Pinpoint Localizer Integration Plan

## Step 1: Baseline Assessment
- Goal: Document current drive and autonomous control structure, focusing on how op modes access drivetrain and localization.
- Actions: Review existing `RoadRunnerMecanumDrive`, `IntoTheDeep*` op modes, and command scheduling patterns; identify where localization data is consumed today.
- Validation: Summarize findings in project notes and confirm no build errors after running the standard Gradle build (`./gradlew build`).

## Step 2: Localizer Wiring
- Goal: Instantiate `PinpointFieldLocalizer` wherever robot hardware is initialized.
- Actions: Update the base robot container or each op mode’s init block to create the localizer, call `setPoseEstimate` with the chosen start pose, and ensure the `update()` loop runs each iteration.
- Validation: Build succeeds; telemetry during a dry-run (robot on blocks) shows coherent pose values updating with manual movement.

### Step 2 Findings (2025-09-28)
- Added `FieldRobot` to centralize drivetrain hardware and a single `PinpointFieldLocalizer` instance, plus `FieldLinearOpMode` so every new TeleOp/Auto automatically updates the localizer each loop.
- Gradle build verified the new scaffolding; next dry-run will confirm telemetry behaves as expected once the new op modes are written.

## Step 3: Pinpoint Calibration Workflow
- Goal: Calibrate pod wheel scale and heading scalar so Pinpoint distance/heading outputs match field reality.
- Actions: Add reusable calibration helpers plus an interactive op mode that guides teams through forward, strafe, and heading calibration trials while auto-computing updated encoder resolution and yaw scalar values.
- Validation: Run the calibration op mode, follow on-screen prompts, and confirm the reported scale factors keep drive distance errors under 2% on a verification pass.

### Step 3 Findings (2025-09-28)
- Forward and strafe calibration trials converged on 1.48× the stock swingarm ticks/mm, yielding a working value of 19.6291 ticks/mm.
- Heading calibration required no adjustment (yaw scalar remains 1.0).
- Added rotation-offset calibration mode to compute pod offsets so the robot doesn’t drift in X/Y during pure spins; helper can now push the suggested offsets directly to the localizer and `DriveConstants`.

## Step 4: Pose Broadcasting
- Goal: Make the current pose accessible to any task/state machine component.
- Actions: Introduce a shared `RobotState` or similar singleton that exposes the pose from `PinpointFieldLocalizer`; refactor existing consumers to read from this source instead of disparate fields.
- Validation: Unit or component smoke test passes (if available), and telemetry reflects identical pose data before and after the refactor.

### Step 4 Findings (2025-09-28)
- Added `RobotState` singleton to mirror Pinpoint pose/velocity, wired into `FieldRobot.update()` so every loop refreshes the shared data.
- Created `Field Drive` TeleOp on top of `FieldLinearOpMode`, driving with the existing mecanum helper while broadcasting pose/velocity telemetry from `RobotState`.
- Full Gradle build succeeded; next step is to validate telemetry on-hardware during a dry run.
- Added a low-speed deadband in `RobotState` so when translational velocity is near zero, sub-3 mm residual ripple from rotations is suppressed while normal compound movements remain unaffected.

## Step 5: Task Definition API
- Goal: Define a data structure representing field waypoints and associated actions.
- Actions: Create task objects (e.g., `DriveToPoseTask`) that store target positions, tolerances, and completion callbacks; ensure they consume the shared pose and drive system references.
- Validation: Build succeeds; write a quick test op mode that queues a single waypoint and confirm the robot begins driving toward the target while reporting progress via telemetry.

### Step 5 Findings (2025-09-28)
- Introduced the `RobotTask` interface plus reusable `WaitTask` and `TimedDriveTask` samples that consume the shared `FieldRobot`/`RobotState` accessors.
- FieldRobot now exposes `getRobotState()` so future tasks can read localization data without additional plumbing.

## Step 6: State Machine Infrastructure
- Goal: Implement a task scheduler/state machine able to step through queued tasks sequentially.
- Actions: Build a simple state machine loop (e.g., `TaskController`) that updates the active task, advances on completion, and handles interrupts such as stop requests.
- Validation: Simulated run (robot disabled motors) shows state transitions in telemetry, and Gradle build remains green.

### Step 6 Findings (2025-09-28)
- Added `TaskController` to execute queued tasks sequentially and report the active task name for telemetry.
- Created `Task Controller Sample` TeleOp demonstrating wait/drive sequences with live pose reporting; build succeeds on the updated branch.

## Step 7: Multi-Waypoint Execution
- Goal: Demonstrate driving through multiple waypoints using the new system.
- Actions: Extend the test op mode to queue several poses with diverse headings; integrate velocity constraints or drive power adjustments as needed.
- Validation: Field test confirms the robot approximately reaches each waypoint within tolerance; capture logs/telemetry as evidence.
- Subtask: Add a PID-based `DriveToPoseTask` that uses odometry to drive to a field-relative target, and exercise it inside the sample task op mode for indoor validation before field testing.

## Step 8: Field-Centric Op Modes
- Goal: Deliver the new TeleOp and autonomous op modes built on `FieldLinearOpMode` with localization-aware driving logic.
- Actions: Implement fresh TeleOp/Auto classes that extend the new base, integrate reusable subsystems, and expose pose telemetry for driver feedback.
- Validation: Dry-run both op modes on blocks to confirm controls, pose updates, and subsystem behaviors before on-field testing.

## Step 9: Error Handling & Recovery
- Goal: Add safeguards for localization loss or task failure.
- Actions: Detect `PinpointFieldLocalizer` fault statuses, implement retry or abort behavior, and surface alerts to telemetry.
- Validation: Induce a simulated fault (e.g., unplug pod) and verify the system halts tasks and reports the issue while a rebuild continues to pass.

## Step 10: Documentation & Training
- Goal: Ensure the team can maintain and extend the system.
- Actions: Document setup steps, usage patterns, and tuning guidelines; create a checklist for initializing the localizer before matches.
- Validation: Peer review of documentation plus a walkthrough with another team member confirming understanding.

### Step 1 Findings (2025-09-28)
- `IntoTheDeep*` autonomous routines rely purely on timed `MecanumDrive.drive()` calls; no localization feedback loop yet.
- TeleOp also drives via `MecanumDrive` constructed from raw motors, with no field-centric state accessible beyond motor encoders.
- `RoadRunnerMecanumDrive` already wires the `GoBildaPinpointDriver` and exposes Road Runner pose/velocity, but current op modes do not instantiate it.
- Legacy `Odometry` class manually reads I2C registers; prefer the packaged `GoBildaPinpointDriver`/`PinpointFieldLocalizer` instead of extending this path.
- No shared robot container manages subsystem singletons, so each op mode builds components ad hoc—will need a central place to host the field localizer.
