# Pedro Pathing Odometry Implementation Plan

The goal is to stand up a reliable odometry/localization stack based on the goBILDA Pinpoint Odometry Computer, *two* deadwheel pods (one forward/back, one lateral), and the Pinpoint’s built-in IMU for heading. The system should integrate with Pedro Pathing for autonomous motion and expose the same pose estimates for field-centric TeleOp control. Follow the steps in order; each stage builds on the previous one. Whenever a step touches source code or build scripts, finish the edits by running `./gradlew build` so regressions surface immediately in addition to the verification tasks called out below. **Rule:** if a verification step can be executed automatically by this agent (e.g., running tests, lint, or scripts), it should be run immediately after the related deliverable is produced. If the verification must happen manually but depends on code (e.g., an OpMode), create or update the needed code artifact so the manual test is straightforward.

## 1. Hardware + Configuration Readiness
1. **Mount the deadwheel pods and capture geometry.**
   - Overview: Install the lateral pod (measures X) and the forward pod (measures Y) per Pinpoint guidance, locking both axes orthogonally so the IMU can supply heading; measure wheel diameter, encoder CPR, lateral distance, and forward offset so software math starts with accurate data.
   - Deliverables: Securely mounted pods, written measurements logged in `doc/odometry-measurements.md`, updated `DriveConstants` placeholders for wheel radius/track widths (including IMU offsets).
   - Verification: Spin each pod and ensure telemetry from an existing encoder test OpMode reports non-zero ticks without binding; no code build needed.
2. **Wire and power the Pinpoint Odometry Computer.**
   - Overview: Connect the Pinpoint to an I2C port on the control hub, confirm voltage and address, and label harnessing to simplify later debugging.
   - Deliverables: Verified I2C wiring, documented port number/address, photos or notes stored with the robot notebook.
   - Verification: Use the REV hardware client (or SDK `I2cDeviceSynch` ping) to confirm the device acknowledges on the bus; no build required.
3. **Confirm encoder polarity and axis conventions.**
   - Overview: Create/modify a minimal TeleOp that streams encoder deltas and IMU heading to telemetry to verify +X (lateral pod) is robot-left and +Y (forward pod) is forward, documenting any sign flips needed.
   - Deliverables: Temporary OpMode (e.g., `EncoderDirectionCheck`) and notes listing the sign of each encoder plus the IMU yaw direction.
   - Verification: Run the OpMode, push the robot in +X/+Y, confirm telemetry matches expectations, and run `./gradlew build` after coding changes.

## 2. Pedro Pathing Library Setup
4. **Add Pedro Pathing to the Gradle build.**
   - Overview: Declare the Pedro dependency (or module) inside `build.dependencies.gradle` and sync so its classes are available to `TeamCode`.
   - Deliverables: Updated `build.dependencies.gradle` pointing at `com.pedropathing:ftc:2.0.1`, Gradle/AGP toolchain bumped to Gradle 8.0.2 + Android Gradle Plugin 8.1.4 (plus `android.nonTransitiveRClass=false` and `android.suppressUnsupportedCompileSdk=29` for legacy FTC resources), dependency lock updates if applicable, synced project.
   - Verification: Execute `./gradlew build` to ensure the dependency resolves and the codebase still compiles.
5. **Store Pedro configuration data in resources.**
   - Overview: Create the JSON (or YAML) config that Pedro expects, containing wheel radius, track width, lateral distance, and tuning scalars so autonomous and TeleOp share the same source of truth.
   - Deliverables: `TeamCode/src/main/resources/pedro_config.json` (or equivalent) committed with the latest measurements.
   - Verification: Add a tiny config loader test (unit or instrumentation) that parses the file, then run `./gradlew build` to verify the resource is packaged.
6. **Expose Pedro config through a wrapper class.**
   - Overview: Implement `PedroConfig` (or similar) that loads the JSON once, caches values, and offers getters for OpModes/tasks to avoid repeated parsing.
   - Deliverables: New Java/Kotlin class with unit coverage if possible.
   - Verification: Run the associated unit test plus `./gradlew build` to confirm the wrapper compiles and passes tests.

## 3. Localization Module
7. **Implement `PinpointLocalizer`.**
   - Overview: Open the I2C device, stream two-wheel displacement plus IMU yaw from the Pinpoint, convert mm→inches, fuse the data into an SE(2) pose, and ensure heading is wrapped in radians compatible with Pedro/Road Runner conventions.
   - Deliverables: `PinpointLocalizer` class, helper conversion utilities, interface compliance with Pedro’s localization contract, and IMU zeroing support.
   - Verification: Write a hardware-opmode smoke test that instantiates the localizer, confirms telemetry updates, and finish with `./gradlew build` for compile-time coverage.
8. **Integrate the localizer into `RobotState`.**
   - Overview: Register the new localizer as the authoritative pose provider, expose `getPose()`, `setPose()`, and `getHeadingVelocity()` so any subsystem can query it.
   - Deliverables: Updated `RobotState` singleton, thread-safe pose cache, optional observer hooks.
   - Verification: Create a unit/integration test that mocks the localizer feed and asserts `RobotState` mirrors it; run `./gradlew build` afterward.
9. **Add an `OdometryTest` diagnostic OpMode.**
   - Overview: Telemetry stream pose, velocity, and raw encoder deltas to the Driver Station and `FtcDashboard` for bench validation and regression tracking.
   - Deliverables: `OdometryTest` OpMode with dashboard packets and logging toggles.
   - Verification: Deploy/run the OpMode on hardware, capture a short log ensuring values change when the robot is moved, then run `./gradlew build` to keep CI green.

## 4. Pedro Pathing Drive Integration
10. **Instantiate Pedro’s drive class with the external localizer.**
    - Overview: Configure `MecanumDrive` (or equivalent) so it pulls poses from `PinpointLocalizer` instead of internal encoder odometry, aligning Pedro with the hardware truth source.
    - Deliverables: New drive builder/factory code that wires the localizer and Pedro tuning constants together.
    - Verification: Create an integration test (or bench-mode OpMode) that prints both Pedro and Pinpoint poses to ensure they match; run `./gradlew build` once code changes are complete.
11. **Map Pedro drive signals to motor power outputs.**
    - Overview: Adapt the Pedro `DriveSignal` into the existing `RoadRunnerMecanumDrive` or another drivetrain class so follower outputs actually drive the hardware.
    - Deliverables: Adapter or updated drive class plus any needed motor feedforward constants.
    - Verification: Run a short “drive 12 inches” command on blocks, ensure motors respond without oscillation, and follow with `./gradlew build`.
12. **Collect reusable path primitives.**
    - Overview: Implement a `PedroPathFactory` (or similar) that builds frequently used paths/splines so OpModes can request them by name.
    - Deliverables: Factory class and sample path definitions checked into source control.
    - Verification: Write a unit test that builds each path and asserts no exceptions plus expected start/end poses; end with `./gradlew build`.

## 5. Autonomous Task Pipeline
13. **Create `PedroFollowPathTask`.**
    - Overview: Extend your task framework so a task can accept a Pedro path and block/yield until the follower reports completion or timeout.
    - Deliverables: New task class, task controller wiring, configurable timeout parameters.
    - Verification: Execute the task in simulation or on hardware against a short path, confirming the state machine advances, then run `./gradlew build`.
14. **Compose autonomous routines from tasks.**
    - Overview: Build higher-level autonomous sequences that reset pose, follow paths, and trigger mechanisms, keeping logic data-driven.
    - Deliverables: Updated OpModes (e.g., `TaskControllerSampleOpMode`) referencing the new tasks and starting poses stored in configuration.
    - Verification: Dry-run the OpMode on a field (or simulator), ensuring telemetry shows the expected task order; rebuild with `./gradlew build` after edits.
15. **Add dashboard markers for visualization.**
    - Overview: Emit `FtcDashboard` markers that render planned Pedro paths together with live pose so offsets are caught early.
    - Deliverables: Dashboard packet helpers and drawing code invoked from autonomous tasks.
    - Verification: Run the OpMode with dashboard connected, verify paths render, then run `./gradlew build`.

## 6. Field-Centric TeleOp
16. **Rotate driver inputs using `RobotState` heading.**
    - Overview: Update `FieldDriveTeleOp` so joystick vectors are rotated by the global heading before being sent to the mecanum power calculator, enabling field-centric drive.
    - Deliverables: Modified TeleOp class, helper method for vector rotation, optional driver preference toggle.
    - Verification: Use the Driver Station to confirm pushing forward always moves away from the driver regardless of robot heading, then run `./gradlew build`.
17. **Add a zero-heading control.**
    - Overview: Implement a button combo that resets the Pinpoint heading (and `RobotState`) so drivers can realign if drift occurs mid-match.
    - Deliverables: TeleOp input mapping, method that calls `PinpointLocalizer.reset()` with the current pose.
    - Verification: While TeleOp is running, trigger the combo and confirm telemetry heading snaps to 0; rebuild via `./gradlew build`.
18. **Stream pose health telemetry.**
    - Overview: Continuously publish pose, heading, and any confidence metrics so drivers know when field-centric control is trustworthy.
    - Deliverables: Telemetry/Dashboard output blocks inside TeleOp.
    - Verification: Observe telemetry values updating at runtime and ensure alerts trigger on disconnect; run `./gradlew build`.

## 7. Calibration + Testing
19. **Run Pedro tuning utilities with Pinpoint truth.**
    - Overview: Use Pedro’s feedforward and track-width tuning routines while sourcing ground-truth pose from the localizer to dial in motion parameters.
    - Deliverables: Updated tuning constants stored in config files and notes in `doc/odometry-measurements.md`.
    - Verification: Execute each tuning OpMode, verify residual error is within spec (<2%), and rerun `./gradlew build` if any constants change in code.
20. **Validate translation accuracy.**
    - Overview: Drive 72" forward and 36" left using Pedro commands, comparing commanded vs. measured displacement to ensure <1% error.
    - Deliverables: Test logs, updated error metrics in documentation.
    - Verification: Analyze telemetry/logs automatically (e.g., Python notebook) to compute error; no build unless code adjustments are needed.
21. **Rehearse full autonomous paths.**
    - Overview: Run complete match paths on a safe surface, comparing Pedro’s planned trajectories to actual pose traces to catch integration drift.
    - Deliverables: Saved `.ppth` files, dashboard screenshots, issue list for deviations.
    - Verification: Overlay planned vs. actual paths (dashboard export or plotting script) and confirm deviations stay within tolerance; rebuild only if code tweaks follow.

## 8. Maintenance Hooks
22. **Document constants and offsets.**
    - Overview: Keep `doc/odometry-measurements.md` synchronized with the latest wheel diameters, offsets, and configuration versions so future recalibration is fast.
    - Deliverables: Updated documentation with timestamps and authorship.
    - Verification: Markdown lint or spell-check pass (e.g., `markdownlint doc/odometry-measurements.md`); no build unless code was touched.
23. **Add automated tests for localization math.**
    - Overview: Write unit tests that feed mocked Pinpoint packets into `PinpointLocalizer` and ensure the pose conversion math stays correct across refactors.
    - Deliverables: Test class under `TeamCode/src/test` plus fixtures for sample packets.
    - Verification: Run `./gradlew test` (which also triggers `./gradlew build`) and ensure the tests pass.
24. **Create a pre-event regression checklist.**
    - Overview: Formalize a script or document covering firmware checks, offset recalibration, short Pedro auto validation, and TeleOp field-centric sanity checks before events.
    - Deliverables: Checklist markdown under `doc/` and optional reusable test OpMode instructions.
    - Verification: Dry-run the checklist before packing for an event and capture completion notes; no code build needed unless the checklist triggers changes.

Following this flow with explicit deliverables and verification gates keeps hardware, localization, and software layers synchronized, resulting in dependable odometry for autonomous precision and intuitive field-centric driver control.
