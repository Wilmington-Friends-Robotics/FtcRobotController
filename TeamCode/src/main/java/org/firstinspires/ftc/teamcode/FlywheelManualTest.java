package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manual test class for the Flywheel subsystem
 * Controls:
 * - A: Toggle flywheel on/off in current direction
 * - B: Switch direction (forward/reverse)
 * - X: Run flywheel forward for 1 second
 * - Y: Run flywheel in reverse for 1 second
 * - Left Bumper: Decrease power by 0.1
 * - Right Bumper: Increase power by 0.1
 * - DPad Up: Set power to 1.0
 * - DPad Down: Set power to 0.0
 */
@TeleOp(name = "Flywheel Manual Test", group = "Test")
public class FlywheelManualTest extends OpMode {
    private Flywheel flywheel;
    private CRServo rawFlywheel; // For direct power control testing
    private ElapsedTime timer;
    private double currentPower = 1.0;
    private boolean timedRunActive = false;
    private double timedRunStartTime = 0;
    private static final double TIMED_RUN_DURATION = 1.0; // 1 second duration
    private static final double POWER_INCREMENT = 0.1;

    @Override
    public void init() {
        // Initialize both the Flywheel class and direct servo access
        flywheel = new Flywheel(hardwareMap, "flywheel");
        rawFlywheel = hardwareMap.get(CRServo.class, "flywheel");
        timer = new ElapsedTime();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "A: Toggle | B: Switch Direction");
        telemetry.addData("Power Controls", "LB/RB: Dec/Inc | DPad: Min/Max");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleTimedRun();
        handleInputs();
        updateTelemetry();
    }

    private void handleTimedRun() {
        if (timedRunActive) {
            if (timer.seconds() - timedRunStartTime >= TIMED_RUN_DURATION) {
                flywheel.stop();
                timedRunActive = false;
            }
        }
    }

    private void handleInputs() {
        // Toggle flywheel
        if (gamepad1.a) {
            flywheel.toggle();
        }

        // Switch direction
        if (gamepad1.b) {
            if (flywheel.isRunning()) {
                flywheel.start(!flywheel.isForward());
            }
        }

        // Timed runs
        if (gamepad1.x && !timedRunActive) {
            flywheel.start(true); // Forward
            timedRunActive = true;
            timedRunStartTime = timer.seconds();
        }
        if (gamepad1.y && !timedRunActive) {
            flywheel.start(false); // Reverse
            timedRunActive = true;
            timedRunStartTime = timer.seconds();
        }

        // Power adjustments
        if (gamepad1.left_bumper) {
            currentPower = Math.max(0, currentPower - POWER_INCREMENT);
            rawFlywheel.setPower(flywheel.isForward() ? -currentPower : currentPower);
        }
        if (gamepad1.right_bumper) {
            currentPower = Math.min(1.0, currentPower + POWER_INCREMENT);
            rawFlywheel.setPower(flywheel.isForward() ? -currentPower : currentPower);
        }

        // Direct power control
        if (gamepad1.dpad_up) {
            currentPower = 1.0;
            rawFlywheel.setPower(flywheel.isForward() ? -currentPower : currentPower);
        }
        if (gamepad1.dpad_down) {
            currentPower = 0.0;
            rawFlywheel.setPower(0);
            flywheel.stop();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Flywheel Status", flywheel.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Direction", flywheel.isForward() ? "Forward" : "Reverse");
        telemetry.addData("Current Power", "%.2f", currentPower);
        telemetry.addData("Timed Run", timedRunActive ? "Active" : "Inactive");
        if (timedRunActive) {
            telemetry.addData("Time Remaining", "%.2f", 
                TIMED_RUN_DURATION - (timer.seconds() - timedRunStartTime));
        }
        telemetry.addLine("\nControls:");
        telemetry.addLine("A: Toggle On/Off");
        telemetry.addLine("B: Switch Direction");
        telemetry.addLine("X: Forward 1s");
        telemetry.addLine("Y: Reverse 1s");
        telemetry.addLine("LB/RB: Dec/Inc Power");
        telemetry.addLine("DPad Up/Down: Max/Min Power");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
} 