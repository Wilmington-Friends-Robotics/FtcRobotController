package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake Slide Test", group = "Test")
public class IntakeSlideTest extends LinearOpMode {
    private DcMotorEx intakeMotor;
    
    // Constants for power control
    private static final double MAX_POWER = 0.5;  // Maximum power to apply
    private static final double HOLDING_POWER = 0;  // Power to hold against gravity
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_slide");
            
            // Configure motor
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Instructions", "Use triggers to control slide:");
            telemetry.addData("- Right Trigger", "Extend (Forward)");
            telemetry.addData("- Left Trigger", "Retract (Backward)");
            telemetry.addData("- Y Button", "Reset Encoder");
            telemetry.addData("Motor Mode", intakeMotor.getMode());
            
            // Test if encoder is responding
            int startPos = intakeMotor.getCurrentPosition();
            intakeMotor.setPower(0.1);
            sleep(100);
            intakeMotor.setPower(0);
            int endPos = intakeMotor.getCurrentPosition();
            telemetry.addData("Encoder Test", startPos != endPos ? "WORKING" : "NOT RESPONDING");
            
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
        telemetry.update();
        
        waitForStart();
        
        double currentPower = 0.0;
        int lastPosition = 0;
        
        while (opModeIsActive()) {
            // Get trigger values for power control
            double extendPower = -gamepad1.right_trigger * MAX_POWER;    // Negative power = extend
            double retractPower = gamepad1.left_trigger * MAX_POWER;     // Positive power = retract
            
            // Combine powers (only one trigger should be used at a time)
            currentPower = extendPower + retractPower;
            
            // Manual encoder reset if needed
            if (gamepad1.y) {
                intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
            // Apply power to motor
            if (Math.abs(currentPower) < 0.01) {
                intakeMotor.setPower(HOLDING_POWER);  // Apply holding power when no input
            } else {
                intakeMotor.setPower(currentPower);
            }
            
            // Get current position and calculate change
            int currentPosition = intakeMotor.getCurrentPosition();
            int positionChange = currentPosition - lastPosition;
            lastPosition = currentPosition;
            
            // Display current state
            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("Position Change", positionChange);
            telemetry.addData("Current Power", "%.2f", currentPower);
            telemetry.addData("Controls", "RT = Extend, LT = Retract, Y = Reset Encoder");
            telemetry.update();
            
            // Small delay to prevent overwhelming the system
            sleep(20);
        }
        
        // Stop motor when done
        intakeMotor.setPower(0);
    }
} 