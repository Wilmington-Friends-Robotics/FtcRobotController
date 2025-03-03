package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main TeleOp mode for the robot.
 * Controls:
 * - DPad Up/Down: Manual intake forward/backward
 * - DPad Right: Retract intake
 * - DPad Left: Lower intake with flywheel forward
 * - Left Bumper: Lower intake with flywheel reverse
 * - Right Bumper: Move slide to low position and open claw
 * - Right Trigger: Close claw
 * - Left Trigger: Start scoring sequence
 * - Y Button: Move to ground and set elbow forward
 * - A Button: Move to ground and set elbow up
 */
@TeleOp(name = "IntoTheDeepTeleOp", group = "TeleOp")
public class IntoTheDeepTeleOp extends OpMode {
    // Robot subsystems
    private MecanumDrive mecanumDrive;
    private JoystickController joystickController;
    private Robot robot;
    private Intake intake;
    private Claw claw;
    private Flywheel flywheel;
    private ElapsedTime timer = new ElapsedTime();
    
    // State tracking variables
    private boolean flywheelScoring = false;       // Tracks if flywheel is running in scoring sequence
    private double flywheelScoringStartTime = 0;   // Tracks when flywheel started running in scoring sequnce
    private boolean isInScoringSequence = false;   // Tracks if scoring sequence is active
    private double scoringSequenceStartTime = 0;   // Time when scoring sequence started
    private static final double FLYWHEEL_RUN_TIME = 0.35; // Duration to run flywheel in seconds
    private boolean isMovingToLow = false;         // Tracks if slide is moving to low position
    private boolean isWaitingToMoveHigh = false;   // Tracks if we're waiting to move to high position
    private boolean isWaitingToMoveMedium = false; // Tracks if we're waiting to move to medium position
    private double clawCloseTime = 0;              // Time when claw was closed
    private static final double CLAW_CLOSE_WAIT_TIME = 0.7; // Time to wait after closing claw
    private int rightTriggerSequence = 0;          // Tracks the right trigger sequence stage
    private boolean rightTriggerPressed = false;   // Tracks if right trigger was previously pressed
    private boolean halfPowerMode = false;         // Tracks if we're in half power mode
    private boolean xButtonPressed = false;        // Tracks if X button was previously pressed
    private boolean started = false;
    
    // Hardware devices for manual control
    private DcMotorEx slideMotor;      // Vertical slide motor
    private TouchSensor limitSwitch;   // Bottom limit switch for slide
    private double currentSlidePower = 0.0;

    @Override
    public void init() {
        // Initialize drive motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        // Initialize subsystems
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        joystickController = new JoystickController(gamepad1, mecanumDrive);
        intake = new Intake(hardwareMap, "intake_slide");
        claw = new Claw(hardwareMap);
        flywheel = new Flywheel(hardwareMap, "flywheel");

        // Initialize and configure vertical slide motor and limit switch
        try {
            slideMotor = hardwareMap.get(DcMotorEx.class, "vertical_slide");
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            telemetry.addData("Slide Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Slide Error", e.getMessage());
        }

        // Initialize robot controller
        robot = new Robot(mecanumDrive, joystickController);
        
        telemetry.addData("Status", "Initialized - Moving to ground position and retracting intake");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!started) {
            // Initial robot setup - move to starting position
            claw.moveToGround();  // Move slide to ground position
            claw.elbowUp();       // Raise elbow
            intake.in(false);          // Retract intake
            started = true;
        }

        // Handle power mode toggle with X button
        if (gamepad1.x) {
            if (!xButtonPressed) {  // Only toggle when the button is first pressed
                joystickController.togglePowerMode();
                xButtonPressed = true;
            }
        } else {
            xButtonPressed = false;  // Reset when button is released
        }

        // Update drive controls from joystick input
        joystickController.update();
        
        // Manual intake motor control
        if (gamepad1.dpad_up) {
            intake.forward();      // Run intake motor forward
        } else if (gamepad1.dpad_down) {
            intake.backward();     // Run intake motor backward
        } else {
            intake.stop();         // Stop intake motor
        }
        
        // Intake position and flywheel control
        if (gamepad1.dpad_right) {
            intake.in(false);           // Retract intake
        } else if (gamepad1.dpad_left) {
            intake.down(true);     // Lower intake with flywheel forward
        } else if (gamepad1.left_bumper) {
            intake.down(false);    // Lower intake with flywheel reverse
        }
        
        // Slide to low position sequence
        if (gamepad1.right_bumper) {
            claw.moveToLow();
            claw.elbowSample();
        }

        if (gamepad1.b) {
            claw.openClaw();
            rightTriggerSequence = 0;  // Reset the sequence
        }
            
        // Ground pickup position
        if (gamepad1.y) {
            claw.moveToGround();
            claw.elbowForward();
            claw.wristUp();
            claw.openClaw();
            rightTriggerSequence = 0;  // Reset the sequence
        }
        
        // Reset position
        if (gamepad1.a) {
            claw.moveToGround();
            claw.elbowUp();
        }
        
        // Claw grip control
        if (gamepad1.right_trigger > 0.1) {
            if (!rightTriggerPressed) {  // Only trigger once when pressed
                if (rightTriggerSequence == 0) {
                    claw.closeClaw();
                    rightTriggerSequence = 1;
                } else if (rightTriggerSequence > 0) {
                    claw.moveToMedium();
                    mecanumDrive.drive(-1, 0, 0);
                    rightTriggerSequence = 2;
                }
                rightTriggerPressed = true;
            }
        } else {
            rightTriggerPressed = false;  // Reset when trigger is released
        }
        
        // Check if we need to move the elbow after slide reaches position
        if (rightTriggerSequence == 2 && claw.isAtTargetPosition()) {
            claw.elbowSample();
            rightTriggerSequence = 3;  // Move to next sequence to prevent repeated calls
        }
        
        // Scoring sequence
        if (gamepad1.left_trigger > 0.1) {
            // Ensure we're in position control mode
            if (claw.getCurrentMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                claw.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            
            // Calculate new target position - move down faster with harder trigger press
            int currentPos = claw.getCurrentPosition();
            int newTarget = currentPos + (int)(-100 * gamepad1.left_trigger); // Decrease by 20 ticks * trigger value
            claw.moveToPosition(newTarget);
        }
        
        // Safety check for slide bottom limit
        claw.checkLimitSwitch();
        
        // Update debug information
        telemetry.addData("Drive Mode", joystickController.isHalfPowerMode() ? "Half Power" : "Full Power");
        telemetry.addData("Slide Position", claw.getCurrentPosition());
        telemetry.addData("Slide Power", "%.2f", currentSlidePower);
        telemetry.addData("Limit Switch", limitSwitch.isPressed() ? "PRESSED" : "NOT PRESSED");
        telemetry.addData("Right Trigger Sequence", rightTriggerSequence == 0 ? "Ready to Close" : 
                                                  rightTriggerSequence == 1 ? "Ready to Raise" : 
                                                  rightTriggerSequence == 2 ? "Ready to Move Medium" : 
                                                  rightTriggerSequence == 3 ? "Sequence Complete" : "Invalid Sequence");
        telemetry.update();
    }
}

/* Example usage for both autonomous and teleop:
 * Robot myRobot = new Robot(new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor),
 *                           new JoystickController(gamepad1, new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)));
 *
 * Teleop:
 * myRobot.updateJoystickControl();
 *
 * Autonomous:
 * myRobot.drive(0.5, 0, 0); // Move forward at 50% power
 */ 