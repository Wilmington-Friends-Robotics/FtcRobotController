package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class DecodeStarterTeleOp extends OpMode {
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private PinpointFieldLocalizer fieldLocalizer;
    private DcMotorEx flywheel;
    private Servo rightServo;
    private Servo leftServo;
    private VisionPortal leftPortal;
    private VisionPortal rightPortal;
    private AprilTagProcessor leftAprilTag;
    private AprilTagProcessor rightAprilTag;
    private boolean flywheelOn = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean isRedTeam = true;
    private boolean lastTagFromLeft = true;
    private boolean fireQueued = false;
    private boolean servoSequenceActive = false;
    private int servoPhase = 0;
    private final ElapsedTime servoMoveTimer = new ElapsedTime();
    private final ElapsedTime headingHoldTimer = new ElapsedTime();
    private final ElapsedTime headingCaptureTimer = new ElapsedTime();
    private boolean headingHoldInitialized = false;
    private boolean wasTurnCommandActive = false;
    private boolean awaitingHeadingCapture = false;
    private double headingHoldTargetRad = 0.0;
    private double headingHoldPrevError = 0.0;
    private double headingHoldPrevTime = 0.0;
    private static final double FLYWHEEL_TARGET_TPS = 1200.0;
    private static final double FLYWHEEL_MAX_TPS = 2540.0; // Set this to your measured max TPS.
    private static final double FLYWHEEL_P = 1;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 32767.0 / FLYWHEEL_MAX_TPS;
    private static final double FLYWHEEL_READY_TPS_TOLERANCE = 100.0;
    private static final double DRIVE_MAX_TPS = 2860.0; // Set this to your measured drive max TPS.
    private static final double DRIVE_P = 0.1;
    private static final double DRIVE_I = 0.0;
    private static final double DRIVE_D = 0.0;
    private static final double DRIVE_F = 32767.0 / DRIVE_MAX_TPS;
    private static final double SERVO_MOVE_DURATION_S = 0.5; // Calibrate for ~180 degrees.
    private static final double RIGHT_CW_POS = 0.0;
    private static final double RIGHT_CCW_POS = 0.5;
    private static final double LEFT_CW_POS = 0.0;
    private static final double LEFT_CCW_POS = 0.5;
    private static final double HEADING_HOLD_KP = 0.45;
    private static final double HEADING_HOLD_KI = 0.0;
    private static final double HEADING_HOLD_KD = 0.0;
    private static final double HEADING_HOLD_MAX_CORRECTION = 0.12;
    private static final double HEADING_HOLD_ERROR_DEADBAND_RAD = Math.toRadians(1.5);
    private static final double HEADING_CAPTURE_MIN_BRAKE_S = 0.08;
    private static final double HEADING_CAPTURE_MAX_WAIT_S = 0.35;
    private static final double HEADING_CAPTURE_SETTLED_RATE_RAD_PER_SEC = Math.toRadians(12.0);
    private static final double LEFT_CAMERA_YAW_OFFSET_DEG = -90.0;
    private static final double RIGHT_CAMERA_YAW_OFFSET_DEG = 90.0;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right");
        fieldLocalizer = new PinpointFieldLocalizer(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        leftAprilTag = new AprilTagProcessor.Builder().build();
        rightAprilTag = new AprilTagProcessor.Builder().build();
        leftPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "left_webcam"))
                .addProcessor(leftAprilTag)
                .build();
        rightPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "right_webcam"))
                .addProcessor(rightAprilTag)
                .build();

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F);
        frontRightMotor.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F);
        backLeftMotor.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F);
        backRightMotor.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        // Reverse one side so forward stick makes the robot go forward.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setPower(0);
        rightServo.setPosition(RIGHT_CCW_POS);
        leftServo.setPosition(LEFT_CW_POS);
        headingHoldTimer.reset();
        headingCaptureTimer.reset();

        telemetry.addLine("DecodeStarterTeleOp ready");
        telemetry.addLine("Left stick Y = drive, Left stick X = strafe, Right stick X = turn");
        telemetry.addLine("A button = toggle flywheel");
        telemetry.addLine("B button = move servos 180 degrees, then reverse");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        forward = applyDeadband(forward, 0.05);
        strafe = applyDeadband(strafe, 0.05);
        turn = applyDeadband(turn, 0.05);

        fieldLocalizer.update();
        double heading = fieldLocalizer.getPoseEstimate().getHeading();
        double headingRateRadPerSec = fieldLocalizer.getHeadingVelocityRadPerSec();
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);
        double fieldForward = strafe * sinHeading + forward * cosHeading;
        double fieldStrafe = -(strafe * cosHeading - forward * sinHeading);
        double turnCorrection = 0.0;
        double holdTurnCommand = 0.0;

        double now = headingHoldTimer.seconds();
        if (!headingHoldInitialized) {
            headingHoldInitialized = true;
            headingHoldTargetRad = heading;
            headingHoldPrevError = 0.0;
            headingHoldPrevTime = now;
        }
        double dt = now - headingHoldPrevTime;
        headingHoldPrevTime = now;

        AprilTagDetection calibrationTag = findCalibrationTag();
        if (calibrationTag != null) {
            Double fieldHeadingDeg = getFieldHeadingDegForTag(calibrationTag.id);
            if (fieldHeadingDeg != null) {
                double cameraYawOffset = lastTagFromLeft ? LEFT_CAMERA_YAW_OFFSET_DEG : RIGHT_CAMERA_YAW_OFFSET_DEG;
                double measuredYawDeg = calibrationTag.ftcPose.yaw + cameraYawOffset;
                double desiredHeadingRad = angleWrapRadians(Math.toRadians(fieldHeadingDeg - measuredYawDeg));
                Pose2d pose = fieldLocalizer.getPoseEstimate();
                fieldLocalizer.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), desiredHeadingRad));
                heading = desiredHeadingRad;
                cosHeading = Math.cos(heading);
                sinHeading = Math.sin(heading);
                fieldForward = strafe * sinHeading + forward * cosHeading;
                fieldStrafe = -(strafe * cosHeading - forward * sinHeading);
                headingHoldTargetRad = heading;
                headingHoldPrevError = 0.0;
                awaitingHeadingCapture = false;
            }
        }

        boolean turnCommandActive = Math.abs(turn) > 0.0;
        if (turnCommandActive) {
            awaitingHeadingCapture = false;
            headingHoldPrevError = 0.0;
        } else {
            if (wasTurnCommandActive) {
                awaitingHeadingCapture = true;
                headingCaptureTimer.reset();
                headingHoldPrevError = 0.0;
            }
            if (awaitingHeadingCapture) {
                double brakeElapsed = headingCaptureTimer.seconds();
                boolean minimumBrakeApplied = brakeElapsed >= HEADING_CAPTURE_MIN_BRAKE_S;
                boolean rotationSettled = Math.abs(headingRateRadPerSec) <= HEADING_CAPTURE_SETTLED_RATE_RAD_PER_SEC;
                boolean waitTimedOut = brakeElapsed >= HEADING_CAPTURE_MAX_WAIT_S;
                if (minimumBrakeApplied && (rotationSettled || waitTimedOut)) {
                    headingHoldTargetRad = heading;
                    headingHoldPrevError = 0.0;
                    awaitingHeadingCapture = false;
                }
            }
            if (!awaitingHeadingCapture) {
                double headingError = angleWrapRadians(headingHoldTargetRad - heading);
                if (Math.abs(headingError) < HEADING_HOLD_ERROR_DEADBAND_RAD) {
                    headingError = 0.0;
                }
                if (dt > 0.0) {
                    double derivative = (headingError - headingHoldPrevError) / dt;
                    turnCorrection = HEADING_HOLD_KP * headingError
                            + HEADING_HOLD_KI * 0.0
                            + HEADING_HOLD_KD * derivative;
                    turnCorrection = clamp(turnCorrection, -HEADING_HOLD_MAX_CORRECTION, HEADING_HOLD_MAX_CORRECTION);
                    holdTurnCommand = -turnCorrection;
                }
                headingHoldPrevError = headingError;
            }
        }
        wasTurnCommandActive = turnCommandActive;

        double turnCommand = clamp(turn + holdTurnCommand, -1.0, 1.0);

        if (gamepad1.a) {
            if (!aWasPressed) {
                flywheelOn = !flywheelOn;
                flywheel.setVelocity(flywheelOn ? FLYWHEEL_TARGET_TPS : 0.0);
                aWasPressed = true;
            }
        } else {
            aWasPressed = false;
        }

        double flywheelTps = flywheel.getVelocity();
        boolean flywheelReady = flywheelOn
                && Math.abs(flywheelTps - FLYWHEEL_TARGET_TPS) <= FLYWHEEL_READY_TPS_TOLERANCE;

        if (gamepad1.b) {
            if (!bWasPressed) {
                if (flywheelOn) {
                    if (!servoSequenceActive && flywheelReady) {
                        startServoSequence();
                    } else {
                        fireQueued = true;
                    }
                }
                bWasPressed = true;
            }
        } else {
            bWasPressed = false;
        }

        if (!servoSequenceActive && fireQueued && flywheelReady) {
            startServoSequence();
            fireQueued = false;
        }

        if (servoSequenceActive) {
            if (servoPhase == 1) {
                rightServo.setPosition(RIGHT_CW_POS);
                leftServo.setPosition(LEFT_CCW_POS);
                if (servoMoveTimer.seconds() >= SERVO_MOVE_DURATION_S) {
                    servoPhase = 2;
                    servoMoveTimer.reset();
                }
            } else if (servoPhase == 2) {
                rightServo.setPosition(RIGHT_CCW_POS);
                leftServo.setPosition(LEFT_CW_POS);
                if (servoMoveTimer.seconds() >= SERVO_MOVE_DURATION_S) {
                    servoSequenceActive = false;
                    servoPhase = 0;
                }
            }
        }

        double denom = Math.max(1.0, Math.abs(fieldForward) + Math.abs(fieldStrafe) + Math.abs(turnCommand));
        double frontLeftPower = (fieldForward + fieldStrafe + turnCommand) / denom;
        double frontRightPower = (fieldForward - fieldStrafe - turnCommand) / denom;
        double backLeftPower = (fieldForward - fieldStrafe + turnCommand) / denom;
        double backRightPower = (fieldForward + fieldStrafe - turnCommand) / denom;

        double frontLeftTargetTps = clamp(frontLeftPower * DRIVE_MAX_TPS, -DRIVE_MAX_TPS, DRIVE_MAX_TPS);
        double frontRightTargetTps = clamp(frontRightPower * DRIVE_MAX_TPS, -DRIVE_MAX_TPS, DRIVE_MAX_TPS);
        double backLeftTargetTps = clamp(backLeftPower * DRIVE_MAX_TPS, -DRIVE_MAX_TPS, DRIVE_MAX_TPS);
        double backRightTargetTps = clamp(backRightPower * DRIVE_MAX_TPS, -DRIVE_MAX_TPS, DRIVE_MAX_TPS);
        frontLeftMotor.setVelocity(frontLeftTargetTps);
        frontRightMotor.setVelocity(frontRightTargetTps);
        backLeftMotor.setVelocity(backLeftTargetTps);
        backRightMotor.setVelocity(backRightTargetTps);

        double frontLeftActualTps = frontLeftMotor.getVelocity();
        double frontRightActualTps = frontRightMotor.getVelocity();
        double backLeftActualTps = backLeftMotor.getVelocity();
        double backRightActualTps = backRightMotor.getVelocity();

        telemetry.addData("FL Cmd", "%.2f", frontLeftPower);
        telemetry.addData("FR Cmd", "%.2f", frontRightPower);
        telemetry.addData("BL Cmd", "%.2f", backLeftPower);
        telemetry.addData("BR Cmd", "%.2f", backRightPower);
        telemetry.addData("FL Target TPS", "%.0f", frontLeftTargetTps);
        telemetry.addData("FR Target TPS", "%.0f", frontRightTargetTps);
        telemetry.addData("BL Target TPS", "%.0f", backLeftTargetTps);
        telemetry.addData("BR Target TPS", "%.0f", backRightTargetTps);
        telemetry.addData("FL TPS", "%.0f", frontLeftActualTps);
        telemetry.addData("FR TPS", "%.0f", frontRightActualTps);
        telemetry.addData("BL TPS", "%.0f", backLeftActualTps);
        telemetry.addData("BR TPS", "%.0f", backRightActualTps);
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Flywheel Target TPS", "%.0f", flywheelOn ? FLYWHEEL_TARGET_TPS : 0.0);
        telemetry.addData("Flywheel TPS", "%.1f", flywheelTps);
        telemetry.addData("Flywheel Ready", flywheelReady ? "YES" : "NO");
        telemetry.addData("Servo Move", servoSequenceActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Fire Queued", fireQueued ? "YES" : "NO");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Turn Input", "%.2f", turn);
        telemetry.addData("Turn Hold", "%.2f", holdTurnCommand);
        telemetry.addData("Hold Target (deg)", "%.1f", Math.toDegrees(headingHoldTargetRad));
        telemetry.addData("Hold Error (deg)", "%.1f", Math.toDegrees(angleWrapRadians(headingHoldTargetRad - heading)));
        telemetry.addData("Heading Rate (deg/s)", "%.1f", Math.toDegrees(headingRateRadPerSec));
        telemetry.addData("Hold Braking", awaitingHeadingCapture ? "YES" : "NO");
        telemetry.addData("Team", isRedTeam ? "RED" : "BLUE");
        telemetry.update();
    }

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double angleWrapRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private void startServoSequence() {
        servoSequenceActive = true;
        servoPhase = 1;
        servoMoveTimer.reset();
    }

    private AprilTagDetection findCalibrationTag() {
        AprilTagDetection detection = getFirstTag(leftAprilTag);
        if (detection != null) {
            lastTagFromLeft = true;
            return detection;
        }
        detection = getFirstTag(rightAprilTag);
        if (detection != null) {
            lastTagFromLeft = false;
        }
        return detection;
    }

    private AprilTagDetection getFirstTag(AprilTagProcessor processor) {
        if (processor == null) {
            return null;
        }
        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection != null && (detection.id == 20 || detection.id == 24)) {
                return detection;
            }
        }
        return null;
    }

    private Double getFieldHeadingDegForTag(int tagId) {
        if (isRedTeam) {
            if (tagId == 24) {
                return 45.0;
            }
            if (tagId == 20) {
                return 135.0;
            }
        } else {
            if (tagId == 20) {
                return -45.0;
            }
            if (tagId == 24) {
                return -135.0;
            }
        }
        return null;
    }

    protected void setTeamRed(boolean isRed) {
        isRedTeam = isRed;
    }
}
