package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "00 DecodeStarterTeleOp (No Cameras, No FSD)", group = "00 DecodeStarter")
public class DecodeStarterTeleOpNoCameraNoFsd extends OpMode {
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private PinpointFieldLocalizer fieldLocalizer;
    private DcMotorEx flywheel;
    private Servo rightServo;
    private Servo leftServo;
    private boolean flywheelOn = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean fireQueued = false;
    private boolean servoSequenceActive = false;
    private int servoPhase = 0;
    private final ElapsedTime servoMoveTimer = new ElapsedTime();
    private final ElapsedTime shotBufferTimer = new ElapsedTime();
    private boolean shotBufferDelayActive = false;
    private final ElapsedTime flywheelDriveScalePidTimer = new ElapsedTime();
    private double flywheelDriveScaleIntegral = 0.0;
    private double flywheelDriveScalePrevError = 0.0;
    private double flywheelDriveScalePrevTime = 0.0;
    private boolean flywheelDriveScalePidInitialized = false;
    private static final double FLYWHEEL_IDLE_TARGET_TPS = 1000.0;
    private static final double FLYWHEEL_SHOOT_TARGET_TPS = 1150.0;
    private static final double FLYWHEEL_MAX_TPS = 2540.0; // Set this to your measured max TPS.
    private static final double FLYWHEEL_P = 1;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 32767.0 / FLYWHEEL_MAX_TPS;
    private static final double FLYWHEEL_READY_TPS_TOLERANCE = 50.0;
    private static final double DRIVE_MAX_TPS = 2860.0; // Set this to your measured drive max TPS.
    private static final double DRIVE_P = 0.1;
    private static final double DRIVE_I = 0.0;
    private static final double DRIVE_D = 0.0;
    private static final double DRIVE_F = 32767.0 / DRIVE_MAX_TPS;
    private static final double SERVO_MOVE_DURATION_S = 0.5; // Calibrate for ~180 degrees.
    private static final double SHOT_BUFFER_CHECK_DELAY_S = 0.5;
    private static final double DRIVE_SCALE_MIN_WHEN_FLYWHEEL_SPINUP = 0.55;
    private static final double FLYWHEEL_DRIVE_SCALE_PID_KP = 1.10;
    private static final double FLYWHEEL_DRIVE_SCALE_PID_KI = 0.35;
    private static final double FLYWHEEL_DRIVE_SCALE_PID_KD = 0.0;
    private static final double FLYWHEEL_DRIVE_SCALE_ERROR_DEADBAND_TPS = 15.0;
    private static final double FLYWHEEL_DRIVE_SCALE_INTEGRAL_MAX = 1.0;
    private static final double FLYWHEEL_DRIVE_SCALE_INTEGRAL_DECAY_PER_S = 2.0;
    private static final double RIGHT_CW_POS = 0.0;
    private static final double RIGHT_CCW_POS = 0.5;
    private static final double LEFT_CW_POS = 0.0;
    private static final double LEFT_CCW_POS = 0.5;

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

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setPower(0);
        rightServo.setPosition(RIGHT_CCW_POS);
        leftServo.setPosition(LEFT_CW_POS);
        shotBufferTimer.reset();
        flywheelDriveScalePidTimer.reset();
        flywheelDriveScaleIntegral = 0.0;
        flywheelDriveScalePrevError = 0.0;
        flywheelDriveScalePrevTime = 0.0;
        flywheelDriveScalePidInitialized = false;

        telemetry.addLine("DecodeStarterTeleOp (No Cameras, No FSD) ready");
        telemetry.addLine("Left stick Y = drive, Left stick X = strafe, Right stick X = turn");
        telemetry.addLine("A button = toggle flywheel");
        telemetry.addLine("B button = move servos 180 degrees, then reverse");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        forward = applyDeadband(forward, 0.05);
        strafe = applyDeadband(strafe, 0.05);
        turn = applyDeadband(turn, 0.05);

        fieldLocalizer.update();
        double heading = fieldLocalizer.getPoseEstimate().getHeading();
        double driveForward = forward;
        double driveStrafe = strafe;
        double turnCommand = turn;

        if (gamepad1.a) {
            if (!aWasPressed) {
                flywheelOn = !flywheelOn;
                aWasPressed = true;
            }
        } else {
            aWasPressed = false;
        }

        if (gamepad1.b) {
            if (!bWasPressed) {
                if (flywheelOn) {
                    fireQueued = true;
                }
                bWasPressed = true;
            }
        } else {
            bWasPressed = false;
        }

        boolean shootingRequested = fireQueued || servoSequenceActive;
        double flywheelTargetTps = 0.0;
        if (flywheelOn) {
            flywheelTargetTps = shootingRequested ? FLYWHEEL_SHOOT_TARGET_TPS : FLYWHEEL_IDLE_TARGET_TPS;
        }
        flywheel.setVelocity(flywheelTargetTps);

        double flywheelTps = flywheel.getVelocity();
        boolean flywheelReadyForShot = flywheelOn
                && Math.abs(flywheelTps - FLYWHEEL_SHOOT_TARGET_TPS) <= FLYWHEEL_READY_TPS_TOLERANCE;

        boolean canCheckBufferedShot = !shotBufferDelayActive
                || shotBufferTimer.seconds() >= SHOT_BUFFER_CHECK_DELAY_S;
        if (!servoSequenceActive && fireQueued && canCheckBufferedShot && flywheelReadyForShot) {
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
                    shotBufferDelayActive = true;
                    shotBufferTimer.reset();
                }
            }
        }

        boolean driveScaleLimiterActive = flywheelOn && shootingRequested;
        double driveScale = computeFlywheelDriveScale(
                driveScaleLimiterActive,
                flywheelTargetTps,
                flywheelTps);
        double flywheelScaleErrorTps = driveScaleLimiterActive
                ? Math.max(0.0, flywheelTargetTps - flywheelTps)
                : 0.0;
        double driveForwardCommand = driveForward * driveScale;
        double driveStrafeCommand = driveStrafe * driveScale;
        double driveTurnCommand = turnCommand * driveScale;

        double denom = Math.max(1.0,
                Math.abs(driveForwardCommand) + Math.abs(driveStrafeCommand) + Math.abs(driveTurnCommand));
        double frontLeftPower = (driveForwardCommand + driveStrafeCommand + driveTurnCommand) / denom;
        double frontRightPower = (driveForwardCommand - driveStrafeCommand - driveTurnCommand) / denom;
        double backLeftPower = (driveForwardCommand - driveStrafeCommand + driveTurnCommand) / denom;
        double backRightPower = (driveForwardCommand + driveStrafeCommand - driveTurnCommand) / denom;

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
        telemetry.addData("Flywheel Target TPS", "%.0f", flywheelTargetTps);
        telemetry.addData("Flywheel TPS", "%.1f", flywheelTps);
        telemetry.addData("Flywheel Ready", flywheelReadyForShot ? "YES" : "NO");
        telemetry.addData("Drive Scale", "%.2f", driveScale);
        telemetry.addData("Scale Err TPS", "%.1f", flywheelScaleErrorTps);
        telemetry.addData("Scale PID I", "%.3f", flywheelDriveScaleIntegral);
        telemetry.addData("Servo Move", servoSequenceActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Fire Queued", fireQueued ? "YES" : "NO");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Turn Input", "%.2f", turn);
        telemetry.update();
    }

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double computeFlywheelDriveScale(boolean active, double targetTps, double measuredTps) {
        if (!active || targetTps <= 0.0) {
            flywheelDriveScaleIntegral = 0.0;
            flywheelDriveScalePrevError = 0.0;
            flywheelDriveScalePidInitialized = false;
            return 1.0;
        }

        double now = flywheelDriveScalePidTimer.seconds();
        if (!flywheelDriveScalePidInitialized) {
            flywheelDriveScalePidInitialized = true;
            flywheelDriveScalePrevTime = now;
            flywheelDriveScalePrevError = 0.0;
            return 1.0;
        }

        double dt = now - flywheelDriveScalePrevTime;
        flywheelDriveScalePrevTime = now;
        if (dt <= 0.0) {
            return 1.0;
        }

        double errorTps = targetTps - measuredTps;
        if (errorTps < FLYWHEEL_DRIVE_SCALE_ERROR_DEADBAND_TPS) {
            errorTps = 0.0;
        }

        double errorNorm = clamp(errorTps / targetTps, 0.0, 1.0);
        if (errorNorm > 0.0) {
            flywheelDriveScaleIntegral += errorNorm * dt;
            flywheelDriveScaleIntegral = clamp(
                    flywheelDriveScaleIntegral,
                    0.0,
                    FLYWHEEL_DRIVE_SCALE_INTEGRAL_MAX);
        } else {
            flywheelDriveScaleIntegral = Math.max(
                    0.0,
                    flywheelDriveScaleIntegral - FLYWHEEL_DRIVE_SCALE_INTEGRAL_DECAY_PER_S * dt);
        }

        double derivative = (errorNorm - flywheelDriveScalePrevError) / dt;
        flywheelDriveScalePrevError = errorNorm;

        double reduction = FLYWHEEL_DRIVE_SCALE_PID_KP * errorNorm
                + FLYWHEEL_DRIVE_SCALE_PID_KI * flywheelDriveScaleIntegral
                + FLYWHEEL_DRIVE_SCALE_PID_KD * derivative;
        double maxReduction = 1.0 - DRIVE_SCALE_MIN_WHEN_FLYWHEEL_SPINUP;
        reduction = clamp(reduction, 0.0, maxReduction);
        return 1.0 - reduction;
    }

    private void startServoSequence() {
        servoSequenceActive = true;
        servoPhase = 1;
        servoMoveTimer.reset();
    }
}
