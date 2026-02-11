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
    private static final double FLYWHEEL_TARGET_TPS = 1200.0;
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

        telemetry.addLine("DecodeStarterTeleOp (No Cameras, No FSD) ready");
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
        double driveForward = forward;
        double driveStrafe = strafe;
        double turnCommand = turn;

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

        double denom = Math.max(1.0, Math.abs(driveForward) + Math.abs(driveStrafe) + Math.abs(turnCommand));
        double frontLeftPower = (driveForward + driveStrafe + turnCommand) / denom;
        double frontRightPower = (driveForward - driveStrafe - turnCommand) / denom;
        double backLeftPower = (driveForward - driveStrafe + turnCommand) / denom;
        double backRightPower = (driveForward + driveStrafe - turnCommand) / denom;

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
        telemetry.update();
    }

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void startServoSequence() {
        servoSequenceActive = true;
        servoPhase = 1;
        servoMoveTimer.reset();
    }
}
