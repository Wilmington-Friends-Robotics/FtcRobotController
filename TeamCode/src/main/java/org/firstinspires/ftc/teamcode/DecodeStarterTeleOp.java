package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DecodeStarterTeleOp", group = "TeleOp")
public class DecodeStarterTeleOp extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
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
    private static final double FLYWHEEL_P = 0.1;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 32767.0 / FLYWHEEL_MAX_TPS;
    private static final double FLYWHEEL_READY_TPS_TOLERANCE = 50.0;
    private static final double SERVO_MOVE_DURATION_S = 0.5; // Calibrate for ~180 degrees.
    private static final double RIGHT_CW_POS = 0.0;
    private static final double RIGHT_CCW_POS = 1.0;
    private static final double LEFT_CW_POS = 0.0;
    private static final double LEFT_CCW_POS = 1.0;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        leftServo = hardwareMap.get(Servo.class, "left_servo");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        // Reverse one side so forward stick makes the robot go forward.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setPower(0);
        rightServo.setPosition(RIGHT_CCW_POS);
        leftServo.setPosition(LEFT_CW_POS);

        telemetry.addLine("DecodeStarterTeleOp ready");
        telemetry.addLine("Left stick Y = drive, Right stick X = turn");
        telemetry.addLine("A button = toggle flywheel");
        telemetry.addLine("B button = move servos 180 degrees, then reverse");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        forward = applyDeadband(forward, 0.05);
        turn = applyDeadband(turn, 0.05);

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

        double leftPower = forward - turn;
        double rightPower = forward + turn;

        double maxMag = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= maxMag * 1.5;
        rightPower /= maxMag * 1.5;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Left Power", "%.2f", leftPower);
        telemetry.addData("Right Power", "%.2f", rightPower);
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Flywheel Target TPS", "%.0f", flywheelOn ? FLYWHEEL_TARGET_TPS : 0.0);
        telemetry.addData("Flywheel TPS", "%.1f", flywheelTps);
        telemetry.addData("Flywheel Ready", flywheelReady ? "YES" : "NO");
        telemetry.addData("Servo Move", servoSequenceActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Fire Queued", fireQueued ? "YES" : "NO");
        telemetry.update();
    }

    private double applyDeadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    private void startServoSequence() {
        servoSequenceActive = true;
        servoPhase = 1;
        servoMoveTimer.reset();
    }
}
