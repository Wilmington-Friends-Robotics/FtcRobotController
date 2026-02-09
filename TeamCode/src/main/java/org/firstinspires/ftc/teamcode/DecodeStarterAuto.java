package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DecodeStarterAuto", group = "Auto")
public class DecodeStarterAuto extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx flywheel;
    private Servo rightServo;
    private Servo leftServo;
    private boolean servoSequenceActive = false;
    private int servoPhase = 0;
    private final ElapsedTime servoMoveTimer = new ElapsedTime();
    private static final int TOTAL_SHOTS = 3;
    private static final double FLYWHEEL_TARGET_TPS = 1250.0;
    private static final double FLYWHEEL_MAX_TPS = 2540.0;
    private static final double FLYWHEEL_P = 0.1;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 32767.0 / FLYWHEEL_MAX_TPS;
    private static final double FLYWHEEL_READY_TPS_TOLERANCE = 50.0;
    private static final double SERVO_MOVE_DURATION_S = 0.5;
    private static final double RIGHT_CW_POS = 0.0;
    private static final double RIGHT_CCW_POS = 1.0;
    private static final double LEFT_CW_POS = 0.0;
    private static final double LEFT_CCW_POS = 1.0;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        leftServo = hardwareMap.get(Servo.class, "left_servo");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setPower(0.0);
        rightServo.setPosition(RIGHT_CCW_POS);
        leftServo.setPosition(LEFT_CW_POS);
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        telemetry.addLine("DecodeStarterAuto ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        flywheel.setVelocity(FLYWHEEL_TARGET_TPS);

        int shotsQueued = TOTAL_SHOTS;
        int shotsFired = 0;
        while (opModeIsActive() && (shotsFired < TOTAL_SHOTS || servoSequenceActive || shotsQueued > 0)) {
            double flywheelTps = flywheel.getVelocity();
            boolean flywheelReady = Math.abs(flywheelTps - FLYWHEEL_TARGET_TPS)
                    <= FLYWHEEL_READY_TPS_TOLERANCE;

            if (!servoSequenceActive && shotsQueued > 0 && flywheelReady) {
                startServoSequence();
                shotsQueued--;
            }

            boolean wasSequenceActive = servoSequenceActive;
            updateServoSequence();
            if (wasSequenceActive && !servoSequenceActive) {
                shotsFired++;
            }

            telemetry.addData("Flywheel TPS", "%.1f", flywheelTps);
            telemetry.addData("Flywheel Ready", flywheelReady ? "YES" : "NO");
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Shots Queued", shotsQueued);
            telemetry.addData("Shots Remaining", TOTAL_SHOTS - shotsFired);
            telemetry.update();
            idle();
        }

        flywheel.setVelocity(0.0);

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        sleep(500);
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }

    private void startServoSequence() {
        servoSequenceActive = true;
        servoPhase = 1;
        servoMoveTimer.reset();
    }

    private void updateServoSequence() {
        if (!servoSequenceActive) {
            return;
        }

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
}
