package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTestTeleOp", group = "Test")
public class ServoTestTeleOp extends OpMode {
    private static final double POS_MIN = 0.0;
    private static final double POS_MID = 0.5;
    private static final double POS_MAX = 1.0;
    private static final double SERVO_MOVE_DURATION_S = 0.5; // Time to wait before reversing.

    private Servo rightServo;
    private Servo leftServo;
    private double rightPos = POS_MID;
    private double leftPos = POS_MID;

    private boolean bWasPressed = false;
    private boolean sequenceActive = false;
    private int sequencePhase = 0;
    private final ElapsedTime sequenceTimer = new ElapsedTime();

    @Override
    public void init() {
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        leftServo = hardwareMap.get(Servo.class, "left_servo");

        rightServo.setPosition(rightPos);
        leftServo.setPosition(leftPos);

        telemetry.addLine("ServoTestTeleOp ready");
        telemetry.addLine("A = both min, X = both mid, Y = both max");
        telemetry.addLine("Dpad left/right = opposite positions");
        telemetry.addLine("B = move then reverse after delay");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            setBoth(POS_MIN);
        }
        if (gamepad1.x) {
            setBoth(POS_MID);
        }
        if (gamepad1.y) {
            setBoth(POS_MAX);
        }
        if (gamepad1.dpad_left) {
            setPositions(POS_MIN, POS_MAX);
        }
        if (gamepad1.dpad_right) {
            setPositions(POS_MAX, POS_MIN);
        }

        if (gamepad1.b) {
            if (!bWasPressed && !sequenceActive) {
                sequenceActive = true;
                sequencePhase = 1;
                sequenceTimer.reset();
                bWasPressed = true;
            }
        } else {
            bWasPressed = false;
        }

        if (sequenceActive) {
            if (sequencePhase == 1) {
                setPositions(POS_MAX, POS_MIN);
                if (sequenceTimer.seconds() >= SERVO_MOVE_DURATION_S) {
                    sequencePhase = 2;
                    sequenceTimer.reset();
                }
            } else if (sequencePhase == 2) {
                setPositions(POS_MIN, POS_MAX);
                if (sequenceTimer.seconds() >= SERVO_MOVE_DURATION_S) {
                    sequenceActive = false;
                    sequencePhase = 0;
                }
            }
        }

        telemetry.addData("Right Pos", "%.2f", rightPos);
        telemetry.addData("Left Pos", "%.2f", leftPos);
        telemetry.addData("Sequence", sequenceActive ? "ACTIVE" : "IDLE");
        telemetry.update();
    }

    private void setBoth(double pos) {
        setPositions(pos, pos);
    }

    private void setPositions(double right, double left) {
        rightPos = clip(right);
        leftPos = clip(left);
        rightServo.setPosition(rightPos);
        leftServo.setPosition(leftPos);
    }

    private double clip(double value) {
        return Math.max(POS_MIN, Math.min(POS_MAX, value));
    }
}
