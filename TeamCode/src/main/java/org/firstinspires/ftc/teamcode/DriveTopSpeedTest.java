package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DriveTopSpeedTest", group = "Test")
public class DriveTopSpeedTest extends OpMode {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private boolean running = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private double leftMaxTps = 0.0;
    private double rightMaxTps = 0.0;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        telemetry.addLine("DriveTopSpeedTest ready");
        telemetry.addLine("A = toggle full power, B = reset max TPS");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            if (!aWasPressed) {
                running = !running;
                aWasPressed = true;
            }
        } else {
            aWasPressed = false;
        }

        if (gamepad1.b) {
            if (!bWasPressed) {
                leftMaxTps = 0.0;
                rightMaxTps = 0.0;
                bWasPressed = true;
            }
        } else {
            bWasPressed = false;
        }

        leftMotor.setPower(running ? 1.0 : 0.0);
        rightMotor.setPower(running ? 1.0 : 0.0);

        double leftTps = leftMotor.getVelocity();
        double rightTps = rightMotor.getVelocity();
        double leftAbs = Math.abs(leftTps);
        double rightAbs = Math.abs(rightTps);
        if (leftAbs > leftMaxTps) {
            leftMaxTps = leftAbs;
        }
        if (rightAbs > rightMaxTps) {
            rightMaxTps = rightAbs;
        }

        telemetry.addData("Running", running ? "ON" : "OFF");
        telemetry.addData("Left TPS", "%.1f", leftTps);
        telemetry.addData("Right TPS", "%.1f", rightTps);
        telemetry.addData("Left Max TPS", "%.1f", leftMaxTps);
        telemetry.addData("Right Max TPS", "%.1f", rightMaxTps);
        telemetry.update();
    }
}
