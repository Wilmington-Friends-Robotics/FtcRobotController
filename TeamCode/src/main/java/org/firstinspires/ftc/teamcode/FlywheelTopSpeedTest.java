package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "FlywheelTopSpeedTest", group = "Test")
public class FlywheelTopSpeedTest extends OpMode {
    private DcMotorEx flywheel;
    private boolean running = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private double maxTps = 0.0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setPower(0.0);

        telemetry.addLine("FlywheelTopSpeedTest ready");
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
                maxTps = 0.0;
                bWasPressed = true;
            }
        } else {
            bWasPressed = false;
        }

        flywheel.setPower(running ? 1.0 : 0.0);

        double tps = flywheel.getVelocity();
        double absTps = Math.abs(tps);
        if (absTps > maxTps) {
            maxTps = absTps;
        }

        telemetry.addData("Running", running ? "ON" : "OFF");
        telemetry.addData("Current TPS", "%.1f", tps);
        telemetry.addData("Max TPS", "%.1f", maxTps);
        telemetry.addData("Encoder Pos", flywheel.getCurrentPosition());
        telemetry.update();
    }
}
