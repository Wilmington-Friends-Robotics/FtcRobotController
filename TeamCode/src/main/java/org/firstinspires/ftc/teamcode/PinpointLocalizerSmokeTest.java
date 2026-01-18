package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Hardware smoke test for PinpointFieldLocalizer. Verifies pose/velocity updates, encoder signs,
 * device status, and lets you drive manually to observe localization while moving.
 */
@TeleOp(name = "Pinpoint Localizer Smoke Test", group = "Diagnostics")
public class PinpointLocalizerSmokeTest extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointFieldLocalizer localizer = new PinpointFieldLocalizer(hardwareMap);
        initDriveMotors();

        telemetry.addLine("Pinpoint Localizer Smoke Test");
        telemetry.addLine("Controls:");
        telemetry.addLine("  - Left stick: drive/strafe, Right stick: rotate");
        telemetry.addLine("  - A: zero pose + heading");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                localizer.resetAndSetPose(new Pose2d());
            }

            driveWithGamepad();
            localizer.update();

            Pose2d pose = localizer.getPoseEstimate();
            Pose2d vel = localizer.getPoseVelocity();

            telemetry.addData("Status", localizer.getDeviceStatus());
            telemetry.addData("Loop Hz", "%.1f", localizer.getUpdateFrequencyHz());
            telemetry.addLine("--- Pose (in/rad) ---");
            telemetry.addData("x", "%.3f", pose.getX());
            telemetry.addData("y", "%.3f", pose.getY());
            telemetry.addData("heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("--- Velocity (in/s, rad/s) ---");
            telemetry.addData("vx", "%.3f", vel.getX());
            telemetry.addData("vy", "%.3f", vel.getY());
            telemetry.addData("omega", "%.3f", vel.getHeading());
            telemetry.addLine("--- Encoders ---");
            telemetry.addData("forward ticks", localizer.getForwardEncoderTicks());
            telemetry.addData("strafe ticks", localizer.getStrafeEncoderTicks());
            telemetry.update();
        }
    }

    private void initDriveMotors() {
        fl = hardwareMap.get(DcMotorEx.class, "front_left");
        fr = hardwareMap.get(DcMotorEx.class, "front_right");
        bl = hardwareMap.get(DcMotorEx.class, "back_left");
        br = hardwareMap.get(DcMotorEx.class, "back_right");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void driveWithGamepad() {
        double y = -gamepad1.left_stick_y; // forward
        double x = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotate

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double brPower = (y + x - rx) / denominator;
        double frPower = (y - x - rx) / denominator;

        fl.setPower(Range.clip(flPower, -1.0, 1.0));
        bl.setPower(Range.clip(blPower, -1.0, 1.0));
        br.setPower(Range.clip(brPower, -1.0, 1.0));
        fr.setPower(Range.clip(frPower, -1.0, 1.0));
    }
}
