package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.PedroConfig;
import org.firstinspires.ftc.teamcode.pedro.PedroFollowerFactory;

/**
 * Integration smoke test for Pedro follower + Pinpoint localizer + drivetrain.
 * Lets you drive with gamepad while Pedro updates pose and drives the motors.
 */
@TeleOp(name = "Pedro Integration Test", group = "Diagnostics")
public class PedroIntegrationTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointFieldLocalizer localizer = new PinpointFieldLocalizer(hardwareMap, new Pose2d());
        Follower follower = PedroFollowerFactory.build(hardwareMap, localizer, PedroConfig.getInstance());
        Pose startPose = new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.startTeleopDrive(true);

        telemetry.addLine("Pedro Integration Test");
        telemetry.addLine("Left stick drive/strafe, right stick rotate. A = zero pose.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Prime Pedro once so its internal pose is non-null before we feed teleop inputs.
        localizer.update();
        RobotState.getInstance().updateFrom(localizer);
        follower.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                localizer.resetAndSetPose(new Pose2d());
                Pose resetPose = new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
                follower.setStartingPose(resetPose);
                follower.setPose(resetPose);
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x; // Flip so right stick right = strafe right
            double rotate = -gamepad1.right_stick_x; // Flip so right stick right = clockwise

            follower.setTeleOpDrive(forward, strafe, rotate, true);

            localizer.update();
            RobotState.getInstance().updateFrom(localizer);
            follower.update();

            Pose2d pinpointPose = localizer.getPoseEstimate();
            Pose pedroPose = follower.getPose();
            if (pedroPose == null) {
                pedroPose = new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
            }

            telemetry.addLine("Status: READY");
            telemetry.addLine("--- Pinpoint Pose (in/rad) ---");
            telemetry.addData("x", "%.2f", pinpointPose.getX());
            telemetry.addData("y", "%.2f", pinpointPose.getY());
            telemetry.addData("h(deg)", "%.1f", Math.toDegrees(pinpointPose.getHeading()));

            telemetry.addLine("--- Pedro Pose (in/rad) ---");
            telemetry.addData("x", "%.2f", pedroPose.getX());
            telemetry.addData("y", "%.2f", pedroPose.getY());
            telemetry.addData("h(deg)", "%.1f", Math.toDegrees(pedroPose.getHeading()));

            telemetry.addLine("--- Diff ---");
            telemetry.addData("dx", "%.2f", pedroPose.getX() - pinpointPose.getX());
            telemetry.addData("dy", "%.2f", pedroPose.getY() - pinpointPose.getY());
            telemetry.addData("dh(deg)", "%.1f", Math.toDegrees(pedroPose.getHeading() - pinpointPose.getHeading()));
            telemetry.update();
        }
    }
}
