package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Simple drive op mode that uses the field localizer and reports pose telemetry.
 */
@TeleOp(name = "Field Drive", group = "Field")
public class FieldDriveTeleOp extends FieldLinearOpMode {
    private final RobotState robotState = RobotState.getInstance();

    @Override
    protected void onInit() {
        telemetry.addLine("Field Drive Initialized");
        telemetry.update();
    }

    @Override
    protected void onLoop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        robot.getDrive().drive(forward, strafe, rotate);

        Pose2d pose = robotState.getPoseInches();
        Pose2d velocity = robotState.getVelocityInches();

        telemetry.addData("Pose (in)", "x: %.2f y: %.2f h: %.1fdeg",
            pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Velocity (in/s)", "x: %.2f y: %.2f h: %.1fdeg/s",
            velocity.getX(), velocity.getY(), Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }
}
