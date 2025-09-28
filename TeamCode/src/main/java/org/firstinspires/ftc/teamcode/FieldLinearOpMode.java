package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Base LinearOpMode that wires in {@link FieldRobot} and keeps the Pinpoint localizer updated each loop.
 */
public abstract class FieldLinearOpMode extends LinearOpMode {
    protected FieldRobot robot;

    protected Pose2d getInitialPose() {
        return new Pose2d();
    }

    protected void onInit() throws InterruptedException {
        // Default no-op. Override as needed.
    }

    protected void onLoop() throws InterruptedException {
        // Default no-op. Override as needed.
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new FieldRobot(hardwareMap, getInitialPose());
        onInit();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
            onLoop();
            idle();
        }
    }
}
