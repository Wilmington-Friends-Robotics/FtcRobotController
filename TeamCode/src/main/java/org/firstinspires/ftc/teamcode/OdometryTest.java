package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Odometry Test")
public class OdometryTest extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // Initialize and configure Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        pinpoint.setOffsets(DriveConstants.X_OFFSET_MM, DriveConstants.Y_OFFSET_MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        RoadRunnerMecanumDrive drive = new RoadRunnerMecanumDrive(hardwareMap);

        // Create a test trajectory
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
            .forward(24) // Move forward 24 inches
            .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "This test will:");
        telemetry.addData("1.", "Move forward 24 inches");
        telemetry.addData("2.", "Turn 90 degrees");
        telemetry.addData("3.", "Move forward 24 inches");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Follow the forward trajectory
        drive.followTrajectory(trajectory);

        // Turn 90 degrees
        drive.turn(Math.toRadians(90));

        // Create and follow another forward trajectory
        trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
            .forward(24)
            .build();
        drive.followTrajectory(trajectory);

        while (opModeIsActive() && !isStopRequested()) {
            // Update drive and Pinpoint
            drive.update();
            pinpoint.update();

            // Display position info
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Pinpoint X (mm)", pinpoint.getPosX());
            telemetry.addData("Pinpoint Y (mm)", pinpoint.getPosY());
            telemetry.addData("Pinpoint Heading (rad)", pinpoint.getHeading());
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            telemetry.update();
        }
    }
} 