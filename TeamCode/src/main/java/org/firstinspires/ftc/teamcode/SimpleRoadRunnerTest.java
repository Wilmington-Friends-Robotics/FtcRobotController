package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Simple RoadRunner Test")
public class SimpleRoadRunnerTest extends LinearOpMode {
    // Hardware
    private GoBildaPinpointDriver pinpoint;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        
        // Configure Pinpoint settings
        pinpoint.setOffsets(DriveConstants.X_OFFSET_MM, DriveConstants.Y_OFFSET_MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        
        // Create RoadRunner drive
        RoadRunnerMecanumDrive drive = new RoadRunnerMecanumDrive(hardwareMap);
        
        // Create trajectory
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
            .forward(12.0)  // Move forward 1 foot (12 inches)
            .build();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Follow trajectory
        drive.followTrajectory(trajectory);
        
        // Turn 90 degrees
        drive.turn(Math.toRadians(90));
        
        while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
            // Update drive
            drive.update();
            
            // Display position info
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("X Position (mm)", pinpoint.getPosX());
            telemetry.addData("Y Position (mm)", pinpoint.getPosY());
            telemetry.addData("Heading (rad)", pinpoint.getHeading());
            telemetry.update();
        }
    }
} 