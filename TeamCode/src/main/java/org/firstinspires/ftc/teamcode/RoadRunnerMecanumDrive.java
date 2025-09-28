package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class RoadRunnerMecanumDrive extends MecanumDrive {
    // Hardware
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private GoBildaPinpointDriver pinpoint;

    private TrajectoryFollower follower;

    private double targetAngle = 0.0;
    private boolean isTurning = false;

    public RoadRunnerMecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH);

        follower = new HolonomicPIDVAFollower(
            new PIDCoefficients(DriveConstants.TRANSLATIONAL_PID_P, DriveConstants.TRANSLATIONAL_PID_I, DriveConstants.TRANSLATIONAL_PID_D),
            new PIDCoefficients(DriveConstants.TRANSLATIONAL_PID_P, DriveConstants.TRANSLATIONAL_PID_I, DriveConstants.TRANSLATIONAL_PID_D),
            new PIDCoefficients(DriveConstants.HEADING_PID_P, DriveConstants.HEADING_PID_I, DriveConstants.HEADING_PID_D),
            new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left");
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        // Initialize GoBilda Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        
        // Configure Pinpoint settings
        pinpoint.setOffsets(DriveConstants.X_OFFSET_MM, DriveConstants.Y_OFFSET_MM);
        pinpoint.setEncoderResolution(DriveConstants.PINPOINT_TICKS_PER_MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setYawScalar(DriveConstants.PINPOINT_YAW_SCALAR);
        pinpoint.resetPosAndIMU();

        // Configure motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Configure motor behavior
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, getVelocityConstraint(), getAccelerationConstraint());
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, getVelocityConstraint(), getAccelerationConstraint());
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void update() {
        // Update Pinpoint data
        pinpoint.update();
        
        // Update pose estimate using Pinpoint data
        updatePoseEstimate();
        
        if (isTurning) {
            // Simple proportional control for turning
            double currentHeading = getPoseEstimate().getHeading();
            double error = targetAngle - currentHeading;
            
            // Normalize the error to be within -π and π
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            
            // Apply proportional control
            double turnPower = error * DriveConstants.HEADING_PID_P;
            
            // Clip the power to prevent too fast turning
            turnPower = Math.max(-0.7, Math.min(0.7, turnPower));
            
            // Stop turning if we're close enough
            if (Math.abs(error) < Math.toRadians(1.0)) {
                setMotorPowers(0, 0, 0, 0);
                isTurning = false;
            } else {
                setMotorPowers(-turnPower, -turnPower, turnPower, turnPower);
            }
        } else {
            // Normal trajectory following
            DriveSignal signal = follower.update(getPoseEstimate(), getPoseVelocity());
            if (signal != null) setDriveSignal(signal);
        }
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return pinpoint.getHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        // Convert Pinpoint's millimeter readings to inches for RoadRunner
        double xPos = pinpoint.getPosX() / 25.4; // mm to inches
        double yPos = pinpoint.getPosY() / 25.4; // mm to inches
        return Arrays.asList(xPos, yPos);
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // Convert Pinpoint's mm/s readings to inches/s for RoadRunner
        double xVel = pinpoint.getVelX() / 25.4; // mm/s to inches/s
        double yVel = pinpoint.getVelY() / 25.4; // mm/s to inches/s
        return Arrays.asList(xVel, yVel);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint() {
        return new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
            new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint() {
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }

    public void turn(double angle) {
        targetAngle = angle;
        isTurning = true;
    }

    public void turnAsync(double angle) {
        turn(angle);
    }

    public boolean isBusy() {
        return follower.isFollowing() || isTurning;
    }
} 
