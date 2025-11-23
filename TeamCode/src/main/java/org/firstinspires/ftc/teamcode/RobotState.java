package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Shared robot state container that keeps the most recent field pose and velocity estimates.
 */
public class RobotState {
    private static final RobotState INSTANCE = new RobotState();

    private Pose2d poseInches = new Pose2d();
    private Pose2d velocityInches = new Pose2d();
    private Pose2D poseMillimeters = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private double headingVelocityRadPerSec = 0.0;

    private static final double VELOCITY_DEADBAND_IN_PER_SEC = 0.5; // ≈12.7 mm/s
    private static final double POSITION_DEADBAND_IN = 0.12; // ≈3 mm

    private RobotState() {
    }

    public static RobotState getInstance() {
        return INSTANCE;
    }

    public synchronized void updateFrom(PinpointFieldLocalizer localizer) {
        Pose2d rawPose = localizer.getPoseEstimate();
        Pose2d rawVelocity = localizer.getPoseVelocity();

        double filteredX = applyDeadband(rawPose.getX(), rawVelocity.getX());
        double filteredY = applyDeadband(rawPose.getY(), rawVelocity.getY());

        poseInches = new Pose2d(filteredX, filteredY, rawPose.getHeading());
        velocityInches = rawVelocity;
        headingVelocityRadPerSec = rawVelocity.getHeading();
        poseMillimeters = new Pose2D(
            DistanceUnit.MM,
            poseInches.getX() * 25.4,
            poseInches.getY() * 25.4,
            AngleUnit.RADIANS,
            poseInches.getHeading()
        );
    }

    /** Convenience overload for tests or simulated feeds without a localizer instance. */
    public synchronized void updateFrom(Pose2d rawPose, Pose2d rawVelocity) {
        double filteredX = applyDeadband(rawPose.getX(), rawVelocity.getX());
        double filteredY = applyDeadband(rawPose.getY(), rawVelocity.getY());

        poseInches = new Pose2d(filteredX, filteredY, rawPose.getHeading());
        velocityInches = rawVelocity;
        headingVelocityRadPerSec = rawVelocity.getHeading();
        poseMillimeters = new Pose2D(
                DistanceUnit.MM,
                poseInches.getX() * 25.4,
                poseInches.getY() * 25.4,
                AngleUnit.RADIANS,
                poseInches.getHeading()
        );
    }

    public synchronized void setPose(Pose2d pose) {
        poseInches = pose;
        poseMillimeters = new Pose2D(DistanceUnit.MM,
            pose.getX() * 25.4,
            pose.getY() * 25.4,
            AngleUnit.RADIANS,
            pose.getHeading());
    }

    public synchronized Pose2d getPoseInches() {
        return poseInches;
    }

    public synchronized Pose2d getVelocityInches() {
        return velocityInches;
    }

    public synchronized double getHeadingRadians() {
        return poseInches.getHeading();
    }

    public synchronized double getHeadingVelocityRadPerSec() {
        return headingVelocityRadPerSec;
    }

    public synchronized Pose2D getPoseMillimeters() {
        return poseMillimeters;
    }

    private double applyDeadband(double valueInches, double velocityInchesPerSec) {
        if (Math.abs(velocityInchesPerSec) < VELOCITY_DEADBAND_IN_PER_SEC
            && Math.abs(valueInches) < POSITION_DEADBAND_IN) {
            return 0.0;
        }
        return valueInches;
    }
}
