package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.PinpointFieldLocalizer;

/**
 * Adapter that lets Pedro Pathing consume {@link PinpointFieldLocalizer} as a Pedro Localizer.
 */
public class PedroPinpointLocalizer implements Localizer {
    private final PinpointFieldLocalizer delegate;
    private Pose pose = new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
    private Pose velocity = new Pose(0, 0, 0, PedroCoordinates.INSTANCE);

    public PedroPinpointLocalizer(PinpointFieldLocalizer delegate) {
        this.delegate = delegate;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public Pose getVelocity() {
        return velocity;
    }

    @Override
    public Vector getVelocityVector() {
        return velocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose startPose) {
        setPose(startPose);
        delegate.resetAndSetPose(toPose2d(startPose));
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
        delegate.setPoseEstimate(toPose2d(pose));
    }

    @Override
    public void update() {
        delegate.update();
        Pose2d rrPose = delegate.getPoseEstimate();
        Pose2d rrVel = delegate.getPoseVelocity();
        pose = new Pose(rrPose.getX(), rrPose.getY(), rrPose.getHeading(), PedroCoordinates.INSTANCE);
        velocity = new Pose(rrVel.getX(), rrVel.getY(), rrVel.getHeading(), PedroCoordinates.INSTANCE);
    }

    @Override
    public double getTotalHeading() {
        return pose.getHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return 1.0;
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        delegate.recalibrateHeading();
    }

    @Override
    public double getIMUHeading() {
        return pose.getHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
    }

    private static Pose2d toPose2d(Pose pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }
}
