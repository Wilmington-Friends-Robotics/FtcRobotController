package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Utility that wraps the goBILDA Pinpoint odometry computer so the robot always has a field-centric pose estimate.
 */
public class PinpointFieldLocalizer {
    private static final double MM_PER_INCH = 25.4;
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_SIZE_MM = FIELD_SIZE_INCHES * MM_PER_INCH;

    private final GoBildaPinpointDriver pinpoint;
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = new Pose2d();
    private GoBildaPinpointDriver.DeviceStatus lastStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    public PinpointFieldLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, "odometry", new Pose2d());
    }

    public PinpointFieldLocalizer(HardwareMap hardwareMap, Pose2d startPoseInches) {
        this(hardwareMap, "odometry", startPoseInches);
    }

    public PinpointFieldLocalizer(HardwareMap hardwareMap, String deviceName, Pose2d startPoseInches) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        pinpoint.setOffsets(DriveConstants.X_OFFSET_MM, DriveConstants.Y_OFFSET_MM);
        pinpoint.setEncoderResolution(DriveConstants.PINPOINT_TICKS_PER_MM);
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setYawScalar(DriveConstants.PINPOINT_YAW_SCALAR);
        setPoseEstimate(startPoseInches);
    }

    public void update() {
        pinpoint.update();
        lastStatus = pinpoint.getDeviceStatus();
        poseEstimate = new Pose2d(
            mmToInches(pinpoint.getPosX()),
            mmToInches(pinpoint.getPosY()),
            pinpoint.getHeading()
        );
        poseVelocity = new Pose2d(
            mmToInches(pinpoint.getVelX()),
            mmToInches(pinpoint.getVelY()),
            pinpoint.getHeadingVelocity()
        );
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public Pose2D getPoseEstimateMm() {
        return new Pose2D(
            DistanceUnit.MM,
            inchesToMm(poseEstimate.getX()),
            inchesToMm(poseEstimate.getY()),
            AngleUnit.RADIANS,
            poseEstimate.getHeading()
        );
    }

    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    public int getForwardEncoderTicks() {
        return pinpoint.getEncoderX();
    }

    public int getStrafeEncoderTicks() {
        return pinpoint.getEncoderY();
    }

    public void setPoseEstimate(Pose2d poseInches) {
        poseEstimate = poseInches;
        Pose2D poseMm = new Pose2D(
            DistanceUnit.MM,
            inchesToMm(poseInches.getX()),
            inchesToMm(poseInches.getY()),
            AngleUnit.RADIANS,
            poseInches.getHeading()
        );
        pinpoint.setPosition(poseMm);
    }

    public void resetAndSetPose(Pose2d poseInches) {
        pinpoint.resetPosAndIMU();
        setPoseEstimate(poseInches);
    }

    public void recalibrateHeading() {
        pinpoint.recalibrateIMU();
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return lastStatus;
    }

    public double getUpdateFrequencyHz() {
        return pinpoint.getFrequency();
    }

    public double getYawScalar() {
        return pinpoint.getYawScalar();
    }

    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods pods) {
        pinpoint.setEncoderResolution(pods);
    }

    public void setEncoderResolution(double ticksPerMillimeter) {
        pinpoint.setEncoderResolution(ticksPerMillimeter);
    }

    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection forward, GoBildaPinpointDriver.EncoderDirection strafe) {
        pinpoint.setEncoderDirections(forward, strafe);
    }

    public void setOffsets(double xOffsetMm, double yOffsetMm) {
        pinpoint.setOffsets(xOffsetMm, yOffsetMm);
    }

    public void setYawScalar(double yawScalar) {
        pinpoint.setYawScalar(yawScalar);
    }

    private static double mmToInches(double mm) {
        return mm / MM_PER_INCH;
    }

    private static double inchesToMm(double inches) {
        return inches * MM_PER_INCH;
    }
}
