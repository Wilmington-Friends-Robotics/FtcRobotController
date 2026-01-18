package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.PedroConfig;

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
    private double headingVelocityRadPerSec = 0.0;
    private GoBildaPinpointDriver.DeviceStatus lastStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    public PinpointFieldLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, PedroConfig.getInstance(), new Pose2d());
    }

    public PinpointFieldLocalizer(HardwareMap hardwareMap, Pose2d startPoseInches) {
        this(hardwareMap, PedroConfig.getInstance(), startPoseInches);
    }

    public PinpointFieldLocalizer(HardwareMap hardwareMap, PedroConfig config, Pose2d startPoseInches) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.getPinpoint().getI2cName());

        // Configure from pedro_config.json
        // Pinpoint expects X offset for the forward pod (positive = left of center)
        // and Y offset for the lateral pod (positive = forward of center).
        double forwardPodOffsetMm = inchesToMm(config.getForwardPod().getLateralOffsetInLeftPositive());
        double lateralPodOffsetMm = inchesToMm(config.getLateralPod().getLongitudinalOffsetInForwardPositive());
        pinpoint.setOffsets((float) forwardPodOffsetMm, (float) lateralPodOffsetMm);
        pinpoint.setEncoderResolution(config.getDeadwheel().getTicksPerMm());
        pinpoint.setEncoderDirections(
                mapDirectionX(config.getForwardPod().getDeltaDirection()),
                mapDirectionY(config.getLateralPod().getDeltaDirection())
        );
        pinpoint.setYawScalar(1.0f);
        resetAndSetPose(startPoseInches);
    }

    public void update() {
        pinpoint.update();
        lastStatus = pinpoint.getDeviceStatus();
        poseEstimate = new Pose2d(
            mmToInches(pinpoint.getPosX()),
            mmToInches(pinpoint.getPosY()),
            Math.toRadians(pinpoint.getHeading())
        );
        headingVelocityRadPerSec = Math.toRadians(pinpoint.getHeadingVelocity());
        poseVelocity = new Pose2d(
            mmToInches(pinpoint.getVelX()),
            mmToInches(pinpoint.getVelY()),
            poseEstimate.getHeading() // Keep velocity frame aligned to robot heading
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

    public double getHeadingVelocityRadPerSec() {
        return headingVelocityRadPerSec;
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

    // X pod should increase when moving forward
    private static GoBildaPinpointDriver.EncoderDirection mapDirectionX(String descriptor) {
        if (descriptor == null) return GoBildaPinpointDriver.EncoderDirection.FORWARD;
        String key = descriptor.toLowerCase();
        if (key.contains("forward")) return GoBildaPinpointDriver.EncoderDirection.FORWARD;
        if (key.contains("back") || key.contains("reverse")) return GoBildaPinpointDriver.EncoderDirection.REVERSED;
        return GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    // Y pod should increase when moving left
    private static GoBildaPinpointDriver.EncoderDirection mapDirectionY(String descriptor) {
        if (descriptor == null) return GoBildaPinpointDriver.EncoderDirection.FORWARD;
        String key = descriptor.toLowerCase();
        if (key.contains("left")) return GoBildaPinpointDriver.EncoderDirection.FORWARD;
        if (key.contains("right")) return GoBildaPinpointDriver.EncoderDirection.REVERSED;
        return GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
