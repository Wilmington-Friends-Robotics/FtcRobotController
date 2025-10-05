package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.PinpointFieldLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Utilities that compute updated Pinpoint scaling factors from recorded motion samples.
 */
public class PinpointCalibrationHelper {
    public enum Axis {
        FORWARD,
        STRAFE
    }

    public static class Sample {
        public final double xMm;
        public final double yMm;
        public final double headingRad;
        public final int encoderX;
        public final int encoderY;

        private Sample(double xMm, double yMm, double headingRad, int encoderX, int encoderY) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingRad = headingRad;
            this.encoderX = encoderX;
            this.encoderY = encoderY;
        }
    }

    public static class DistanceCalibrationResult {
        public final Axis axis;
        public final double expectedDistanceMm;
        public final double measuredDistanceMm;
        public final double deltaTicks;
        public final double suggestedTicksPerMm;
        public final double errorPercent;

        private DistanceCalibrationResult(Axis axis, double expectedDistanceMm, double measuredDistanceMm,
                                          double deltaTicks, double suggestedTicksPerMm, double errorPercent) {
            this.axis = axis;
            this.expectedDistanceMm = expectedDistanceMm;
            this.measuredDistanceMm = measuredDistanceMm;
            this.deltaTicks = deltaTicks;
            this.suggestedTicksPerMm = suggestedTicksPerMm;
            this.errorPercent = errorPercent;
        }

        public double scaleFactor(double currentTicksPerMm) {
            return currentTicksPerMm == 0 ? Double.NaN : suggestedTicksPerMm / currentTicksPerMm;
        }
    }

    public static class HeadingCalibrationResult {
        public final double expectedTurnRad;
        public final double measuredTurnRad;
        public final double suggestedYawScalar;
        public final double errorDegrees;

        private HeadingCalibrationResult(double expectedTurnRad, double measuredTurnRad,
                                         double suggestedYawScalar, double errorDegrees) {
            this.expectedTurnRad = expectedTurnRad;
            this.measuredTurnRad = measuredTurnRad;
            this.suggestedYawScalar = suggestedYawScalar;
            this.errorDegrees = errorDegrees;
        }
    }

    public static class OffsetCalibrationResult {
        public final double headingDeltaRad;
        public final double measuredDeltaXmm;
        public final double measuredDeltaYmm;
        public final double suggestedXOffsetMm;
        public final double suggestedYOffsetMm;

        private OffsetCalibrationResult(double headingDeltaRad, double measuredDeltaXmm, double measuredDeltaYmm,
                                        double suggestedXOffsetMm, double suggestedYOffsetMm) {
            this.headingDeltaRad = headingDeltaRad;
            this.measuredDeltaXmm = measuredDeltaXmm;
            this.measuredDeltaYmm = measuredDeltaYmm;
            this.suggestedXOffsetMm = suggestedXOffsetMm;
            this.suggestedYOffsetMm = suggestedYOffsetMm;
        }
    }

    public static class OffsetAccumulator {
        private Sample lastSample;
        private double totalHeading;
        private double totalX;
        private double totalY;

        private OffsetAccumulator(Sample start) {
            reset(start);
        }

        private void reset(Sample start) {
            lastSample = start;
            totalHeading = 0;
            totalX = 0;
            totalY = 0;
        }

        private void addSample(Sample sample) {
            if (lastSample == null) {
                lastSample = sample;
                return;
            }
            double dHeading = normalizeDelta(sample.headingRad - lastSample.headingRad);
            double dX = sample.xMm - lastSample.xMm;
            double dY = sample.yMm - lastSample.yMm;

            totalHeading += dHeading;
            totalX += dX;
            totalY += dY;
            lastSample = sample;
        }

        private OffsetCalibrationResult toResult() {
            if (Math.abs(totalHeading) < Math.toRadians(20)) {
                return new OffsetCalibrationResult(totalHeading, totalX, totalY, Double.NaN, Double.NaN);
            }
            double xOffset = totalY / totalHeading;
            double yOffset = -totalX / totalHeading;
            return new OffsetCalibrationResult(totalHeading, totalX, totalY, xOffset, yOffset);
        }

        public double getAccumulatedHeadingRad() {
            return totalHeading;
        }

        public double getAccumulatedXmm() {
            return totalX;
        }

        public double getAccumulatedYmm() {
            return totalY;
        }
    }

    private final PinpointFieldLocalizer localizer;

    public PinpointCalibrationHelper(PinpointFieldLocalizer localizer) {
        this.localizer = localizer;
    }

    public Sample captureSample() {
        localizer.update();
        Pose2D poseMm = localizer.getPoseEstimateMm();
        return new Sample(
            poseMm.getX(DistanceUnit.MM),
            poseMm.getY(DistanceUnit.MM),
            poseMm.getHeading(AngleUnit.RADIANS),
            localizer.getForwardEncoderTicks(),
            localizer.getStrafeEncoderTicks()
        );
    }

    public DistanceCalibrationResult computeDistanceResult(Axis axis, Sample start, Sample end, double expectedDistanceMm) {
        double measuredDistanceMm = axis == Axis.FORWARD
            ? end.xMm - start.xMm
            : end.yMm - start.yMm;
        double deltaTicks = axis == Axis.FORWARD
            ? end.encoderX - start.encoderX
            : end.encoderY - start.encoderY;

        double absoluteDistance = Math.abs(measuredDistanceMm);
        double absoluteTicks = Math.abs(deltaTicks);
        double suggestedTicksPerMm = expectedDistanceMm == 0 ? Double.NaN : absoluteTicks / expectedDistanceMm;
        double errorPercent = expectedDistanceMm == 0
            ? 0
            : ((absoluteDistance - expectedDistanceMm) / expectedDistanceMm) * 100.0;

        return new DistanceCalibrationResult(axis, expectedDistanceMm, absoluteDistance, absoluteTicks,
            suggestedTicksPerMm, errorPercent);
    }

    public HeadingCalibrationResult computeHeadingResult(Sample start, Sample end, double expectedTurnRad) {
        double measuredTurn = normalizeDelta(end.headingRad - start.headingRad);
        double suggestedYawScalar = measuredTurn == 0 ? Double.NaN : expectedTurnRad / measuredTurn;
        double errorDegrees = Math.toDegrees(measuredTurn - expectedTurnRad);
        return new HeadingCalibrationResult(expectedTurnRad, measuredTurn, suggestedYawScalar, errorDegrees);
    }

    public OffsetAccumulator createOffsetAccumulator(Sample start) {
        return new OffsetAccumulator(start);
    }

    public void accumulateOffsetSample(OffsetAccumulator accumulator, Sample sample) {
        if (accumulator != null) {
            accumulator.addSample(sample);
        }
    }

    public OffsetCalibrationResult computeOffsetResult(OffsetAccumulator accumulator) {
        if (accumulator == null) {
            return new OffsetCalibrationResult(0, 0, 0, Double.NaN, Double.NaN);
        }
        return accumulator.toResult();
    }

    public void applyDistanceCalibration(DistanceCalibrationResult result) {
        if (!Double.isNaN(result.suggestedTicksPerMm) && result.suggestedTicksPerMm > 0) {
            localizer.setEncoderResolution(result.suggestedTicksPerMm);
        }
    }

    public void applyHeadingCalibration(HeadingCalibrationResult result) {
        if (!Double.isNaN(result.suggestedYawScalar) && result.suggestedYawScalar > 0) {
            localizer.setYawScalar(result.suggestedYawScalar);
        }
    }

    public void applyOffsetCalibration(OffsetCalibrationResult result) {
        if (!Double.isNaN(result.suggestedXOffsetMm) && !Double.isNaN(result.suggestedYOffsetMm)) {
            localizer.setOffsets(result.suggestedXOffsetMm, result.suggestedYOffsetMm);
        }
    }

    private static double normalizeDelta(double delta) {
        while (delta > Math.PI) {
            delta -= 2 * Math.PI;
        }
        while (delta < -Math.PI) {
            delta += 2 * Math.PI;
        }
        return delta;
    }
}
