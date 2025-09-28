package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PinpointFieldLocalizer;
import org.firstinspires.ftc.teamcode.calibration.PinpointCalibrationHelper.Axis;
import org.firstinspires.ftc.teamcode.calibration.PinpointCalibrationHelper.DistanceCalibrationResult;
import org.firstinspires.ftc.teamcode.calibration.PinpointCalibrationHelper.HeadingCalibrationResult;
import org.firstinspires.ftc.teamcode.calibration.PinpointCalibrationHelper.Sample;

@TeleOp(name = "Pinpoint Calibration", group = "Calibration")
public class PinpointCalibrationOpMode extends LinearOpMode {
    private enum Mode {
        FORWARD_DISTANCE,
        STRAFE_DISTANCE,
        HEADING
    }

    private static final double MM_PER_INCH = 25.4;
    private static final double DEFAULT_TICKS_PER_MM = 13.26291192;
    private static final double[] DISTANCE_CHOICES_INCHES = {24.0, 36.0, 48.0};
    private static final double[] HEADING_CHOICES_DEGREES = {90.0, 135.0, 180.0};

    private boolean leftBumperLatch;
    private boolean rightBumperLatch;
    private boolean dpadUpLatch;
    private boolean dpadDownLatch;
    private boolean aLatch;
    private boolean bLatch;
    private boolean xLatch;
    private boolean yLatch;
    private boolean startLatch;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        configureMotor(frontLeftMotor);
        configureMotor(frontRightMotor);
        configureMotor(backLeftMotor);
        configureMotor(backRightMotor);

        PinpointFieldLocalizer localizer = new PinpointFieldLocalizer(hardwareMap);
        PinpointCalibrationHelper helper = new PinpointCalibrationHelper(localizer);

        Mode mode = Mode.FORWARD_DISTANCE;
        int distanceIndex = 0;
        int headingIndex = 0;

        Sample startSample = null;
        DistanceCalibrationResult lastDistanceResult = null;
        HeadingCalibrationResult lastHeadingResult = null;
        boolean startCaptured = false;
        boolean justApplied = false;

        while (!isStarted() && !isStopRequested()) {
            localizer.update();
            mode = adjustMode(mode);
            if (mode == Mode.HEADING) {
                headingIndex = adjustIndex(headingIndex, HEADING_CHOICES_DEGREES.length);
            } else {
                distanceIndex = adjustIndex(distanceIndex, DISTANCE_CHOICES_INCHES.length);
            }
            telemetryLoop(localizer, mode, distanceIndex, headingIndex,
                startCaptured, lastDistanceResult, lastHeadingResult, justApplied);
            sleep(40);
        }

        waitForStart();
        localizer.resetAndSetPose(new Pose2d());

        while (opModeIsActive() && !isStopRequested()) {
            localizer.update();
            justApplied = false;

            mode = adjustMode(mode);
            if (mode == Mode.HEADING) {
                headingIndex = adjustIndex(headingIndex, HEADING_CHOICES_DEGREES.length);
            } else {
                distanceIndex = adjustIndex(distanceIndex, DISTANCE_CHOICES_INCHES.length);
            }

            if (gamepad1.start) {
                if (!startLatch) {
                    localizer.resetAndSetPose(new Pose2d());
                    startSample = null;
                    lastDistanceResult = null;
                    lastHeadingResult = null;
                    startCaptured = false;
                    justApplied = false;
                    startLatch = true;
                }
            } else {
                startLatch = false;
            }

            if (gamepad1.a) {
                if (!aLatch) {
                    startSample = helper.captureSample();
                    startCaptured = true;
                    lastDistanceResult = null;
                    lastHeadingResult = null;
                    aLatch = true;
                }
            } else {
                aLatch = false;
            }

            if (gamepad1.b && startCaptured) {
                if (!bLatch) {
                    Sample endSample = helper.captureSample();
                    if (mode == Mode.HEADING) {
                        double expectedTurnRad = Math.toRadians(HEADING_CHOICES_DEGREES[headingIndex]);
                        lastHeadingResult = helper.computeHeadingResult(startSample, endSample, expectedTurnRad);
                        lastDistanceResult = null;
                    } else {
                        double expectedDistanceMm = DISTANCE_CHOICES_INCHES[distanceIndex] * MM_PER_INCH;
                        Axis axis = mode == Mode.FORWARD_DISTANCE ? Axis.FORWARD : Axis.STRAFE;
                        lastDistanceResult = helper.computeDistanceResult(axis, startSample, endSample, expectedDistanceMm);
                        lastHeadingResult = null;
                    }
                    startSample = null;
                    startCaptured = false;
                    bLatch = true;
                }
            } else {
                bLatch = false;
            }

            if (gamepad1.x) {
                if (!xLatch) {
                    if (lastDistanceResult != null) {
                        helper.applyDistanceCalibration(lastDistanceResult);
                        justApplied = true;
                    } else if (lastHeadingResult != null) {
                        helper.applyHeadingCalibration(lastHeadingResult);
                        justApplied = true;
                    }
                    xLatch = true;
                }
            } else {
                xLatch = false;
            }

            if (gamepad1.y) {
                if (!yLatch) {
                    startSample = null;
                    startCaptured = false;
                    lastDistanceResult = null;
                    lastHeadingResult = null;
                    yLatch = true;
                }
            } else {
                yLatch = false;
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            driveMecanum(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, forward, strafe, rotate);

            telemetryLoop(localizer, mode, distanceIndex, headingIndex,
                startCaptured, lastDistanceResult, lastHeadingResult, justApplied);
            idle();
        }
    }

    private void configureMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private Mode adjustMode(Mode currentMode) {
        if (gamepad1.left_bumper) {
            if (!leftBumperLatch) {
                int previous = (currentMode.ordinal() + Mode.values().length - 1) % Mode.values().length;
                currentMode = Mode.values()[previous];
                leftBumperLatch = true;
            }
        } else {
            leftBumperLatch = false;
        }

        if (gamepad1.right_bumper) {
            if (!rightBumperLatch) {
                int next = (currentMode.ordinal() + 1) % Mode.values().length;
                currentMode = Mode.values()[next];
                rightBumperLatch = true;
            }
        } else {
            rightBumperLatch = false;
        }
        return currentMode;
    }

    private int adjustIndex(int index, int length) {
        if (gamepad1.dpad_up) {
            if (!dpadUpLatch) {
                index = (index + 1) % length;
                dpadUpLatch = true;
            }
        } else {
            dpadUpLatch = false;
        }

        if (gamepad1.dpad_down) {
            if (!dpadDownLatch) {
                index = (index + length - 1) % length;
                dpadDownLatch = true;
            }
        } else {
            dpadDownLatch = false;
        }
        return index;
    }

    private void telemetryLoop(PinpointFieldLocalizer localizer, Mode mode, int distanceIndex, int headingIndex,
                               boolean startCaptured, DistanceCalibrationResult distanceResult,
                               HeadingCalibrationResult headingResult, boolean justApplied) {
        telemetry.addData("Mode", modeName(mode));
        telemetry.addData("Pose (in)", "x: %.2f y: %.2f h: %.1f",
            localizer.getPoseEstimate().getX(),
            localizer.getPoseEstimate().getY(),
            Math.toDegrees(localizer.getPoseEstimate().getHeading()));

        if (mode == Mode.HEADING) {
            telemetry.addData("Target Turn", "%.0f deg", HEADING_CHOICES_DEGREES[headingIndex]);
        } else {
            telemetry.addData("Target Distance", "%.0f in", DISTANCE_CHOICES_INCHES[distanceIndex]);
        }

        if (!startCaptured && distanceResult == null && headingResult == null) {
            telemetry.addLine("Press A to capture start pose, drive, then press B to capture end pose.");
        } else if (startCaptured) {
            telemetry.addLine("Drive carefully to the field mark, then press B.");
        }

        if (distanceResult != null) {
            telemetry.addData("Measured Distance", "%.2f in", distanceResult.measuredDistanceMm / MM_PER_INCH);
            telemetry.addData("Ticks Counted", "%.0f", distanceResult.deltaTicks);
            telemetry.addData("Suggested ticks/mm", "%.5f", distanceResult.suggestedTicksPerMm);
            telemetry.addData("Scale vs default", "%.3f x",
                distanceResult.scaleFactor(DEFAULT_TICKS_PER_MM));
            telemetry.addData("Error", "%.2f %%", distanceResult.errorPercent);
            telemetry.addLine("Press X to apply, Y to reset for another run.");
        }

        if (headingResult != null) {
            telemetry.addData("Measured Turn", "%.1f deg", Math.toDegrees(headingResult.measuredTurnRad));
            telemetry.addData("Suggested yaw scalar", "%.5f", headingResult.suggestedYawScalar);
            telemetry.addData("Heading error", "%.2f deg", headingResult.errorDegrees);
            telemetry.addLine("Press X to apply, Y to reset for another run.");
        }

        if (justApplied) {
            telemetry.addLine("New calibration value sent to Pinpoint.");
        }

        telemetry.addLine("Bumpers switch mode; D-Pad adjusts target; Start resets pose.");
        telemetry.update();
    }

    private String modeName(Mode mode) {
        switch (mode) {
            case STRAFE_DISTANCE:
                return "Strafe Distance";
            case HEADING:
                return "Heading";
            default:
                return "Forward Distance";
        }
    }

    private void driveMecanum(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,
                              double forward, double strafe, double rotate) {
        double frontLeftPower = forward - strafe + rotate;
        double frontRightPower = forward + strafe - rotate;
        double backLeftPower = forward + strafe + rotate;
        double backRightPower = forward - strafe - rotate;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        double scale = 0.5; // run at half speed to make calibration easier

        frontLeft.setPower(frontLeftPower * scale);
        frontRight.setPower(-frontRightPower * scale);
        backLeft.setPower(backLeftPower * scale);
        backRight.setPower(-backRightPower * scale);
    }
}
