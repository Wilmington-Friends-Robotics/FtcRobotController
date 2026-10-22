package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection;
import org.firstinspires.ftc.teamcode.config.PedroConfig;

/**
 * TeleOp diagnostic that exercises Pedro configuration data together with the Pinpoint hardware.
 * Push the robot in each axis and verify encoder signs match the values in {@code pedro_config.json}.
 */
@TeleOp(name = "Pedro Bring-up Diagnostic", group = "Diagnostics")
public class PedroBringupDiagnostic extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private PedroConfig config;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        config = PedroConfig.getInstance();
        PedroConfig.Pinpoint pinCfg = config.getPinpoint();

        telemetry.addLine("Pedro Bring-up Diagnostic");
        telemetry.addLine("Verifies pedro_config.json + Pinpoint wiring.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  - Gamepad1 A: zero Pinpoint encoders + IMU");
        telemetry.addLine("  - Move robot +X (forward) and +Y (left) slowly to confirm polarity");
        telemetry.addLine("Press START to initialize hardware.");
        telemetry.update();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinCfg.getI2cName());
        configurePinpoint();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            pinpoint.update();

            if (gamepad1.a) {
                pinpoint.resetPosAndIMU();
            }

            telemetry.addData("Device status", pinpoint.getDeviceStatus());
            telemetry.addData("Loop freq (Hz)", "%.1f", pinpoint.getFrequency());
            telemetry.addLine("--- Pose (mm/rad) ---");
            telemetry.addData("X (forward)", "%.1f", pinpoint.getPosX());
            telemetry.addData("Y (strafe)", "%.1f", pinpoint.getPosY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pinpoint.getHeading()));

            telemetry.addLine("--- Encoder deltas ---");
            telemetry.addData("X encoder", pinpoint.getEncoderX());
            telemetry.addData("Y encoder", pinpoint.getEncoderY());

            telemetry.addLine("--- Config snapshot ---");
            telemetry.addData("Deadwheel ticks/in", "%.2f", config.getDeadwheel().getTicksPerInch());
            telemetry.addData("Forward pod offsets (in)", "long=%.2f lat=%.2f",
                    config.getForwardPod().getLongitudinalOffsetInForwardPositive(),
                    config.getForwardPod().getLateralOffsetInLeftPositive());
            telemetry.addData("Lateral pod offsets (in)", "long=%.2f lat=%.2f",
                    config.getLateralPod().getLongitudinalOffsetInForwardPositive(),
                    config.getLateralPod().getLateralOffsetInLeftPositive());

            telemetry.addLine("Instructions:");
            telemetry.addLine(" 1. Push robot forward ~6\" => X encoder should increase.");
            telemetry.addLine(" 2. Push robot left ~6\" => Y encoder should decrease (right-positive).");
            telemetry.addLine(" 3. Rotate robot => heading updates smoothly.");
            telemetry.addData("Result hint", buildStatusHint(pinpoint.getDeviceStatus()));
            telemetry.update();
        }
    }

    private void configurePinpoint() {
        double xOffsetMm = inchesToMillimeters(config.getForwardPod().getLateralOffsetInLeftPositive());
        double yOffsetMm = inchesToMillimeters(config.getLateralPod().getLongitudinalOffsetInForwardPositive());
        double ticksPerMm = config.getDeadwheel().getTicksPerMm();

        pinpoint.setOffsets((float) xOffsetMm, (float) yOffsetMm);
        pinpoint.setEncoderResolution((float) ticksPerMm);
        pinpoint.setEncoderDirections(resolveDirection(config.getForwardPod().getDeltaDirection()),
                resolveDirection(config.getLateralPod().getDeltaDirection()));
        pinpoint.setYawScalar(1.0f);
        pinpoint.resetPosAndIMU();
    }

    private EncoderDirection resolveDirection(String descriptor) {
        if ("forward_positive".equalsIgnoreCase(descriptor) || "right_positive".equalsIgnoreCase(descriptor)) {
            return EncoderDirection.FORWARD;
        }
        return EncoderDirection.REVERSED;
    }

    private double inchesToMillimeters(double inches) {
        return inches * 25.4;
    }

    private String buildStatusHint(DeviceStatus status) {
        if (status == DeviceStatus.READY) {
            return "Ready: verify encoder signs via telemetry.";
        } else if (status == DeviceStatus.CALIBRATING) {
            return "Wait for IMU calibration";
        } else if (status == DeviceStatus.FAULT_X_POD_NOT_DETECTED) {
            return "Check forward pod wiring";
        } else if (status == DeviceStatus.FAULT_Y_POD_NOT_DETECTED) {
            return "Check lateral pod wiring";
        } else if (status == DeviceStatus.FAULT_IMU_RUNAWAY) {
            return "Cycle power on Pinpoint";
        }
        return "Status=" + status;
    }
}
