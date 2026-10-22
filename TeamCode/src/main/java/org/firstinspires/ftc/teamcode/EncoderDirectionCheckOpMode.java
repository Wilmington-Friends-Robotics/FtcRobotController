package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Diagnostic OpMode that streams raw encoder counts and heading deltas from the goBILDA Pinpoint module.
 * Push the robot along +X (forward) and +Y (left) to determine which direction produces positive ticks.
 */
@TeleOp(name = "Encoder Direction Check", group = "Diagnostics")
public class EncoderDirectionCheckOpMode extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        pinpoint.setOffsets(DriveConstants.X_OFFSET_MM, DriveConstants.Y_OFFSET_MM);
        pinpoint.setEncoderResolution(DriveConstants.PINPOINT_TICKS_PER_MM);
        pinpoint.setEncoderDirections(DriveConstants.PINPOINT_X_DIRECTION, DriveConstants.PINPOINT_Y_DIRECTION);
        pinpoint.setYawScalar(DriveConstants.PINPOINT_YAW_SCALAR);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Encoder Direction Check Ready");
        telemetry.addLine("Push robot +X (forward) and +Y (left)");
        telemetry.addLine("Press X to zero encoders/heading");
        telemetry.update();

        waitForStart();

        int lastX = pinpoint.getEncoderX();
        int lastY = pinpoint.getEncoderY();
        double lastHeading = pinpoint.getHeading();
        ElapsedTime deltaTimer = new ElapsedTime();

        while (opModeIsActive()) {
            pinpoint.update();

            int currentX = pinpoint.getEncoderX();
            int currentY = pinpoint.getEncoderY();
            double currentHeading = pinpoint.getHeading();

            int deltaX = currentX - lastX;
            int deltaY = currentY - lastY;
            double deltaHeading = currentHeading - lastHeading;

            telemetry.addData("Δt (s)", "%.3f", deltaTimer.seconds());
            telemetry.addData("X encoder", "%d (Δ %d)", currentX, deltaX);
            telemetry.addData("Y encoder", "%d (Δ %d)", currentY, deltaY);
            telemetry.addData("Heading (rad)", "%.3f (Δ %.3f)", currentHeading, deltaHeading);
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.update();

            lastX = currentX;
            lastY = currentY;
            lastHeading = currentHeading;
            deltaTimer.reset();

            if (gamepad1.x) {
                pinpoint.resetPosAndIMU();
                lastX = pinpoint.getEncoderX();
                lastY = pinpoint.getEncoderY();
                lastHeading = pinpoint.getHeading();
            }
        }
    }
}
