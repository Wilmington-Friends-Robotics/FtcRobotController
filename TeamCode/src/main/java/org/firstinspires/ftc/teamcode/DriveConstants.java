package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    // Robot physical constants
    public static final double TRACK_WIDTH = 13.6; // inches between center of right and left wheels
    public static final double WHEEL_BASE = 13.6; // inches between center of front and back wheels

    // GoBilda Pinpoint odometry constants (in millimeters)
    public static final double X_OFFSET_MM = -84.0; // Left of center is positive
    public static final double Y_OFFSET_MM = -168.0; // Forward of center is positive

    // These are example values you need to tune
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF());

    public static double kV = 1.0 / 312.0; // 1.0 / max rpm
    public static double kA = 0;
    public static double kStatic = 0;

    // Feedforward constants, tune these for your robot
    public static double MAX_VEL = 30; // inches per second
    public static double MAX_ACCEL = 30; // inches per second squared
    public static double MAX_ANG_VEL = Math.toRadians(180); // radians per second
    public static double MAX_ANG_ACCEL = Math.toRadians(180); // radians per second squared

    // PID values for trajectory following
    public static double TRANSLATIONAL_PID_P = 8;
    public static double TRANSLATIONAL_PID_I = 0;
    public static double TRANSLATIONAL_PID_D = 0;

    public static double HEADING_PID_P = 8;
    public static double HEADING_PID_I = 0;
    public static double HEADING_PID_D = 0;

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / 312.0; // 32767 / max rpm
    }
} 