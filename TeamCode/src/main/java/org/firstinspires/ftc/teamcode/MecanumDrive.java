package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.Telemetry;

class MecanumDrive {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Odometry odometry;  // Replace IMU with Odometry
    private double headingOffset = 0.0; // Stores the initial heading offset
    private Telemetry telemetry;  // Add telemetry field

    // Getter methods to access motor positions
    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }

    // Getter methods to access motor powers
    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    public double getBackRightPower() {
        return backRight.getPower();
    }

    // Constructor to initialize all four drive motors and IMU
    public MecanumDrive(HardwareMap hardwareMap, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Telemetry telemetry) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;

        // Set brake mode for all drive motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Odometry
        try {
            odometry = new Odometry(hardwareMap);
            resetHeading(); // Store initial heading
            telemetry.addData("Odometry Status", "Initialized Successfully");
            System.out.println("Mecanum drive and Odometry initialized");
        } catch (Exception e) {
            telemetry.addData("Odometry Status", "Failed to Initialize: %s", e.getMessage());
            System.out.println("Failed to initialize Odometry: " + e.getMessage());
            odometry = null;
        }
        telemetry.update();
    }

    // Reset the heading offset to make current heading the zero heading
    public void resetHeading() {
        if (odometry != null) {
            odometry.resetHeading();
            headingOffset = odometry.getHeading();
            telemetry.addData("Heading Reset", "Success - New Offset: %.2f", headingOffset);
        } else {
            telemetry.addData("Heading Reset", "Failed - Odometry is null");
        }
        telemetry.update();
    }

    // Get the current heading relative to the initial heading
    private double getHeading() {
        if (odometry != null) {
            return odometry.getHeading() - headingOffset;
        }
        telemetry.addData("Heading", "No Data - Odometry is null");
        telemetry.update();
        return 0.0;
    }

    // Method to drive the robot using mecanum wheels with field-centric control
    public void drive(double forward, double strafe, double rotate) {
        // If odometry is not available, fall back to robot-centric control
        if (odometry == null) {
            telemetry.addData("Drive Mode", "Robot-Centric (Odometry Unavailable)");
            telemetry.update();
            driveRobotCentric(forward, strafe, rotate);
            return;
        }

        telemetry.addData("Drive Mode", "Field-Centric");
        telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(getHeading()));
        telemetry.update();

        // Get the robot's heading
        double heading = getHeading();

        // Convert desired field-relative motion to robot-relative motion
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        
        double fieldForward = forward * cos + strafe * sin;
        double fieldStrafe = -forward * sin + strafe * cos;

        // Drive with the converted values
        driveRobotCentric(fieldForward, fieldStrafe, rotate);
    }

    // Robot-centric driving method
    private void driveRobotCentric(double forward, double strafe, double rotate) {
        // Calculate power for each wheel based on forward, strafe, and rotate inputs
        double frontLeftPower = -forward + strafe - rotate;
        double frontRightPower = -forward - strafe + rotate;
        double backLeftPower = -forward - strafe - rotate;
        double backRightPower = -forward + strafe + rotate;

        // Normalize the wheel powers if any power is greater than 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set power to each motor
        frontLeft.setPower(-frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
        System.out.println(String.format("Drive command - FrontLeft: %2.2f, FrontRight: %2.2f, BackLeft: %2.2f, BackRight: %2.2f", 
            frontLeftPower, frontRightPower, backLeftPower, backRightPower));
    }

    // Method to reset encoders and set motors to the desired mode
    public void resetAndRunToPosition() {
        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Method to set target position for all motors and then set mode to RUN_TO_POSITION
    public void setTargetPositionAndRun(int ticks) {
        // Set target position
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        // Set motors to RUN_TO_POSITION mode after setting the target
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Method to check if all motors are busy
    public boolean isBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }
}