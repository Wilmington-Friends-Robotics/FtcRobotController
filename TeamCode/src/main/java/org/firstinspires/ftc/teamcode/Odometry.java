package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobildaodometry.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry {
    private GoBildaPinpointDriver odometry;
    private double headingOffset = 0.0;
    
    // Position tracking
    private double robotX = 0.0; // X position in mm
    private double robotY = 0.0; // Y position in mm

    public Odometry(HardwareMap hardwareMap) {
        // Initialize odometry driver
        try {
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
            resetHeading();
            System.out.println("Odometry initialized");
        } catch (Exception e) {
            System.out.println("Failed to initialize odometry: " + e.getMessage());
            odometry = null;
        }
        
        // Initialize position tracking
        resetPosition();
    }

    // Reset the heading offset to make current heading the zero heading
    public void resetHeading() {
        if (odometry != null) {
            headingOffset = odometry.getHeading(AngleUnit.RADIANS);
        }
    }

    // Get the current heading relative to the initial heading
    public double getHeading() {
        if (odometry != null) {
            return odometry.getHeading(AngleUnit.RADIANS) - headingOffset;
        }
        return 0.0;
    }

    // Reset position tracking to zero
    public void resetPosition() {
        robotX = 0.0;
        robotY = 0.0;
        if (odometry != null) {
            odometry.resetPosition();
        }
    }

    // Update position tracking
    public void update() {
        if (odometry != null) {
            // Get position and heading from odometry
            robotX = odometry.getX();
            robotY = odometry.getY();
        }
    }

    // Get current X position in mm
    public double getX() {
        return robotX;
    }

    // Get current Y position in mm
    public double getY() {
        return robotY;
    }

    // Get raw encoder values for debugging
    public double getParallelPosition() {
        return odometry != null ? odometry.getParallelPosition() : 0.0;
    }

    public double getPerpendicularPosition() {
        return odometry != null ? odometry.getPerpendicularPosition() : 0.0;
    }
} 