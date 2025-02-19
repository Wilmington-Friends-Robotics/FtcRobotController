package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class Odometry {
    private I2cDeviceSynch odometry;
    private double headingOffset = 0.0;
    
    // Position tracking
    private double robotX = 0.0; // X position in mm
    private double robotY = 0.0; // Y position in mm

    public Odometry(HardwareMap hardwareMap) {
        // Initialize odometry driver
        try {
            odometry = hardwareMap.get(I2cDeviceSynch.class, "odometry");
            odometry.setI2cAddress(I2cAddr.create7bit(0x1E)); // Default I2C address for GoBILDA Pinpoint
            odometry.engage();
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
            // Read heading from I2C device
            byte[] response = odometry.read(0x04, 2); // Example register for heading
            double heading = ((response[1] & 0xFF) << 8 | (response[0] & 0xFF)) / 100.0; // Convert to radians
            headingOffset = heading;
        }
    }

    // Get the current heading relative to the initial heading
    public double getHeading() {
        if (odometry != null) {
            // Read heading from I2C device
            byte[] response = odometry.read(0x04, 2); // Example register for heading
            double heading = ((response[1] & 0xFF) << 8 | (response[0] & 0xFF)) / 100.0; // Convert to radians
            return heading - headingOffset;
        }
        return 0.0;
    }

    // Reset position tracking to zero
    public void resetPosition() {
        robotX = 0.0;
        robotY = 0.0;
        if (odometry != null) {
            // Send reset command to I2C device
            odometry.write(0x00, new byte[]{0x01}); // Example reset command
        }
    }

    // Update position tracking
    public void update() {
        if (odometry != null) {
            // Read X position (4 bytes)
            byte[] xResponse = odometry.read(0x08, 4);
            robotX = bytesToFloat(xResponse);

            // Read Y position (4 bytes)
            byte[] yResponse = odometry.read(0x0C, 4);
            robotY = bytesToFloat(yResponse);
        }
    }

    // Convert 4 bytes to float
    private float bytesToFloat(byte[] bytes) {
        int intBits = bytes[3] << 24 | (bytes[2] & 0xFF) << 16 | (bytes[1] & 0xFF) << 8 | (bytes[0] & 0xFF);
        return Float.intBitsToFloat(intBits);
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
        if (odometry != null) {
            byte[] response = odometry.read(0x10, 4); // Example register for parallel encoder
            return bytesToFloat(response);
        }
        return 0.0;
    }

    public double getPerpendicularPosition() {
        if (odometry != null) {
            byte[] response = odometry.read(0x14, 4); // Example register for perpendicular encoder
            return bytesToFloat(response);
        }
        return 0.0;
    }
} 