package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

class JoystickController {
    private Gamepad gamepad;
    private MecanumDrive mecanumDrive;
    private boolean halfPowerMode = false;

    // Constructor that takes the gamepad and mecanum drive as inputs
    public JoystickController(Gamepad gamepad, MecanumDrive mecanumDrive) {
        this.gamepad = gamepad;
        this.mecanumDrive = mecanumDrive;
        System.out.println("Joystick controller initialized");
    }

    // Method to toggle power mode
    public void togglePowerMode() {
        halfPowerMode = !halfPowerMode;
    }

    // Get current power mode
    public boolean isHalfPowerMode() {
        return halfPowerMode;
    }

    // Update method to read joystick inputs and control the robot accordingly
    public void update() {
        // Forward/backward movement is controlled by the left stick Y-axis
        double forward = gamepad.left_stick_y;
        // Left/right strafing is controlled by the left stick X-axis
        double strafe = gamepad.left_stick_x;
        // Rotation is controlled by the right stick X-axis
        double rotate = -gamepad.right_stick_x;

        // Apply power scaling if in half power mode
        if (halfPowerMode) {
            forward *= 0.5;
            strafe *= 0.5;
            rotate *= 0.5;
        }

        // Drive the robot based on the scaled joystick inputs
        mecanumDrive.drive(forward, strafe, rotate);
        System.out.println(String.format("Joystick update - Forward: %2.2f, Strafe: %2.2f, Rotate: %2.2f, Mode: %s", 
            forward, strafe, rotate, halfPowerMode ? "Half" : "Full"));
    }
}
