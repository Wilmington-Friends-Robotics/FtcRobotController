package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Touch Sensor Test", group = "Test")
public class LimitSwitchTest extends LinearOpMode {
    private TouchSensor touchSensor;
    
    @Override
    public void runOpMode() {
        // Initialize the touch sensor
        touchSensor = hardwareMap.get(TouchSensor.class, "limit_switch");
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press the touch sensor to test");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Get and display the sensor state
            boolean isPressed = touchSensor.isPressed();
            
            telemetry.addData("Sensor State", isPressed ? "PRESSED" : "NOT PRESSED");
            telemetry.update();
            
            sleep(50);  // Small delay to prevent overwhelming the telemetry
        }
    }
} 