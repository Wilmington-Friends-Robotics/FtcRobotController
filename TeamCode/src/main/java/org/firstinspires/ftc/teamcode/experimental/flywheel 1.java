package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class flywheel extends LinearOpMode {

    private static final AtomicBoolean ejectRunning = new AtomicBoolean(false);


    private CRServo flywheelSpeed;
    private ElapsedTime runtime = new ElapsedTime();

    // best power = 1.0 for 5 seconds
    @Override
    public void eject(double power, int duration) {
        
        power = 2.0;
        duration = 5;

        flywheelSpeed = hardwareMap.get(CRServo.class, "flywheelSpeed");

        waitForStart();
        runtime.reset();
        while (ejectRunning.get() && runtime.seconds < duration ) {
            flywheel.setPower(power);
        }
        flywheel.setPower(0);
    }
}
