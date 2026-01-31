package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="flywheel_1", group="Linear Opmode")
public class flywheel_1 extends LinearOpMode {

    private static final AtomicBoolean ejectRunning = new AtomicBoolean(false);


    private CRServo flywheelSpeed;
    private ElapsedTime runtime = new ElapsedTime();

    // best power = 2.0 for 5 seconds
    @Override
    public void runOpMode() {
        flywheelSpeed = hardwareMap.get(CRServo.class, "flywheelSpeed");
        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
    }

    public void eject(double power, int duration) {    
        ejectRunning.set(true);
        runtime.reset();

        while (ejectRunning.get() && opModeIsActive() && runtime.seconds() < duration ) {
            flywheelSpeed.setPower(power);
        }
        flywheelSpeed.setPower(0.0);
        ejectRunning.set(false);
    }
}
