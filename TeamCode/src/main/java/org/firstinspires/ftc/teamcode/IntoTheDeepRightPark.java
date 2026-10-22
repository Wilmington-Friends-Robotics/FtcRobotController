package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "IntoTheDeepRightPark", group = "Competition")
public class IntoTheDeepRightPark extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private Claw claw;
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        // Initialize drive motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        
        // Initialize drive system
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        
        // Initialize claw system
        claw = new Claw(hardwareMap);
        
        // Set initial claw position
        claw.closeClaw();
        claw.elbowUp();
        claw.wristUp();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Claw", "Closed, Elbow Up, Wrist Up");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Reset the timer
            runtime.reset();

            mecanumDrive.drive(0, -0.5, 0);

            while (opModeIsActive() && runtime.seconds() < 1.5) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            // Stop the robot
            mecanumDrive.drive(0, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 30) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }
        }
    }
} 