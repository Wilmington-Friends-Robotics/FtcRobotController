package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "IntoTheDeepRightAutonomous", group = "Competition")
public class IntoTheDeepRightAutonomous extends LinearOpMode {
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

            claw.elbowForward();
            claw.moveToMedium();
            
            // Drive forward
            mecanumDrive.drive(0.45, 0, -0.01);
            
            // Keep running for 0.7 seconds
            while (opModeIsActive() && runtime.seconds() < 0.95) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }
            
            // Stop the robot
            mecanumDrive.drive(0, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 0.12) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }
            
            claw.moveToLow();

            while (opModeIsActive() && runtime.seconds() < 1.5) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            claw.openClaw();
            claw.elbowUp();
            claw.moveToGround();

            mecanumDrive.drive(-0.45, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 1.8) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(0, -0.9, 0);

            while (opModeIsActive() && runtime.seconds() < 2.5) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(0.7, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 3.3) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(0, -0.6, 0);

            while (opModeIsActive() && runtime.seconds() < 3.7) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(-0.6, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 4.7) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }
            
            mecanumDrive.drive(0.6, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 5.7) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(0, -0.6, 0);

            while (opModeIsActive() && runtime.seconds() < 6) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(-0.6, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 6) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(0.6, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 6) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(0, -0.6, 0);

            while (opModeIsActive() && runtime.seconds() < 6) {
                telemetry.addData("Time", "%.2f", runtime.seconds());
                telemetry.addData("Slide Position", claw.getCurrentPosition());
                telemetry.update();
            }

            mecanumDrive.drive(-0.6, 0, 0);

            while (opModeIsActive() && runtime.seconds() < 6) {
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