package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Core robot container that instantiates the drivetrain and field localizer for the new task-driven system.
 */
public class FieldRobot {
    private final HardwareMap hardwareMap;
    private final MecanumDrive mecanumDrive;
    private final PinpointFieldLocalizer fieldLocalizer;
    private final RobotState robotState = RobotState.getInstance();

    public FieldRobot(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d());
    }

    public FieldRobot(HardwareMap hardwareMap, Pose2d startPose) {
        this.hardwareMap = hardwareMap;
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right");

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        fieldLocalizer = new PinpointFieldLocalizer(hardwareMap, startPose);
        robotState.setPose(startPose);
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public MecanumDrive getDrive() {
        return mecanumDrive;
    }

    public void drive(double forward, double strafe, double rotate) {
        mecanumDrive.drive(forward, strafe, rotate);
    }

    public PinpointFieldLocalizer getFieldLocalizer() {
        return fieldLocalizer;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public void resetPose(Pose2d pose) {
        fieldLocalizer.resetAndSetPose(pose);
        robotState.setPose(pose);
    }

    public void update() {
        fieldLocalizer.update();
        robotState.updateFrom(fieldLocalizer);
    }
}
