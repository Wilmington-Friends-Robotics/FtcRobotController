package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.FieldRobot;
import org.firstinspires.ftc.teamcode.RobotState;

/**
 * Drives the mecanum base with fixed powers for a duration.
 */
public class TimedDriveTask implements RobotTask {
    private final double forward;
    private final double strafe;
    private final double rotate;
    private final double durationSec;

    public TimedDriveTask(double forward, double strafe, double rotate, double durationSec) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.durationSec = durationSec;
    }

    @Override
    public void start(FieldRobot robot, RobotState state) {
        robot.drive(0, 0, 0);
    }

    @Override
    public boolean update(FieldRobot robot, RobotState state, double elapsedSeconds) {
        robot.drive(forward, strafe, rotate);
        return elapsedSeconds >= durationSec;
    }

    @Override
    public void stop(FieldRobot robot, RobotState state) {
        robot.drive(0, 0, 0);
    }

    @Override
    public String getName() {
        return String.format("Drive f%.2f s%.2f r%.2f", forward, strafe, rotate);
    }
}
