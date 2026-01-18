package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.FieldRobot;
import org.firstinspires.ftc.teamcode.RobotState;

/**
 * Simple task that waits for a fixed amount of time.
 */
public class WaitTask implements RobotTask {
    private final double durationSec;

    public WaitTask(double durationSec) {
        this.durationSec = durationSec;
    }

    @Override
    public boolean update(FieldRobot robot, RobotState state, double elapsedSeconds) {
        return elapsedSeconds >= durationSec;
    }

    @Override
    public String getName() {
        return String.format("Wait %.1fs", durationSec);
    }
}
