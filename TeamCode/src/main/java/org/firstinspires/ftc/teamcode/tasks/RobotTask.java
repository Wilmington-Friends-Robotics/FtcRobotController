package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.FieldRobot;
import org.firstinspires.ftc.teamcode.RobotState;

/**
 * Basic contract for tasks used in the state-machine controller.
 */
public interface RobotTask {
    /**
     * Called once before the task begins executing.
     */
    default void start(FieldRobot robot, RobotState state) {
        // Optional override
    }

    /**
     * Called every loop; return true when the task has finished its work.
     *
     * @param robot the shared robot container
     * @param state the latest RobotState snapshot
     * @param elapsedSeconds time in seconds since {@link #start(FieldRobot, RobotState)} was invoked
     * @return true when the task is complete
     */
    boolean update(FieldRobot robot, RobotState state, double elapsedSeconds);

    /**
     * Called once when the task finishes or is aborted.
     */
    default void stop(FieldRobot robot, RobotState state) {
        // Optional override
    }

    /**
     * Human-readable task name for telemetry/debugging.
     */
    default String getName() {
        return getClass().getSimpleName();
    }
}
