package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldRobot;
import org.firstinspires.ftc.teamcode.RobotState;

import java.util.LinkedList;
import java.util.Queue;

/**
 * Lightweight task scheduler that runs {@link RobotTask}s sequentially.
 */
public class TaskController {
    private final Queue<RobotTask> queue = new LinkedList<>();
    private final ElapsedTime taskTimer = new ElapsedTime();
    private RobotTask currentTask;

    public void addTask(RobotTask task) {
        queue.add(task);
    }

    /**
     * Clears all pending tasks and stops the current one if running.
     */
    public void clear(FieldRobot robot, RobotState state) {
        if (currentTask != null) {
            currentTask.stop(robot, state);
        }
        currentTask = null;
        queue.clear();
    }

    /**
     * Update the controller. Call this once per loop.
     *
     * @return true while there are tasks executing or queued.
     */
    public boolean update(FieldRobot robot, RobotState state) {
        if (currentTask == null && !queue.isEmpty()) {
            advance(robot, state);
        }

        if (currentTask == null) {
            return false;
        }

        double elapsed = taskTimer.seconds();
        boolean finished = currentTask.update(robot, state, elapsed);
        if (finished) {
            currentTask.stop(robot, state);
            advance(robot, state);
        }

        return currentTask != null || !queue.isEmpty();
    }

    public boolean isIdle() {
        return currentTask == null && queue.isEmpty();
    }

    public String getActiveTaskName() {
        return currentTask != null ? currentTask.getName() : "Idle";
    }

    private void advance(FieldRobot robot, RobotState state) {
        currentTask = queue.poll();
        taskTimer.reset();
        if (currentTask != null) {
            currentTask.start(robot, state);
        }
    }
}
