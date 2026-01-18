package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FieldLinearOpMode;
import org.firstinspires.ftc.teamcode.RobotState;

@TeleOp(name = "Task Controller Sample", group = "Field")
public class TaskControllerSampleOpMode extends FieldLinearOpMode {
    private final TaskController controller = new TaskController();

    @Override
    protected void onInit() {
        controller.addTask(new WaitTask(1.0));
        controller.addTask(new TimedDriveTask(0.4, 0.0, 0.0, 1.5));
        controller.addTask(new WaitTask(0.5));
        controller.addTask(new TimedDriveTask(0.0, 0.4, 0.0, 1.5));
        controller.addTask(new WaitTask(0.5));
        controller.addTask(new TimedDriveTask(0.0, 0.0, 0.4, 1.0));

        telemetry.addLine("Task queue loaded. Press start to execute sequence.");
        telemetry.update();
    }

    @Override
    protected void onLoop() {
        RobotState state = robot.getRobotState();
        boolean running = controller.update(robot, state);

        telemetry.addData("Active Task", controller.getActiveTaskName());
        telemetry.addData("Pose (in)", "x: %.2f y: %.2f h: %.1f",
            state.getPoseInches().getX(),
            state.getPoseInches().getY(),
            Math.toDegrees(state.getPoseInches().getHeading()));
        telemetry.addData("Running", running);
        telemetry.update();

        if (!running) {
            requestOpModeStop();
        }
    }
}
