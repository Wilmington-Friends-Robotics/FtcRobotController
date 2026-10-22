package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.FieldLinearOpMode;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedro.PedroPathFactory;

/**
 * Demonstrates running a Pedro path via the task controller.
 */
@Autonomous(name = "Pedro Task Sample", group = "Field")
public class PedroTaskSampleOpMode extends FieldLinearOpMode {
    private final TaskController controller = new TaskController();

    @Override
    protected void onInit() {
        controller.addTask(new PedroFollowPathTask(
            PedroPathFactory.getPath(PedroPathFactory.PathId.SQUARE_24),
            20.0
        ));
        telemetry.addLine("Pedro path task queued. Press start to run square.");
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
