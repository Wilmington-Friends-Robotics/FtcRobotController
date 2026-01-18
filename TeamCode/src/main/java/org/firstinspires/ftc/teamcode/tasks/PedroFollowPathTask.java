package org.firstinspires.ftc.teamcode.tasks;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.FieldRobot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.config.PedroConfig;
import org.firstinspires.ftc.teamcode.pedro.PedroFollowerFactory;

/**
 * Task that drives a Pedro path until completion or timeout.
 */
public class PedroFollowPathTask implements RobotTask {
    private final PathChain pathChain;
    private final double timeoutSec;

    private Follower follower;

    public PedroFollowPathTask(PathChain pathChain, double timeoutSec) {
        this.pathChain = pathChain;
        this.timeoutSec = timeoutSec;
    }

    public PedroFollowPathTask(PathChain pathChain) {
        this(pathChain, 10.0);
    }

    @Override
    public void start(FieldRobot robot, RobotState state) {
        follower = PedroFollowerFactory.build(robot.getHardwareMap(), robot.getFieldLocalizer(), PedroConfig.getInstance());

        Pose startPose = toPedroPose(state);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.followPath(pathChain);
    }

    @Override
    public boolean update(FieldRobot robot, RobotState state, double elapsedSeconds) {
        if (follower == null) {
            return true;
        }

        follower.update();

        boolean timedOut = elapsedSeconds >= timeoutSec;
        boolean finished = !follower.isBusy();
        return finished || timedOut;
    }

    @Override
    public void stop(FieldRobot robot, RobotState state) {
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    @Override
    public String getName() {
        return String.format("PedroFollowPath(%.1fs)", timeoutSec);
    }

    private static Pose toPedroPose(RobotState state) {
        return new Pose(
            state.getPoseInches().getX(),
            state.getPoseInches().getY(),
            state.getPoseInches().getHeading(),
            PedroCoordinates.INSTANCE
        );
    }
}
