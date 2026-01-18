package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.After;
import org.junit.Test;

public class RobotStateTest {

    @After
    public void reset() {
        RobotState.getInstance().setPose(new Pose2d());
    }

    @Test
    public void updateFromPoseAppliesDeadbandsAndStoresHeadingVel() {
        RobotState state = RobotState.getInstance();

        // Values above deadbands so filtering does not zero them
        Pose2d pose = new Pose2d(0.2, 0.2, Math.toRadians(10));
        Pose2d vel = new Pose2d(1.0, 1.0, Math.toRadians(30));

        state.updateFrom(pose, vel);

        Pose2d stored = state.getPoseInches();
        assertEquals(0.2, stored.getX(), 1e-6);
        assertEquals(0.2, stored.getY(), 1e-6);
        assertEquals(Math.toRadians(10), stored.getHeading(), 1e-6);

        Pose2d storedVel = state.getVelocityInches();
        assertEquals(1.0, storedVel.getX(), 1e-6);
        assertEquals(1.0, storedVel.getY(), 1e-6);
        assertEquals(Math.toRadians(30), state.getHeadingVelocityRadPerSec(), 1e-6);

        // Ensure mm conversion matches inches * 25.4
        assertEquals(stored.getX() * 25.4, state.getPoseMillimeters().getX(DistanceUnit.MM), 1e-6);
        assertEquals(stored.getY() * 25.4, state.getPoseMillimeters().getY(DistanceUnit.MM), 1e-6);
    }
}
