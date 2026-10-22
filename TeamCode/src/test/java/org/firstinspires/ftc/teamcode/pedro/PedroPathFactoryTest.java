package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.PedroConfig;
import org.junit.Test;

public class PedroPathFactoryTest {
    private static final double EPS = 1e-6;

    @Test
    public void buildsAllPathsWithoutNulls() {
        PedroConfig config = PedroConfig.getInstance();
        for (PedroPathFactory.PathId id : PedroPathFactory.PathId.values()) {
            PathChain chain = PedroPathFactory.getPath(id, config);
            assertNotNull("PathChain for " + id + " is null", chain);
            assertTrue("PathChain for " + id + " should contain at least one path", chain.size() > 0);
            assertNotNull("End pose should be defined for " + id, chain.endPose());
        }
    }

    @Test
    public void forward24EndsAtExpectedPose() {
        PathChain chain = PedroPathFactory.getPath(PedroPathFactory.PathId.FORWARD_24);
        Pose end = chain.endPose();
        assertEquals(24.0, end.getX(), EPS);
        assertEquals(0.0, end.getY(), EPS);
    }

    @Test
    public void strafeRight24EndsAtExpectedPose() {
        PathChain chain = PedroPathFactory.getPath(PedroPathFactory.PathId.STRAFE_RIGHT_24);
        Pose end = chain.endPose();
        assertEquals(0.0, end.getX(), EPS);
        assertEquals(-24.0, end.getY(), EPS);
    }

    @Test
    public void squareEndsBackAtOrigin() {
        PathChain chain = PedroPathFactory.getPath(PedroPathFactory.PathId.SQUARE_24);
        Pose end = chain.endPose();
        assertEquals(0.0, end.getX(), EPS);
        assertEquals(0.0, end.getY(), EPS);
        assertEquals(4, chain.size());
    }
}
