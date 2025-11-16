package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

public class PedroConfigTest {

    @After
    public void reset() {
        PedroConfig.resetForTesting();
    }

    @Test
    public void pedroConfigLoadsAndCachesValues() {
        PedroConfig config = PedroConfig.getInstance();
        assertNotNull(config);
        assertSame("PedroConfig should memoize the instance",
                config, PedroConfig.getInstance());

        PedroConfig.Deadwheel deadwheel = config.getDeadwheel();
        assertEquals(32.0, deadwheel.getDiameterMm(), 1e-6);
        assertEquals(2000, deadwheel.getEncoderCpr());
        assertTrue(deadwheel.getTicksPerInch() > 0);

        PedroConfig.Pod forwardPod = config.getForwardPod();
        assertTrue(forwardPod.getLongitudinalOffsetInForwardPositive() < 0);

        PedroConfig.Pod lateralPod = config.getLateralPod();
        assertTrue(lateralPod.getLateralOffsetInLeftPositive() < 1);

        PedroConfig.Pinpoint pinpoint = config.getPinpoint();
        assertEquals("odometry", pinpoint.getI2cName());

        PedroConfig.PedroDefaults defaults = config.getPedroDefaults();
        assertTrue(defaults.getMaxVelocityIps() > 0);
    }
}
