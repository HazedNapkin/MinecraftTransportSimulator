package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.Test;

public class AnalyticPathsTest {
    private static final double RADIUS = 4;

    @Test
    public void straightAheadIsJustAStraight() {
        List<AnalyticPaths.Path> dubins = AnalyticPaths.dubins(0, 0, 0, 0, 10, 0, RADIUS);
        assertEquals(10, dubins.get(0).length, 1e-6);
        List<AnalyticPaths.Path> reedsShepp = AnalyticPaths.reedsShepp(0, 0, 0, 0, 10, 0, RADIUS);
        assertEquals(10, reedsShepp.get(0).length, 1e-6);
        assertFalse(reedsShepp.get(0).reverses());
    }

    @Test
    public void straightBehindBacksUp() {
        List<AnalyticPaths.Path> reedsShepp = AnalyticPaths.reedsShepp(0, 0, 0, 0, -6, 0, RADIUS);
        assertEquals(6, reedsShepp.get(0).length, 1e-6);
        assertTrue(reedsShepp.get(0).reverses());
        //Forward only, it has to go right round.
        assertTrue(AnalyticPaths.dubins(0, 0, 0, 0, -6, 0, RADIUS).get(0).length > 6 + 2 * Math.PI * RADIUS * 0.5);
    }

    @Test
    public void aQuarterTurnIsOneArc() {
        //Left from facing +Z at radius 4 ends 4 along and 4 to the left, which is +X, facing +X.
        AnalyticPaths.Path path = AnalyticPaths.dubins(0, 0, 0, 4, 4, 90, RADIUS).get(0);
        assertEquals(RADIUS * Math.PI / 2, path.length, 1e-6);
    }

    @Test
    public void alwaysJoinsAnyTwoPoses() {
        Random random = new Random(3);
        for (int i = 0; i < 2000; ++i) {
            double x0 = random.nextDouble() * 30 - 15;
            double z0 = random.nextDouble() * 30 - 15;
            double yaw0 = random.nextDouble() * 360 - 180;
            double x1 = random.nextDouble() * 30 - 15;
            double z1 = random.nextDouble() * 30 - 15;
            double yaw1 = random.nextDouble() * 360 - 180;
            List<AnalyticPaths.Path> dubins = AnalyticPaths.dubins(x0, z0, yaw0, x1, z1, yaw1, RADIUS);
            List<AnalyticPaths.Path> reedsShepp = AnalyticPaths.reedsShepp(x0, z0, yaw0, x1, z1, yaw1, RADIUS);
            assertFalse("dubins " + i, dubins.isEmpty());
            assertFalse("reeds-shepp " + i, reedsShepp.isEmpty());
            for (AnalyticPaths.Path path : dubins) {
                assertFalse(path.reverses());
            }
            //Being allowed to back up never makes it longer, and is never shorter than as the crow flies.
            assertTrue(reedsShepp.get(0).length <= dubins.get(0).length + 1e-6);
            assertTrue(reedsShepp.get(0).length >= Math.hypot(x1 - x0, z1 - z0) - 1e-6);
            //The quick lengths, which don't drive the paths to check them, are the shortest that check out.
            assertEquals("dubins length " + i, dubins.get(0).length, AnalyticPaths.dubinsLength(x0, z0, yaw0, x1, z1, yaw1, RADIUS), 1e-6);
            assertEquals("reeds-shepp length " + i, reedsShepp.get(0).length, AnalyticPaths.reedsSheppLength(x0, z0, yaw0, x1, z1, yaw1, RADIUS), 1e-6);
        }
    }
}
