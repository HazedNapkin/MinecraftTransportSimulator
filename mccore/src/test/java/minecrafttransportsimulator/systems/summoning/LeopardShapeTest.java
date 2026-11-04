package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Plans with the shape a real tank pack gives: a Leopard 1, its hull and turret 62 collision boxes that only collide
 * with entities, a gun barrel of small boxes reaching far out in front, and 4 track wheels.  Taken from its summon debug log.
 */
public class LeopardShapeTest {
    static final List<double[]> BOXES = Arrays.asList(
            new double[] { -0.01, 1.5499999999999998, -3.51, 0.1, 0.09999999999999998 },
            new double[] { -1.18, 2.235, -0.36, 0.23, 0.43499999999999994 },
            new double[] { -0.0, 1.3, 2.1, 0.94, 0.36999999999999994 },
            new double[] { -0.01, 2.17, 5.21, 0.12, 0.1200000000000001 },
            new double[] { 1.4, 1.405, 2.78, 0.4, 0.43500000000000005 },
            new double[] { -0.01, 2.17, 2.29, 0.12, 0.1200000000000001 },
            new double[] { -0.0, 0.89, -2.19, 1.23, 0.4 },
            new double[] { -0.02, 2.17, 2.78, 0.12, 0.1200000000000001 },
            new double[] { 1.4, 1.405, -1.55, 0.4, 0.43500000000000005 },
            new double[] { 1.39, 1.405, 0.61, 0.4, 0.43500000000000005 },
            new double[] { -0.23, 2.295, -1.52, 0.9, 0.43499999999999994 },
            new double[] { -0.28, 0.89, 1.83, 0.95, 0.4 },
            new double[] { -0.01, 2.17, 3.5, 0.12, 0.1200000000000001 },
            new double[] { -0.02, 2.17, 4.23, 0.12, 0.1200000000000001 },
            new double[] { -0.01, 1.47, -3.53, 0.17, 0.16999999999999993 },
            new double[] { -1.42, 1.405, 2.79, 0.4, 0.43500000000000005 },
            new double[] { -0.01, 2.17, 2.53, 0.12, 0.1200000000000001 },
            new double[] { -0.0, 0.89, -2.83, 1.23, 0.4 },
            new double[] { 0.26, 0.89, 1.83, 0.95, 0.4 },
            new double[] { -1.17, 2.235, 0.01, 0.23, 0.43499999999999994 },
            new double[] { 1.39, 1.405, -0.1, 0.4, 0.43500000000000005 },
            new double[] { 1.39, 1.405, -3.0, 0.4, 0.43500000000000005 },
            new double[] { 1.39, 1.405, -2.28, 0.4, 0.43500000000000005 },
            new double[] { -0.01, 2.17, 3.25, 0.12, 0.1200000000000001 },
            new double[] { -0.01, 2.17, 3.74, 0.12, 0.1200000000000001 },
            new double[] { -0.01, 2.17, 1.79, 0.12, 0.1200000000000001 },
            new double[] { -1.18, 2.235, 0.4, 0.23, 0.43499999999999994 },
            new double[] { -1.4, 1.405, -2.28, 0.4, 0.43500000000000005 },
            new double[] { -1.42, 1.405, -1.56, 0.4, 0.43500000000000005 },
            new double[] { -0.01, 2.17, 6.42, 0.12, 0.1200000000000001 },
            new double[] { -1.41, 1.405, -3.72, 0.4, 0.43500000000000005 },
            new double[] { -1.41, 1.405, 2.06, 0.4, 0.43500000000000005 },
            new double[] { -1.42, 1.405, -0.83, 0.4, 0.43500000000000005 },
            new double[] { -0.01, 2.17, 2.04, 0.12, 0.1200000000000001 },
            new double[] { -0.0, 2.17, 4.72, 0.12, 0.1200000000000001 },
            new double[] { 0.31, 0.5800000000000001, -0.07, 0.9, 0.08000000000000002 },
            new double[] { -1.41, 1.405, -3.0, 0.4, 0.43500000000000005 },
            new double[] { -0.01, 1.5, -2.02, 0.94, 0.45999999999999996 },
            new double[] { 1.16, 2.235, -0.36, 0.23, 0.43499999999999994 },
            new double[] { -0.01, 2.17, 6.18, 0.12, 0.1200000000000001 },
            new double[] { -0.47, 2.185, 1.27, 0.61, 0.43500000000000005 },
            new double[] { 1.39, 1.405, 1.33, 0.4, 0.43500000000000005 },
            new double[] { -0.33, 0.5800000000000001, -0.07, 0.9, 0.08000000000000002 },
            new double[] { -0.01, 2.17, 4.97, 0.12, 0.1200000000000001 },
            new double[] { -0.01, 2.17, 5.45, 0.12, 0.1200000000000001 },
            new double[] { 1.15, 2.235, 0.4, 0.23, 0.43499999999999994 },
            new double[] { -0.01, 1.5, -3.33, 0.94, 0.45999999999999996 },
            new double[] { -0.01, 2.17, 3.01, 0.12, 0.1200000000000001 },
            new double[] { -0.02, 2.17, 3.99, 0.12, 0.1200000000000001 },
            new double[] { 0.19, 2.295, -1.52, 0.9, 0.43499999999999994 },
            new double[] { -0.01, 1.3, 2.11, 0.94, 0.36999999999999994 },
            new double[] { 1.16, 2.235, 0.01, 0.23, 0.43499999999999994 },
            new double[] { -0.01, 2.17, 5.93, 0.12, 0.1200000000000001 },
            new double[] { -1.41, 1.405, 0.61, 0.4, 0.43500000000000005 },
            new double[] { -0.0, 2.17, 4.48, 0.12, 0.1200000000000001 },
            new double[] { -0.02, 2.17, 5.7, 0.12, 0.1200000000000001 },
            new double[] { 1.39, 1.405, 2.05, 0.4, 0.43500000000000005 },
            new double[] { 0.45, 2.185, 1.27, 0.61, 0.43500000000000005 },
            new double[] { -1.41, 1.405, 1.33, 0.4, 0.43500000000000005 },
            new double[] { 1.4, 1.405, -0.83, 0.4, 0.43500000000000005 },
            new double[] { 1.39, 1.405, -3.72, 0.4, 0.43500000000000005 },
            new double[] { -1.42, 1.405, -0.11, 0.4, 0.43500000000000005 });
    static final List<double[]> WHEELS = Arrays.asList(
            new double[] { -1.51, 0.075, -2.74, 0.5, 0.075 },
            new double[] { 1.51, 0.075, -2.74, 0.5, 0.075 },
            new double[] { -1.51, 0.075, 1.76, 0.5, 0.075 },
            new double[] { 1.51, 0.075, 1.76, 0.5, 0.075 });

    private static final Point3D PLAYER = new Point3D(0.5, 0, 0.5);

    @Test
    public void bodyLengthLeavesOutTheBarrel() {
        VehicleShape shape = VehicleShape.of(BOXES, WHEELS);
        assertEquals(10.81, shape.front + shape.back, 0.01);
        assertEquals(7.46, shape.bodyLength, 0.01);
    }

    @Test
    public void plansAStopOffThePlayer() {
        VehicleShape shape = VehicleShape.of(BOXES, WHEELS);
        VehicleKinematics kinematics = new VehicleKinematics(1, 0.35, 0, false);
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.turnRadius = 1.25 * kinematics.getTurnRadius(3);
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(FakeTerrain.flat(80)), shape), kinematics, settings, PLAYER, 0);
        long start = System.nanoTime();
        int ticks = 1;
        while (!planner.step(10_000_000)) {
            ++ticks;
        }
        System.out.println(String.format("LEOPARD plan %.0f ms over %d ticks: %s", (System.nanoTime() - start) / 1e6, ticks, planner.getTimings()));
        SummonRoute route = planner.getRoute();
        assertNotNull(route);
        //The player isn't between the tracks, 1.5 either side of the origin.
        assertTrue(Math.hypot(route.goal.x - PLAYER.x, route.goal.z - PLAYER.z) >= 1.5 + 1.2 - 0.1);
    }
}
