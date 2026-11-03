package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

public class PathTrackerTest {
    private static final VehicleKinematics KINEMATICS = new VehicleKinematics(2.6, 0.35, 0, false);

    private static SummonRoute plannedRoute() {
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(FakeTerrain.flat(60).fill(-10, 0, 14, 10, 6, 15)), ArrivalPlannerTest.car()), KINEMATICS, new RoutePlanner.Settings(), new Point3D(0.5, 0, 0.5), 0);
        planner.finish();
        assertNotNull(planner.getRoute());
        return planner.getRoute();
    }

    private static void checkDrive(DriveSimulator.Drive drive, SummonRoute route) {
        assertTrue(drive.arrived);
        assertEquals(0, drive.getEndError(route), 0.35);
        assertEquals(0, drive.getYawError(route), 3);
        assertTrue("strayed " + drive.worstCrossTrack, drive.worstCrossTrack < 0.35);
    }

    @Test
    public void followsAPlannedRoute() {
        SummonRoute route = plannedRoute();
        checkDrive(DriveSimulator.drive(route, KINEMATICS, 0, 0, 0, 0), route);
    }

    @Test
    public void followsATightTurn() {
        SummonRoute route = PathSmootherTest.route(new double[] { 12, 0 }, new double[] { 4 * Math.PI / 2, 1 / 4D }, new double[] { 8, 0 });
        checkDrive(DriveSimulator.drive(route, KINEMATICS, 0, 0, 0, 0), route);
    }

    @Test
    public void getsBackOntoTheRoute() {
        SummonRoute route = plannedRoute();
        checkDrive(DriveSimulator.drive(route, KINEMATICS, 0, 0.8, 12, 8), route);
    }

    @Test
    public void setsOffAtSpeed() {
        SummonRoute route = plannedRoute();
        checkDrive(DriveSimulator.drive(route, KINEMATICS, Double.MAX_VALUE, 0, 0, 0), route);
    }
}
