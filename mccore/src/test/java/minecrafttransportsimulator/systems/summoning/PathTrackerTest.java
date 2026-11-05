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

    private static SummonRoute plannedReverse(FakeTerrain terrain, SummonRoute.Pose start) {
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.allowReverse = true;
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car()), KINEMATICS, settings, new Point3D(0.5, 0, 0.5), 0);
        planner.setStart(start.x, start.y, start.z, start.yaw);
        planner.finish();
        SummonRoute route = planner.getRoute();
        assertNotNull(route);
        assertTrue(!route.getCusps().isEmpty());
        return route;
    }

    private static void checkReverseDrive(DriveSimulator.Drive drive, SummonRoute route) {
        checkDrive(drive, route);
        //It shifted once for each change of direction, and stopped about where it should each time.
        assertEquals(route.getCusps().size(), drive.shifts);
        assertTrue("overran " + drive.worstOverrun, drive.worstOverrun < 0.5);
    }

    @Test
    public void backsOutOfAnAlleyAndDrivesIn() {
        SummonRoute route = plannedReverse(RoutePlannerTest.alley(), new SummonRoute.Pose(-27.5, 0, 17, -90, 0));
        checkReverseDrive(DriveSimulator.drive(route, KINEMATICS, 0, 0, 0, 0), route);
    }

    @Test
    public void makesAThreePointTurn() {
        SummonRoute route = plannedReverse(RoutePlannerTest.narrowStreet(), new SummonRoute.Pose(-40.5, 0, 14.5, -90, 0));
        checkReverseDrive(DriveSimulator.drive(route, KINEMATICS, 0, 0, 0, 0), route);
    }

    @Test
    public void getsBackOntoTheRouteBackingUp() {
        //Backing up a long curve, then forward to a straight run-in, setting off to the side and turned.
        SummonRoute route = PathSmootherTest.route(new double[] { 6, 0, 1 }, new double[] { 10, 0.15, 1 }, new double[] { 4, 0, 1 }, new double[] { 12, 0 });
        DriveSimulator.Drive drive = DriveSimulator.drive(route, KINEMATICS, 0, 0.6, 8, 6);
        checkReverseDrive(drive, route);
    }
}
