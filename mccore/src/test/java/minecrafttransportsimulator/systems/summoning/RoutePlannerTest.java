package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

public class RoutePlannerTest {
    private static final Point3D PLAYER = new Point3D(0.5, 0, 0.5);
    private static final VehicleKinematics KINEMATICS = new VehicleKinematics(2.6, 0.35, 0, false);

    private static RoutePlanner plan(FakeTerrain terrain, Point3D player, SummonRoute.Pose start) {
        return plan(terrain, player, start, false);
    }

    private static RoutePlanner plan(FakeTerrain terrain, Point3D player, SummonRoute.Pose start, boolean allowReverse) {
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.allowReverse = allowReverse;
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car()), KINEMATICS, settings, player, 0);
        if (start != null) {
            planner.setStart(start.x, start.y, start.z, start.yaw);
        }
        planner.finish();
        return planner;
    }

    /**Checks the route is one the vehicle can drive, and returns it.**/
    private static SummonRoute checkDrivable(FakeTerrain terrain, RoutePlanner planner) {
        SummonRoute route = planner.getRoute();
        assertNotNull(route);
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        PoseChecker checker = new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car());
        for (int i = 0; i < route.poses.size(); ++i) {
            SummonRoute.Pose pose = route.poses.get(i);
            assertTrue("fits at pose " + i, checker.canDrive(pose.x, pose.y, pose.z, pose.yaw));
            if (i > 0) {
                SummonRoute.Pose last = route.poses.get(i - 1);
                double spacing = Math.hypot(pose.x - last.x, pose.z - last.z);
                //Smoothing evens out the poses a little, so they may be a touch further apart than the search put them.
                assertTrue("spacing at pose " + i, spacing > 0.1 && spacing <= HybridSearch.SAMPLE_SPACING * 1.2);
                double turn = Math.toRadians(Math.abs(VehicleSummonFlight.clampAngle(pose.yaw - last.yaw)));
                //Points are a chord apart, a touch shorter than the arc between them.
                assertTrue("turn at pose " + i, turn / spacing <= 1.01 / settings.turnRadius);
                assertTrue("step at pose " + i, pose.y - last.y <= settings.maxUp && last.y - pose.y <= settings.maxDown);
            }
        }
        //It ends exactly where it pulls up, having driven straight in.
        SummonRoute.Pose end = route.getEnd();
        assertEquals(route.goal.x, end.x, 1e-9);
        assertEquals(route.goal.z, end.z, 1e-9);
        assertEquals(route.goal.yaw, end.yaw, 1e-9);
        //The last 2 blocks of the run-in are always held straight; the start of it eases out of the last turn.
        SummonRoute.Pose runInStart = route.poses.get(route.poses.size() - 1 - 4);
        assertEquals(end.yaw, runInStart.yaw, 1e-9);
        if (route.placed) {
            SummonRoute.Pose start = route.getStart();
            assertTrue(checker.canPark(start.x, start.y, start.z, start.yaw));
            assertTrue(route.getLength() >= settings.summonDistance * 0.6);
            assertTrue(Math.hypot(start.x - PLAYER.x, start.z - PLAYER.z) >= settings.summonDistance / 2);
        }
        return route;
    }

    @Test
    public void arcsRetraceThemselves() {
        for (double curvature : new double[] { 0, 0.25, -0.25 }) {
            double[] back = HybridSearch.arc(3, 4, 30, curvature, -1.5);
            double[] forward = HybridSearch.arc(back[0], back[1], back[2], curvature, 1.5);
            assertEquals(3, forward[0], 1e-9);
            assertEquals(4, forward[1], 1e-9);
            assertEquals(30, forward[2], 1e-9);
        }
        //A quarter turn left at radius 4 from facing +Z ends 4 along and 4 to the left, which is +X, facing +X.
        double[] quarter = HybridSearch.arc(0, 0, 0, 0.25, 4 * Math.PI / 2);
        assertEquals(4, quarter[0], 1e-9);
        assertEquals(4, quarter[1], 1e-9);
        assertEquals(90, quarter[2], 1e-9);
    }

    @Test
    public void drivesInOnFlatGround() {
        FakeTerrain terrain = FakeTerrain.flat(60);
        RoutePlanner planner = plan(terrain, PLAYER, null);
        SummonRoute route = checkDrivable(terrain, planner);
        assertTrue(route.placed);
        assertNotNull(planner.getSpawnTargets());
    }

    @Test
    public void drivesInFromTheSummonDistanceInTheOpen() {
        //Nowhere to hide, so nothing should pull it nearer than the summon distance.
        FakeTerrain terrain = FakeTerrain.flat(60);
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, null));
        assertTrue(route.placed);
        assertEquals(new RoutePlanner.Settings().summonDistance, route.getLength(), 4);
    }

    @Test
    public void findsTheWayOutOfAWalledYard() {
        //Walls 12 out all round the player, with one gap on the +X side.
        FakeTerrain terrain = FakeTerrain.flat(60).fill(-12, 0, -12, 12, 2, -12).fill(-12, 0, 12, 12, 2, 12).fill(-12, 0, -12, -12, 2, 12).fill(12, 0, -12, 12, 2, -4).fill(12, 0, 4, 12, 2, 12);
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, null));
        //It starts outside the yard.
        SummonRoute.Pose start = route.getStart();
        assertTrue(Math.abs(start.x) > 12 || Math.abs(start.z) > 12);
    }

    @Test
    public void drivesUpTheHillToAPlayerOnTop() {
        //A hill stepping up a block every 3 blocks in towards the player, 4 steps high, with them on top.
        FakeTerrain terrain = FakeTerrain.flat(60);
        for (int step = 0; step < 4; ++step) {
            int edge = 16 - 3 * step;
            terrain.fill(-edge, step, -edge, edge, step, edge);
        }
        Point3D player = new Point3D(0.5, 4, 0.5);
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car()), KINEMATICS, new RoutePlanner.Settings(), player, 0);
        planner.finish();
        SummonRoute route = planner.getRoute();
        assertNotNull(route);
        assertEquals(4, route.getEnd().y);
        assertTrue(route.getStart().y < route.getEnd().y);
    }

    @Test
    public void setsOffOutOfSight() {
        //A tall wall across the player's view, 14 blocks out, which hides what's behind it.
        FakeTerrain terrain = FakeTerrain.flat(60).fill(-10, 0, 14, 10, 6, 15);
        RoutePlanner planner = plan(terrain, PLAYER, null);
        SummonRoute route = checkDrivable(terrain, planner);
        SummonRoute.Pose start = route.getStart();
        PoseChecker checker = new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car());
        assertFalse(SpawnSelector.isVisible(checker, PLAYER, start.x, start.y, start.z));
    }

    @Test
    public void startsFromWhereANearbyVehicleIs() {
        FakeTerrain terrain = FakeTerrain.flat(60);
        SummonRoute.Pose start = new SummonRoute.Pose(-14.5, 0, 16.5, 90, 0);
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, start));
        assertFalse(route.placed);
        assertTrue(Math.hypot(route.getStart().x - start.x, route.getStart().z - start.z) <= 1);
    }

    /**A dead-end alley 4 blocks wide running out along -x from the open ground at x = -14, closed at x = -31.**/
    private static FakeTerrain alley() {
        return FakeTerrain.flat(60).fill(-31, 0, 14, -14, 2, 14).fill(-31, 0, 19, -14, 2, 19).fill(-31, 0, 14, -31, 2, 19);
    }

    /**A dead-end street 7 blocks wide, too narrow to turn round in, running out along -x from x = -10 to x = -51.**/
    private static FakeTerrain narrowStreet() {
        return FakeTerrain.flat(60).fill(-51, 0, 10, -10, 2, 10).fill(-51, 0, 18, -10, 2, 18).fill(-51, 0, 10, -51, 2, 18);
    }

    private static void checkDirections(SummonRoute route) {
        int maxCusps = new HybridSearch.Settings().maxCusps;
        assertTrue("too many changes of direction", route.getCusps().size() <= maxCusps);
        //It always drives the run-in forward, so it pulls up facing the right way.
        assertFalse(route.getEnd().reverse);
        assertFalse(route.poses.get(route.poses.size() - 2).reverse);
    }

    @Test
    public void cantLeaveAnAlleyNoseInWithoutReversing() {
        FakeTerrain terrain = alley();
        SummonRoute.Pose start = new SummonRoute.Pose(-27.5, 0, 17, -90, 0);
        assertNull(plan(terrain, PLAYER, start, false).getRoute());
    }

    @Test
    public void backsOutOfAnAlleyItsNosedInto() {
        FakeTerrain terrain = alley();
        SummonRoute.Pose start = new SummonRoute.Pose(-27.5, 0, 17, -90, 0);
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, start, true));
        checkDirections(route);
        assertTrue(route.getStart().reverse);
        assertFalse(route.getCusps().isEmpty());
        assertTrue(Math.hypot(route.getStart().x - start.x, route.getStart().z - start.z) <= 1);
    }

    @Test
    public void turnsRoundInAStreetTooNarrowToUTurn() {
        FakeTerrain terrain = narrowStreet();
        SummonRoute.Pose start = new SummonRoute.Pose(-40.5, 0, 14.5, -90, 0);
        assertNull(plan(terrain, PLAYER, start, false).getRoute());
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, start, true));
        checkDirections(route);
        assertFalse(route.getCusps().isEmpty());
        //It turns round where it is rather than backing all the way out.
        assertTrue("backed " + route.getReverseLength(), route.getReverseLength() < 12);
    }

    @Test
    public void doesntReverseWhenItHasRoom() {
        FakeTerrain terrain = FakeTerrain.flat(60);
        SummonRoute.Pose start = new SummonRoute.Pose(-14.5, 0, 16.5, 90, 0);
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, start, true));
        assertTrue(route.getCusps().isEmpty());
        assertFalse(route.getStart().reverse);
    }

    @Test
    public void placedVehiclesNeverReverse() {
        FakeTerrain terrain = narrowStreet();
        SummonRoute route = checkDrivable(terrain, plan(terrain, PLAYER, null, true));
        assertTrue(route.getCusps().isEmpty());
        assertEquals(0, route.getReverseLength(), 0);
    }

    @Test
    public void noRouteOutOfASealedYard() {
        FakeTerrain terrain = FakeTerrain.flat(60).fill(-12, 0, -12, 12, 2, -12).fill(-12, 0, 12, 12, 2, 12).fill(-12, 0, -12, -12, 2, 12).fill(12, 0, -12, 12, 2, 12);
        RoutePlanner planner = plan(terrain, PLAYER, null);
        assertNull(planner.getRoute());
        //There's still room by the player, to bring it straight there.
        assertFalse(planner.getGoals().isEmpty());
    }
}
