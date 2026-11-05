package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Collections;
import java.util.List;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

/**Vehicles that skid steer turning on the spot: where they do, where they mustn't, and driving it.**/
public class PivotTest {
    private static final Point3D PLAYER = new Point3D(0.5, 0, 0.5);
    private static final VehicleKinematics KINEMATICS = new VehicleKinematics(2.6, 0.35, 0, false);

    /**Plans for a car-sized vehicle that may turn on the spot but not back up, as vehicles that always skid steer.**/
    private static RoutePlanner plan(FakeTerrain terrain, SummonRoute.Pose start, boolean allowPivot, double turnRadius) {
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.allowPivot = allowPivot;
        settings.turnRadius = turnRadius;
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car()), KINEMATICS, settings, PLAYER, 0);
        if (start != null) {
            planner.setStart(start.x, start.y, start.z, start.yaw);
        }
        planner.finish();
        return planner;
    }

    /**The indices of the poses turned on the spot to.**/
    private static int countPivots(SummonRoute route) {
        int pivots = 0;
        for (SummonRoute.Pose pose : route.poses) {
            pivots += pose.pivot ? 1 : 0;
        }
        return pivots;
    }

    /**Checks the vehicle stays put through each turn on the spot, and fits at every pose.**/
    private static void checkPivots(FakeTerrain terrain, SummonRoute route) {
        PoseChecker checker = new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car());
        for (int i = 0; i < route.poses.size(); ++i) {
            SummonRoute.Pose pose = route.poses.get(i);
            assertTrue("fits at pose " + i, checker.canDrive(pose.x, pose.y, pose.z, pose.yaw));
            if (pose.pivot) {
                SummonRoute.Pose last = route.poses.get(i - 1);
                assertEquals(last.x, pose.x, 1e-9);
                assertEquals(last.z, pose.z, 1e-9);
                assertTrue("pivot step " + i, Math.abs(VehicleSummonFlight.clampAngle(pose.yaw - last.yaw)) < 20);
            }
        }
        assertEquals(0, route.getReverseLength(), 0);
        //It still pulls up exactly, having driven straight in.
        assertEquals(route.goal.x, route.getEnd().x, 1e-9);
        assertEquals(route.goal.z, route.getEnd().z, 1e-9);
        assertEquals(route.goal.yaw, route.getEnd().yaw, 1e-9);
    }

    /**A bay 7 blocks wide, too narrow to turn round in at a radius of 4 but wide enough to turn on the spot in, closed at x = -31.**/
    private static FakeTerrain bay() {
        return FakeTerrain.flat(60).fill(-31, 0, 13, -14, 2, 13).fill(-31, 0, 21, -14, 2, 21).fill(-31, 0, 13, -31, 2, 21);
    }

    @Test
    public void turnsOnTheSpotToFaceTheWayOut() {
        SummonRoute.Pose start = new SummonRoute.Pose(-26.5, 0, 17.5, -90, 0);
        //Without turning on the spot or backing up, it can't get out.
        assertNull(plan(bay(), start, false, 4).getRoute());
        SummonRoute route = plan(bay(), start, true, 4).getRoute();
        assertNotNull(route);
        checkPivots(bay(), route);
        //It turns first, right where it is, then drives out.
        assertTrue(route.poses.get(1).pivot);
        assertTrue("turned " + route.getPivotDegrees(), route.getPivotDegrees() > 90);
        assertEquals(start.x, route.getStart().x, 1e-9);
        assertEquals(start.z, route.getStart().z, 1e-9);
    }

    @Test
    public void wontTurnOnTheSpotWithoutRoom() {
        //The car fits in a bay 3 wide, but its corners would sweep through the sides turning.
        FakeTerrain terrain = FakeTerrain.flat(60).fill(-31, 0, 14, -14, 2, 14).fill(-31, 0, 18, -14, 2, 18).fill(-31, 0, 14, -31, 2, 18);
        assertNull(plan(terrain, new SummonRoute.Pose(-26.5, 0, 16.5, -90, 0), true, 4).getRoute());
    }

    @Test
    public void checksTheCircleItSweepsAndThePlayer() {
        FakeTerrain terrain = FakeTerrain.flat(60).fill(10, 0, -10, 10, 2, 10);
        PoseChecker checker = new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car());
        HybridSearch.Settings settings = new HybridSearch.Settings();
        settings.allowPivot = true;
        HybridSearch search = new HybridSearch(checker, KINEMATICS, settings, PLAYER, null, Collections.emptyList(), Collections.emptyList(), false);
        //In the open, well away from the player.
        assertTrue(search.canPivot(-10.5, 0, 0.5, 0, 180));
        //Its corners would sweep through the wall, though it fits facing either way along it.
        assertTrue(new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car()).canDrive(8.3, 0, 0.5, 0));
        assertFalse(search.canPivot(8.3, 0, 0.5, 0, 180));
        //Close enough to the player that it'd sweep through them, or too near for comfort.
        assertFalse(search.canPivot(2.5, 0, 0.5, 0, 90));
        assertFalse(search.canPivot(PLAYER.x, 0, PLAYER.z + ArrivalPlannerTest.car().radius() + 1, 0, 90));
    }

    /**
     * Searches for a route from where the vehicle is, facing into the bay, to pull up at its mouth facing out, with
     * the player well away.  The run-in starts deep in the bay, where it can't turn round without turning on the spot.
     */
    private static SummonRoute searchOutOfBay(boolean allowPivot) {
        FakeTerrain terrain = bay();
        Point3D player = new Point3D(20.5, 0, 0.5);
        PoseChecker checker = new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car());
        ArrivalPlanner.Goal goal = new ArrivalPlanner.Goal(-18.5, 0, 17.5, 90, 4, 0);
        FlowField.Settings fieldSettings = new FlowField.Settings();
        fieldSettings.clearance = checker.shape.side - 0.5;
        FlowField field = new FlowField(checker.terrain, fieldSettings, 0, 17);
        field.addSource(-23, 0, 17, 4);
        field.finish();
        HybridSearch.Settings settings = new HybridSearch.Settings();
        settings.allowPivot = allowPivot;
        HybridSearch search = new HybridSearch(checker, KINEMATICS, settings, player, field, Collections.singletonList(goal), Collections.singletonList(new HybridSearch.Start(0.5, 0, 17.5, -90, 0)), false);
        search.finish();
        return search.getRoute();
    }

    @Test
    public void turnsOnTheSpotToLineUpWithTheRunIn() {
        assertNull(searchOutOfBay(false));
        SummonRoute route = searchOutOfBay(true);
        assertNotNull(route);
        checkPivots(bay(), route);
        //It drives into the bay, turns on the spot where the run-in starts, and drives the run-in out.
        int lastPivot = -1;
        for (int i = 0; i < route.poses.size(); ++i) {
            if (route.poses.get(i).pivot) {
                lastPivot = i;
            }
        }
        assertEquals(route.poses.size() - 1 - 8, lastPivot);
        assertEquals(route.goal.yaw, route.poses.get(lastPivot).yaw, 1e-9);
        assertTrue("turned " + route.getPivotDegrees(), route.getPivotDegrees() > 150);
    }

    @Test
    public void neverTurnsOnTheSpotInTheOpen() {
        FakeTerrain terrain = FakeTerrain.flat(60);
        for (double yaw = -180; yaw < 180; yaw += 45) {
            SummonRoute route = plan(terrain, new SummonRoute.Pose(-14.5, 0, 16.5, yaw, 0), true, 4).getRoute();
            assertNotNull("from yaw " + yaw, route);
            assertEquals("pivots from yaw " + yaw, 0, countPivots(route));
        }
        SummonRoute placed = plan(terrain, null, true, 4).getRoute();
        assertNotNull(placed);
        assertEquals(0, countPivots(placed));
    }

    @Test
    public void legsSplitAtTurnsOnTheSpot() {
        SummonRoute route = plan(bay(), new SummonRoute.Pose(-26.5, 0, 17.5, -90, 0), true, 4).getRoute();
        List<int[]> legs = route.getLegs();
        assertEquals(SummonRoute.LEG_PIVOT, legs.get(0)[2]);
        assertEquals(SummonRoute.LEG_FORWARD, legs.get(1)[2]);
        assertEquals(legs.get(0)[1], legs.get(1)[0]);
        assertEquals(route.poses.size() - 1, legs.get(legs.size() - 1)[1]);
        //It's stopped through the turn.
        SpeedProfile profile = SpeedProfile.plan(route, KINEMATICS, new SpeedProfile.Settings(), 0);
        for (int i = legs.get(0)[0]; i <= legs.get(0)[1]; ++i) {
            assertEquals(0, profile.speeds[i], 0);
        }
    }

    @Test
    public void drivesATurnOnTheSpotAtTheStart() {
        SummonRoute route = plan(bay(), new SummonRoute.Pose(-26.5, 0, 17.5, -90, 0), true, 4).getRoute();
        DriveSimulator.Drive drive = DriveSimulator.drive(route, KINEMATICS, 0, 0, 0, 0, true);
        assertTrue(drive.arrived);
        assertEquals(0, drive.getEndError(route), 0.35);
        assertEquals(0, drive.getYawError(route), 3);
        assertTrue("strayed " + drive.worstCrossTrack, drive.worstCrossTrack < 0.35);
        assertEquals(route.getPivotDegrees(), drive.pivoted, 3);
        assertEquals(0, drive.pivotDrift, 0.05);
    }

    @Test
    public void drivesATurnOnTheSpotToLineUp() {
        SummonRoute route = searchOutOfBay(true);
        DriveSimulator.Drive drive = DriveSimulator.drive(route, KINEMATICS, 0, 0, 0, 0, true);
        assertTrue(drive.arrived);
        assertEquals(0, drive.getEndError(route), 0.35);
        assertEquals(0, drive.getYawError(route), 3);
        assertTrue("strayed " + drive.worstCrossTrack, drive.worstCrossTrack < 0.35);
        assertTrue("drifted " + drive.pivotDrift, drive.pivotDrift < 0.3);
    }
}
