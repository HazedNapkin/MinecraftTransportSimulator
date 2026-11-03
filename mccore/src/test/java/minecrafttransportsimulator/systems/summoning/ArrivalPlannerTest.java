package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

public class ArrivalPlannerTest {
    private static final Point3D PLAYER = new Point3D(0.5, 0, 0.5);

    /**A car 2 wide, 4 long and 1.25 high, with its origin in the middle and a wheel near each corner.**/
    static VehicleShape car() {
        List<double[]> boxes = Arrays.asList(new double[] { 0, 0.75, -1, 1, 0.5 }, new double[] { 0, 0.75, 1, 1, 0.5 });
        List<double[]> wheels = Arrays.asList(new double[] { 0.8, 0.25, 1.3, 0.25, 0.25 }, new double[] { -0.8, 0.25, 1.3, 0.25, 0.25 }, new double[] { 0.8, 0.25, -1.3, 0.25, 0.25 }, new double[] { -0.8, 0.25, -1.3, 0.25, 0.25 });
        return VehicleShape.of(boxes, wheels);
    }

    private static ArrivalPlanner planner(FakeTerrain terrain, ArrivalPlanner.Settings settings) {
        return new ArrivalPlanner(new PoseChecker(new TerrainMap(terrain), car()), settings, PLAYER, 0);
    }

    private static double bearingOf(ArrivalPlanner.Goal goal) {
        return Math.toDegrees(Math.atan2(goal.x - PLAYER.x, goal.z - PLAYER.z));
    }

    @Test
    public void arrivalYawPutsTheRightSideToThePlayer() {
        //At bearing 0 the vehicle is in +Z of the player.  Left side to them faces +X, with its left pointing -Z.
        assertEquals(90, ArrivalPlanner.getArrivalYaw(0, 0), 1e-9);
        //Nose first faces -Z, straight at them.
        assertEquals(180, Math.abs(ArrivalPlanner.getArrivalYaw(0, 90)), 1e-9);
        //Right side faces -X, tail first faces +Z.
        assertEquals(-90, ArrivalPlanner.getArrivalYaw(0, 180), 1e-9);
        assertEquals(0, ArrivalPlanner.getArrivalYaw(0, 270), 1e-9);
    }

    @Test
    public void stopsTheGapAway() {
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        ArrivalPlanner planner = planner(FakeTerrain.flat(30), settings);
        for (double bearing : new double[] { 0, 30, -45, 90 }) {
            double[] origin = planner.getOrigin(bearing);
            double nearest = planner.getNearestDistance(origin[0], origin[1], origin[2]);
            assertTrue(nearest >= settings.gap);
            assertTrue(nearest < settings.gap + 0.1);
        }
        //Side on at bearing 0, the origin is half the width plus the gap out, straight in front.
        double[] origin = planner.getOrigin(0);
        assertEquals(PLAYER.x, origin[0], 1e-9);
        assertEquals(PLAYER.z + 1 + settings.gap, origin[1], 0.06);
    }

    @Test
    public void linesUpTheSeatOppositeThePlayer() {
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        //A driver's seat a block forward of the origin, on the left.
        settings.alignX = 0.5;
        settings.alignZ = 1;
        double[] origin = planner(FakeTerrain.flat(30), settings).getOrigin(0);
        //Facing +X, the seat is a block towards +X of the origin, so the origin is a block towards -X of the player.
        assertEquals(PLAYER.x - 1, origin[0], 1e-9);
    }

    @Test
    public void prefersStraightAheadWithFullRunIn() {
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        ArrivalPlanner planner = planner(FakeTerrain.flat(30), settings);
        planner.finish();
        List<ArrivalPlanner.Goal> goals = planner.getGoals();
        assertEquals(ArrivalPlanner.MAX_GOALS, goals.size());
        ArrivalPlanner.Goal best = goals.get(0);
        assertEquals(0, bearingOf(best), 1);
        assertEquals(90, best.yaw, 1e-9);
        assertEquals(0, best.y);
        assertEquals(settings.preferredRunIn, best.runIn, 1e-9);
        for (int i = 1; i < goals.size(); ++i) {
            assertTrue(goals.get(i - 1).cost <= goals.get(i).cost);
        }
    }

    @Test
    public void goesRoundWallsInFront() {
        //A wall right in front of the player, across their view.
        FakeTerrain terrain = FakeTerrain.flat(30).fill(-5, 0, 2, 5, 3, 3);
        ArrivalPlanner planner = planner(terrain, new ArrivalPlanner.Settings());
        planner.finish();
        assertFalse(planner.getGoals().isEmpty());
        assertTrue(Math.abs(bearingOf(planner.getGoals().get(0))) > 30);
    }

    @Test
    public void runInStopsAtObstacles() {
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        //Facing +X from bearing 0, the run-in goes back towards -X.  Put a pillar at x = -7, which the tail, 2 blocks
        //behind the origin, reaches once the origin is 4.5 blocks back from the goal.
        FakeTerrain terrain = FakeTerrain.flat(30).fill(-7, 0, 2, -7, 3, 2);
        ArrivalPlanner planner = planner(terrain, settings);
        double[] origin = planner.getOrigin(0);
        double[] runIn = planner.measureRunIn(origin[0], 0, origin[1], origin[2]);
        assertEquals(4.5, runIn[0], 1e-9);
        assertEquals(0, runIn[1], 0);
    }

    @Test
    public void turnInNeedsRoomOnOneSide() {
        //A straight starting well in front of the player facing +Z, between two walls.  Its quarter turns in end
        //4 blocks either side of it, facing across, so the walls block them when 4 or 5 blocks out but not when 9 or 10.
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        FakeTerrain narrow = FakeTerrain.flat(30).fill(-4, 0, 8, -4, 3, 20).fill(5, 0, 8, 5, 3, 20);
        FakeTerrain wide = FakeTerrain.flat(30).fill(-9, 0, 8, -9, 3, 20).fill(10, 0, 8, 10, 3, 20);
        assertFalse(planner(narrow, settings).canTurnIn(0.5, 0, 20.5, 0));
        assertTrue(planner(wide, settings).canTurnIn(0.5, 0, 20.5, 0));
        //Only one side needs room.
        FakeTerrain oneSide = FakeTerrain.flat(30).fill(-4, 0, 8, -4, 3, 20);
        assertTrue(planner(oneSide, settings).canTurnIn(0.5, 0, 20.5, 0));
    }

    @Test
    public void movesOutWhenItCantTurnIn() {
        //Facing +X from bearing 0, the run-in goes back towards -X.  A wall across it at x = -7 cuts it to 4.5 blocks,
        //long enough on its own, but leaves no room to turn onto it either way that close to the player.
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        FakeTerrain terrain = FakeTerrain.flat(30).fill(-7, 0, 1, -7, 3, 3);
        ArrivalPlanner planner = planner(terrain, settings);
        double[] origin = planner.getOrigin(0);
        double[] runIn = planner.measureRunIn(origin[0], 0, origin[1], origin[2]);
        assertEquals(4.5, runIn[0], 1e-9);
        assertFalse(planner.canTurnIn(origin[0] - runIn[0], 0, origin[1], origin[2]));
        //So straight ahead it pulls up further out, where it can get in.
        planner.finish();
        for (ArrivalPlanner.Goal goal : planner.getGoals()) {
            if (Math.abs(bearingOf(goal)) < 1) {
                assertTrue(planner.getNearestDistance(goal.x, goal.z, goal.yaw) > settings.gap + 0.4);
            }
        }
    }

    @Test
    public void everyGoalCanBeDrivenInto() {
        //Scattered pillars, kept off the player.
        java.util.Random random = new java.util.Random(1);
        for (int layout = 0; layout < 10; ++layout) {
            FakeTerrain terrain = FakeTerrain.flat(30);
            for (int i = 0; i < 25; ++i) {
                int x = random.nextInt(31) - 15;
                int z = random.nextInt(31) - 15;
                if (Math.abs(x) > 1 || Math.abs(z) > 1) {
                    terrain.fill(x, 0, z, x, 2, z);
                }
            }
            ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
            ArrivalPlanner planner = planner(terrain, settings);
            planner.finish();
            for (ArrivalPlanner.Goal goal : planner.getGoals()) {
                assertTrue(planner.getNearestDistance(goal.x, goal.z, goal.yaw) >= settings.gap);
                double[] runIn = planner.measureRunIn(goal.x, goal.y, goal.z, goal.yaw);
                //The car is 4 long, so its run-in has to be at least that, and it should be twice that.
                assertTrue(runIn[0] >= 4);
                if (runIn[0] < 8) {
                    double forwardX = Math.sin(Math.toRadians(goal.yaw));
                    double forwardZ = Math.cos(Math.toRadians(goal.yaw));
                    assertTrue(planner.canTurnIn(goal.x - forwardX * runIn[0], (int) runIn[2], goal.z - forwardZ * runIn[0], goal.yaw));
                }
            }
        }
    }

    @Test
    public void comesDownToTheGroundBelowAPlayerOnABlock() {
        //The player stands on a single block, so their feet are at y = 1 and the ground round them at 0.
        FakeTerrain terrain = FakeTerrain.flat(30).fill(0, 0, 0, 0, 0, 0);
        ArrivalPlanner planner = new ArrivalPlanner(new PoseChecker(new TerrainMap(terrain), car()), new ArrivalPlanner.Settings(), new Point3D(0.5, 1, 0.5), 0);
        planner.finish();
        assertEquals(0, planner.getGoals().get(0).y);
    }
}
