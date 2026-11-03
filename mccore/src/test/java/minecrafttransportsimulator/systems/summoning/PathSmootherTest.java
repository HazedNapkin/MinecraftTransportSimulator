package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

public class PathSmootherTest {
    private static final double SPACING = 0.5;

    /**
     * Makes a route of poses every half block, driving each of the pieces in turn from the origin facing +Z.
     * Each piece is {length, curvature}.
     */
    static SummonRoute route(double[]... pieces) {
        List<SummonRoute.Pose> poses = new ArrayList<>();
        double[] pose = { 0.5, 0.5, 0 };
        poses.add(new SummonRoute.Pose(pose[0], 0, pose[1], pose[2], pieces[0][1]));
        for (double[] piece : pieces) {
            int samples = (int) Math.round(piece[0] / SPACING);
            for (int i = 0; i < samples; ++i) {
                pose = HybridSearch.arc(pose[0], pose[1], pose[2], piece[1], SPACING);
                poses.add(new SummonRoute.Pose(pose[0], 0, pose[1], pose[2], piece[1]));
            }
        }
        ArrivalPlanner.Goal goal = new ArrivalPlanner.Goal(pose[0], 0, pose[1], pose[2], 6, 0);
        return new SummonRoute(poses, true, goal);
    }

    private static double maxCurvatureJump(SummonRoute route) {
        double jump = 0;
        for (int i = 1; i < route.poses.size(); ++i) {
            jump = Math.max(jump, Math.abs(route.poses.get(i).curvature - route.poses.get(i - 1).curvature));
        }
        return jump;
    }

    @Test
    public void easesSteeringIntoTurns() {
        //Straight, a quarter turn left at radius 6, then a straight run-in.
        SummonRoute original = route(new double[] { 10, 0 }, new double[] { 6 * Math.PI / 2, 1 / 6D }, new double[] { 6, 0 });
        PoseChecker checker = new PoseChecker(new TerrainMap(FakeTerrain.flat(40)), ArrivalPlannerTest.car());
        SummonRoute smoothed = PathSmoother.smooth(original, checker, 1 / 4D, 6);
        assertEquals(original.poses.size(), smoothed.poses.size());

        //The steering changes by much less from one pose to the next.
        assertTrue(maxCurvatureJump(smoothed) < maxCurvatureJump(original) / 3);
        //The start and the end of the run-in stay exactly as they were.  The 6 block run-in is held for its last 4.
        for (int i : new int[] { 0, 1, smoothed.poses.size() - 1, smoothed.poses.size() - 8 }) {
            assertEquals(original.poses.get(i).x, smoothed.poses.get(i).x, 1e-9);
            assertEquals(original.poses.get(i).z, smoothed.poses.get(i).z, 1e-9);
            assertEquals(original.poses.get(i).yaw, smoothed.poses.get(i).yaw, 1e-9);
        }
        //It still fits everywhere and never turns tighter than the vehicle can.
        for (int i = 1; i < smoothed.poses.size(); ++i) {
            SummonRoute.Pose pose = smoothed.poses.get(i);
            SummonRoute.Pose last = smoothed.poses.get(i - 1);
            assertTrue(checker.canDrive(pose.x, pose.y, pose.z, pose.yaw));
            double turn = Math.toRadians(Math.abs(VehicleSummonFlight.clampAngle(pose.yaw - last.yaw)));
            assertTrue(turn / Math.hypot(pose.x - last.x, pose.z - last.z) <= 1.01 / 4);
        }
    }

    @Test
    public void ironsOutWiggles() {
        //A little left-right wiggle in an otherwise straight route.
        SummonRoute original = route(new double[] { 8, 0 }, new double[] { 1.5, 0.25 }, new double[] { 3, -0.25 }, new double[] { 1.5, 0.25 }, new double[] { 10, 0 });
        PoseChecker checker = new PoseChecker(new TerrainMap(FakeTerrain.flat(40)), ArrivalPlannerTest.car());
        SummonRoute smoothed = PathSmoother.smooth(original, checker, 1 / 4D, 6);
        double originalPeak = 0;
        double smoothedPeak = 0;
        for (int i = 0; i < original.poses.size(); ++i) {
            originalPeak = Math.max(originalPeak, Math.abs(original.poses.get(i).curvature));
            smoothedPeak = Math.max(smoothedPeak, Math.abs(smoothed.poses.get(i).curvature));
        }
        assertTrue(smoothedPeak < originalPeak / 2);
    }

    @Test
    public void keepsToTheRouteWhereSmoothingWouldHitThings() {
        //A tight quarter turn.  Smoothing on open ground cuts the corner, covering a block the route didn't.
        SummonRoute original = route(new double[] { 10, 0 }, new double[] { 4 * Math.PI / 2, 1 / 4D }, new double[] { 8, 0 });
        VehicleShape car = ArrivalPlannerTest.car();
        SummonRoute open = PathSmoother.smooth(original, new PoseChecker(new TerrainMap(FakeTerrain.flat(40)), car), 1 / 3D, 6);
        java.util.Set<Long> originalColumns = getColumns(original, car);
        Long cut = null;
        for (long column : getColumns(open, car)) {
            if (!originalColumns.contains(column)) {
                cut = column;
                break;
            }
        }
        assertTrue(cut != null);

        //Put a pillar there.  The route as it was still fits, and smoothing must keep it fitting.
        FakeTerrain terrain = FakeTerrain.flat(40);
        int x = (int) (cut >> 32);
        int z = (int) (long) cut;
        terrain.fill(x, 0, z, x, 2, z);
        assertTrue(allFit(original, new PoseChecker(new TerrainMap(terrain), car)));
        SummonRoute smoothed = PathSmoother.smooth(original, new PoseChecker(new TerrainMap(terrain), car), 1 / 3D, 6);
        assertTrue(allFit(smoothed, new PoseChecker(new TerrainMap(terrain), car)));
    }

    private static java.util.Set<Long> getColumns(SummonRoute route, VehicleShape shape) {
        java.util.Set<Long> columns = new java.util.HashSet<>();
        for (SummonRoute.Pose pose : route.poses) {
            columns.addAll(shape.getFootprint(pose.x, pose.z, pose.yaw).keySet());
        }
        return columns;
    }

    private static boolean allFit(SummonRoute route, PoseChecker checker) {
        for (SummonRoute.Pose pose : route.poses) {
            if (!checker.canDrive(pose.x, pose.y, pose.z, pose.yaw)) {
                return false;
            }
        }
        return true;
    }
}
