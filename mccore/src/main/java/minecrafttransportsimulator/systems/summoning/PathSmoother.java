package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Smooths a route found by the {@link HybridSearch}.  Its routes are strings of arcs, each a fixed steering angle,
 * so the steering jumps where one meets the next, and there can be small wiggles where a few arcs make up a bend.
 * Smoothing eases the steering in and out of turns and irons out wiggles, which looks far more natural.
 * <p>
 * Each pose is moved to lessen how sharply the route bends round it and its neighbours, which evens out the bends,
 * and back towards where it was, which keeps it near the route the search checked fits.  Lessening the bending,
 * rather than just pulling each pose towards halfway between its neighbours, also blends smoothly into the poses
 * that are held, instead of leaving a kink where it meets them.  After smoothing, every pose is checked
 * again: the vehicle must still fit there, and the route must turn no tighter than the vehicle can.  Wherever it
 * doesn't, that stretch is put back as it was and held there, and the rest smoothed again.
 * <p>
 * The first poses stay put, so the route still sets off where and how it was planned to, and so does the end of the
 * straight run-in, so it still pulls up exactly where and how it should.  The start of the run-in is left free, as
 * that's where the steering eases off out of the last turn.
 * <p>
 * A route that changes direction is smoothed a way at a time.  Where it stops to change direction stays put, and so
 * do the poses either side, so it still stops there facing the way it was planned to, and no pose is smoothed
 * against ones the other side of a change of direction, which would round off the corner it makes there.
 */
final class PathSmoother {
    /**
     * How strongly each pose is moved to lessen the bending, and back to where it was, each pass.  The first must stay
     * under 1/16, or the passes overshoot and the route shakes apart.
     */
    private static final double SMOOTH_WEIGHT = 0.06;
    private static final double ORIGINAL_WEIGHT = 0.0005;
    private static final int PASSES = 3000;
    /**Most times to put back stretches that don't fit and smooth again, before giving up and keeping the route as it was.**/
    private static final int MAX_ROUNDS = 6;
    /**How many poses either side of one that doesn't fit are put back too.**/
    private static final int HOLD_SPREAD = 2;
    /**
     * How much tighter than the vehicle's tightest turn the route may seem to turn between poses.  Poses are a chord
     * apart, a touch shorter than the arc between them, so even a turn exactly as tight as it can seems a bit tighter.
     */
    private static final double CURVATURE_TOLERANCE = 1.01;
    /**How many poses at the start are held, so the route sets off facing the way it was planned to.**/
    private static final int HELD_AT_START = 2;
    /**
     * How much of the start of the run-in is left free to ease out of the last turn along, and the least of the run-in
     * that's always held straight, in blocks.
     */
    private static final double EASE_OUT = 2;
    private static final double MIN_HELD_RUN_IN = 2;

    private PathSmoother() {
    }

    /**
     * Returns the route smoothed, or the route as it was if smoothing couldn't be made to fit.  The route ends with a
     * straight run-in runIn blocks long, the end of which is held as it is, and no part may turn tighter than maxCurvature.
     */
    static SummonRoute smooth(SummonRoute route, PoseChecker checker, double maxCurvature, double runIn) {
        List<SummonRoute.Pose> poses = route.poses;
        int count = poses.size();
        if (count < 5) {
            return route;
        }
        double[] originalX = new double[count];
        double[] originalZ = new double[count];
        double[] x = new double[count];
        double[] z = new double[count];
        boolean[] held = new boolean[count];
        double heldRunIn = Math.max(MIN_HELD_RUN_IN, runIn - EASE_OUT);
        double fromEnd = 0;
        for (int i = count - 1; i >= 0; --i) {
            SummonRoute.Pose pose = poses.get(i);
            originalX[i] = x[i] = pose.x;
            originalZ[i] = z[i] = pose.z;
            if (i < count - 1) {
                fromEnd += Math.hypot(pose.x - poses.get(i + 1).x, pose.z - poses.get(i + 1).z);
            }
            held[i] = i < HELD_AT_START || fromEnd <= heldRunIn + 1e-6;
        }
        //Each pose is smoothed against the two either side of it, so holding as many either side of a change of
        //direction keeps every pose that's moved to poses going its way.
        for (int cusp : route.getCusps()) {
            for (int i = Math.max(0, cusp - HELD_AT_START); i <= Math.min(count - 1, cusp + HELD_AT_START); ++i) {
                held[i] = true;
            }
        }

        for (int round = 0; round < MAX_ROUNDS; ++round) {
            for (int pass = 0; pass < PASSES; ++pass) {
                //The first two and last are always held, so every pose moved has two either side.
                for (int i = 2; i < count - 2; ++i) {
                    if (!held[i]) {
                        x[i] -= SMOOTH_WEIGHT * getBending(x, i) - ORIGINAL_WEIGHT * (originalX[i] - x[i]);
                        z[i] -= SMOOTH_WEIGHT * getBending(z, i) - ORIGINAL_WEIGHT * (originalZ[i] - z[i]);
                    }
                }
            }
            List<SummonRoute.Pose> smoothed = toPoses(route, x, z, held);
            List<Integer> misfits = findMisfits(smoothed, checker, maxCurvature);
            if (misfits.isEmpty()) {
                return new SummonRoute(Collections.unmodifiableList(smoothed), route.placed, route.goal);
            }
            for (int misfit : misfits) {
                for (int i = Math.max(0, misfit - HOLD_SPREAD); i <= Math.min(count - 1, misfit + HOLD_SPREAD); ++i) {
                    x[i] = originalX[i];
                    z[i] = originalZ[i];
                    held[i] = true;
                }
            }
        }
        return route;
    }

    /**
     * Which way, and how much, moving the point would add to the bending at it and its neighbours: the fourth
     * difference along the route.  Moving it against this lessens the bending.
     */
    private static double getBending(double[] values, int i) {
        return values[i - 2] - 4 * values[i - 1] + 6 * values[i] - 4 * values[i + 1] + values[i + 2];
    }

    /**
     * Makes the poses for the smoothed points.  Each faces along the line from the point before it to the one after,
     * or the other way where it's backing up, and its curvature is how fast that changes along the route, for the
     * way it's going from there.  Held poses keep their yaw, as the start's, the run-in's and those either side of
     * changes of direction must be exact.  Heights stay as they were, as points only move a little.
     */
    private static List<SummonRoute.Pose> toPoses(SummonRoute route, double[] x, double[] z, boolean[] held) {
        List<SummonRoute.Pose> poses = route.poses;
        int count = poses.size();
        double[] yaws = new double[count];
        for (int i = 0; i < count; ++i) {
            if (held[i]) {
                yaws[i] = poses.get(i).yaw;
            } else {
                int before = Math.max(0, i - 1);
                int after = Math.min(count - 1, i + 1);
                double heading = Math.toDegrees(Math.atan2(x[after] - x[before], z[after] - z[before]));
                yaws[i] = route.leavesInReverse(i) ? VehicleSummonFlight.clampAngle(heading + 180) : heading;
            }
        }
        List<SummonRoute.Pose> smoothed = new ArrayList<>(count);
        for (int i = 0; i < count; ++i) {
            //Where it changes direction, the steering is for the way it sets off from there.
            int before = route.isCusp(i) ? i : Math.max(0, i - 1);
            int after = Math.min(count - 1, i + 1);
            double length = 0;
            for (int j = before; j < after; ++j) {
                length += Math.hypot(x[j + 1] - x[j], z[j + 1] - z[j]);
            }
            double curvature = length > 0 ? Math.toRadians(VehicleSummonFlight.clampAngle(yaws[after] - yaws[before])) / length : 0;
            //Backing up, the yaw changes the other way for the same steering.
            if (route.leavesInReverse(i)) {
                curvature = -curvature;
            }
            smoothed.add(new SummonRoute.Pose(x[i], poses.get(i).y, z[i], yaws[i], curvature, poses.get(i).reverse));
        }
        return smoothed;
    }

    /**The poses where the vehicle no longer fits, or that turn tighter from the pose before than it can.**/
    private static List<Integer> findMisfits(List<SummonRoute.Pose> poses, PoseChecker checker, double maxCurvature) {
        List<Integer> misfits = new ArrayList<>();
        for (int i = 0; i < poses.size(); ++i) {
            SummonRoute.Pose pose = poses.get(i);
            boolean fits = checker.canDrive(pose.x, pose.y, pose.z, pose.yaw);
            if (fits && i > 0) {
                SummonRoute.Pose last = poses.get(i - 1);
                double spacing = Math.hypot(pose.x - last.x, pose.z - last.z);
                double turn = Math.toRadians(Math.abs(VehicleSummonFlight.clampAngle(pose.yaw - last.yaw)));
                fits = spacing > 0 && turn / spacing <= maxCurvature * CURVATURE_TOLERANCE;
            }
            if (!fits) {
                misfits.add(i);
            }
        }
        return misfits;
    }
}
