package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

public class PathTrackerTest {
    private static final VehicleKinematics KINEMATICS = new VehicleKinematics(2.6, 0.35, 0, false);
    /**How fast the simulated vehicle speeds up and brakes, in m/s per second, and the length of a tick in seconds.**/
    private static final double ACCELERATION = 2.5;
    private static final double BRAKING = 4;
    private static final double TICK = 0.05;

    /**What happened driving a route.**/
    private static final class Drive {
        double x;
        double z;
        double yaw;
        double speed;
        double worstCrossTrack;
        boolean arrived;
    }

    /**
     * Drives the route with a simple vehicle that steers the way MTS cars do, starting off to the side and turned by
     * the offsets.  Errors are only counted once it's had settleDistance blocks to get onto the route.
     */
    private static Drive drive(SummonRoute route, double sideOffset, double yawOffset, double settleDistance) {
        SpeedProfile profile = SpeedProfile.plan(route, KINEMATICS, new SpeedProfile.Settings(), 0);
        PathTracker tracker = new PathTracker(route, profile, KINEMATICS, new PathTracker.Settings());
        SummonRoute.Pose start = route.getStart();
        Drive drive = new Drive();
        drive.yaw = start.yaw + yawOffset;
        drive.x = start.x + Math.cos(Math.toRadians(start.yaw)) * sideOffset;
        drive.z = start.z - Math.sin(Math.toRadians(start.yaw)) * sideOffset;
        for (int tick = 0; tick < 20 * 60; ++tick) {
            PathTracker.Command command = tracker.update(drive.x, drive.z, drive.yaw, drive.speed);
            if (command.along > settleDistance) {
                drive.worstCrossTrack = Math.max(drive.worstCrossTrack, Math.abs(command.crossTrack));
            }
            if (command.arrived) {
                drive.arrived = true;
                break;
            }
            double change = command.targetSpeed - drive.speed;
            drive.speed += Math.max(-BRAKING * TICK, Math.min(ACCELERATION * TICK, change));
            double moved = drive.speed * TICK;
            double turn = Math.toDegrees(KINEMATICS.getCurvature(command.rudder, drive.speed) * moved);
            double midYaw = Math.toRadians(drive.yaw + turn / 2);
            drive.x += Math.sin(midYaw) * moved;
            drive.z += Math.cos(midYaw) * moved;
            drive.yaw += turn;
        }
        return drive;
    }

    private static SummonRoute plannedRoute() {
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(FakeTerrain.flat(60).fill(-10, 0, 14, 10, 6, 15)), ArrivalPlannerTest.car()), KINEMATICS, new RoutePlanner.Settings(), new Point3D(0.5, 0, 0.5), 0);
        planner.finish();
        assertNotNull(planner.getRoute());
        return planner.getRoute();
    }

    private static void checkArrival(Drive drive, SummonRoute route) {
        assertTrue(drive.arrived);
        SummonRoute.Pose end = route.getEnd();
        assertEquals(0, Math.hypot(drive.x - end.x, drive.z - end.z), 0.35);
        assertEquals(0, VehicleSummonFlight.clampAngle(drive.yaw - end.yaw), 3);
    }

    @Test
    public void followsAPlannedRoute() {
        SummonRoute route = plannedRoute();
        Drive drive = drive(route, 0, 0, 0);
        checkArrival(drive, route);
        assertTrue("strayed " + drive.worstCrossTrack, drive.worstCrossTrack < 0.35);
    }

    @Test
    public void followsATightTurn() {
        SummonRoute route = PathSmootherTest.route(new double[] { 12, 0 }, new double[] { 4 * Math.PI / 2, 1 / 4D }, new double[] { 8, 0 });
        Drive drive = drive(route, 0, 0, 0);
        checkArrival(drive, route);
        assertTrue("strayed " + drive.worstCrossTrack, drive.worstCrossTrack < 0.35);
    }

    @Test
    public void getsBackOntoTheRoute() {
        SummonRoute route = plannedRoute();
        Drive drive = drive(route, 0.8, 12, 8);
        checkArrival(drive, route);
        assertTrue("strayed " + drive.worstCrossTrack, drive.worstCrossTrack < 0.35);
    }
}
