package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

public class SpeedProfileTest {
    private static final VehicleKinematics KINEMATICS = new VehicleKinematics(2.6, 0.35, 0, false);

    private static void checkLimits(SpeedProfile profile, SpeedProfile.Settings settings) {
        double[] speeds = profile.speeds;
        double[] distances = profile.distances;
        assertEquals(0, speeds[speeds.length - 1], 0);
        for (int i = 1; i < speeds.length; ++i) {
            double gap = distances[i] - distances[i - 1];
            assertTrue("speeding up at " + i, speeds[i] * speeds[i] - speeds[i - 1] * speeds[i - 1] <= 2 * settings.acceleration * gap + 1e-9);
            assertTrue("braking at " + i, speeds[i - 1] * speeds[i - 1] - speeds[i] * speeds[i] <= 2 * settings.braking * gap + 1e-9);
            assertTrue(speeds[i] <= settings.cruiseSpeed + 1e-9);
        }
    }

    @Test
    public void speedsUpCruisesAndStops() {
        SummonRoute route = PathSmootherTest.route(new double[] { 60, 0 });
        SpeedProfile.Settings settings = new SpeedProfile.Settings();
        SpeedProfile profile = SpeedProfile.plan(route, KINEMATICS, settings, 0);
        checkLimits(profile, settings);
        assertEquals(settings.minSpeed, profile.speeds[0], 1e-9);
        //Halfway along a 60 block straight it's at cruise speed.
        assertEquals(settings.cruiseSpeed, profile.speeds[profile.speeds.length / 2], 1e-9);
    }

    @Test
    public void slowsForTurns() {
        double curvature = 0.2;
        SummonRoute route = PathSmootherTest.route(new double[] { 20, 0 }, new double[] { 10, curvature }, new double[] { 20, 0 });
        SpeedProfile.Settings settings = new SpeedProfile.Settings();
        SpeedProfile profile = SpeedProfile.plan(route, KINEMATICS, settings, 0);
        checkLimits(profile, settings);
        double limit = Math.min(Math.sqrt(settings.sideAcceleration / curvature), KINEMATICS.getMaxSpeed(curvature * settings.steeringMargin));
        assertTrue(limit < settings.cruiseSpeed);
        for (int i = 0; i < route.poses.size(); ++i) {
            if (route.poses.get(i).curvature == curvature) {
                assertTrue(profile.speeds[i] <= limit + 1e-9);
            }
        }
    }

    @Test
    public void slowsForSteps() {
        SummonRoute flat = PathSmootherTest.route(new double[] { 40, 0 });
        List<SummonRoute.Pose> poses = new ArrayList<>();
        for (int i = 0; i < flat.poses.size(); ++i) {
            SummonRoute.Pose pose = flat.poses.get(i);
            poses.add(new SummonRoute.Pose(pose.x, i >= 40 ? 1 : 0, pose.z, pose.yaw, 0));
        }
        SpeedProfile.Settings settings = new SpeedProfile.Settings();
        SpeedProfile profile = SpeedProfile.plan(new SummonRoute(poses, true, flat.goal), KINEMATICS, settings, 0);
        checkLimits(profile, settings);
        assertTrue(profile.speeds[40] <= settings.stepSpeed + 1e-9);
        assertTrue(profile.speeds[30] > settings.stepSpeed);
    }

    @Test
    public void carriesOnAtTheSpeedItsDoing() {
        SummonRoute route = PathSmootherTest.route(new double[] { 40, 0 });
        SpeedProfile profile = SpeedProfile.plan(route, KINEMATICS, new SpeedProfile.Settings(), 6);
        assertEquals(6, profile.speeds[0], 1e-9);
    }
}
