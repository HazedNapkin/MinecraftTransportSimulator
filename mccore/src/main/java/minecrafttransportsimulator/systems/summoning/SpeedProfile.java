package minecrafttransportsimulator.systems.summoning;

import java.util.List;

/**
 * The speed a vehicle should be doing at each pose along a route, so it drives in briskly but naturally: up to
 * speed smoothly from where it sets off, slowing ahead of turns and steps rather than at them, and braking gently
 * to a stop exactly where it pulls up.
 * <p>
 * Each pose gets the fastest speed it can be taken at: no faster than the vehicle drives in, slow enough that its
 * steering can make the turn with some to spare (MTS vehicles steer less the faster they go), slow enough that it
 * doesn't need more grip than a car has turning, and slow over steps.  Then a pass back from the end slows each pose
 * to what braking gently from it can get down to the speeds after it, ending at a stop, and a pass forward from the
 * start limits each to what speeding up gently from the speed it sets off at can reach.
 */
final class SpeedProfile {
    /**What limits the speeds, in m/s, m/s per second and blocks.**/
    static final class Settings {
        double cruiseSpeed = 8.3;
        /**How much more steering than a turn needs to keep in hand, for correcting drift, as a multiple.**/
        double steeringMargin = 1.25;
        /**Most sideways acceleration in turns, in m/s per second.  About a third of a g looks brisk without skidding.**/
        double sideAcceleration = 3;
        double acceleration = 2;
        double braking = 2.5;
        /**Speed to be down to at a step up or down, and the least to plan for anywhere short of the end.**/
        double stepSpeed = 2.5;
        double minSpeed = 1;
    }

    /**How far along the route each pose is, in blocks.**/
    final double[] distances;
    /**The speed to be doing at each pose, in m/s.  The last is 0.**/
    final double[] speeds;

    private SpeedProfile(double[] distances, double[] speeds) {
        this.distances = distances;
        this.speeds = speeds;
    }

    /**Plans the speeds along the route for the vehicle, setting off at startSpeed.**/
    static SpeedProfile plan(SummonRoute route, VehicleKinematics kinematics, Settings settings, double startSpeed) {
        List<SummonRoute.Pose> poses = route.poses;
        int count = poses.size();
        double[] distances = new double[count];
        double[] speeds = new double[count];
        for (int i = 0; i < count; ++i) {
            SummonRoute.Pose pose = poses.get(i);
            double curvature = Math.abs(pose.curvature);
            double speed = Math.min(settings.cruiseSpeed, kinematics.getMaxSpeed(curvature * settings.steeringMargin));
            if (curvature > 0) {
                speed = Math.min(speed, Math.sqrt(settings.sideAcceleration / curvature));
            }
            if (i > 0) {
                SummonRoute.Pose last = poses.get(i - 1);
                distances[i] = distances[i - 1] + Math.hypot(pose.x - last.x, pose.z - last.z);
                if (pose.y != last.y) {
                    speed = Math.min(speed, settings.stepSpeed);
                }
            }
            speeds[i] = Math.max(settings.minSpeed, speed);
        }
        speeds[count - 1] = 0;
        for (int i = count - 2; i >= 0; --i) {
            speeds[i] = Math.min(speeds[i], Math.sqrt(speeds[i + 1] * speeds[i + 1] + 2 * settings.braking * (distances[i + 1] - distances[i])));
        }
        //Setting off, it can only speed up so fast.  It still needs some speed to plan for, or it'd never get going.
        speeds[0] = Math.min(speeds[0], Math.max(startSpeed, settings.minSpeed));
        for (int i = 1; i < count - 1; ++i) {
            speeds[i] = Math.min(speeds[i], Math.sqrt(speeds[i - 1] * speeds[i - 1] + 2 * settings.acceleration * (distances[i] - distances[i - 1])));
        }
        return new SpeedProfile(distances, speeds);
    }
}
