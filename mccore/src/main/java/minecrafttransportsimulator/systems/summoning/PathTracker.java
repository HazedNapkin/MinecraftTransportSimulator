package minecrafttransportsimulator.systems.summoning;

import java.util.List;

/**
 * Steers a vehicle along a route and says how fast to go, one tick at a time.
 * <p>
 * Each tick it finds where along the route the vehicle is, by the nearest point on the route rather than the nearest
 * pose, and how far it is off to the side and how far its heading is off the route's.  It steers for the route's own
 * curvature a moment ahead, so it starts turning as the route does rather than once it's drifted off, plus a
 * correction that brings it back onto the route over a distance that grows with speed, so corrections are gentle at
 * speed and firm when slow.  The steering wheel only turns so fast, like a driver's would.
 * <p>
 * The speed is the {@link SpeedProfile}'s at that point, slower if it's well off the route, and no faster than braking
 * gently stops it exactly at the end, worked out from the exact distance left so it doesn't stop short or overrun.
 */
final class PathTracker {
    /**How it steers and stops.**/
    static final class Settings {
        /**How far ahead in time the route's curvature is steered for, in seconds.  The vehicle takes a moment to respond.**/
        double previewTime = 0.25;
        /**
         * Distance over which errors are corrected: this many seconds of travel at the speed, but no shorter than the
         * least, in blocks.  Longer is smoother, shorter holds the route more tightly.
         */
        double correctionTime = 0.7;
        double minCorrectionDistance = 2.5;
        /**The most the steering may change in a tick, in degrees of rudder input.**/
        double maxSteeringRate = 5;
        /**How near the end counts as there, in blocks.**/
        double arriveTolerance = 0.2;
        /**How hard it plans to brake for the end, in m/s per second.**/
        double braking = 2.5;
        /**How far off the route it may be before it slows down, and the most off it's corrected for, in blocks.**/
        double offRouteDistance = 1;
        double maxCorrectedError = 2;
        /**How many poses on from the last nearest one to look for the nearest one now.**/
        int trackWindow = 24;
    }

    /**What to do this tick.**/
    static final class Command {
        /**The rudder input to steer with.**/
        double rudder;
        /**The speed to be doing, in m/s.**/
        double targetSpeed;
        /**True once the vehicle's at the end.**/
        boolean arrived;
        /**How far the vehicle is to the left of the route, in blocks, negative to the right.**/
        double crossTrack;
        /**How far along the route the vehicle is, and how far is left, in blocks.**/
        double along;
        double remaining;
    }

    private final List<SummonRoute.Pose> poses;
    private final SpeedProfile profile;
    private final VehicleKinematics kinematics;
    private final Settings settings;
    private final double length;
    /**The segment, from pose segment to segment + 1, the vehicle was last nearest.**/
    private int segment;
    private double lastRudder;
    private final Command command = new Command();

    PathTracker(SummonRoute route, SpeedProfile profile, VehicleKinematics kinematics, Settings settings) {
        this.poses = route.poses;
        this.profile = profile;
        this.kinematics = kinematics;
        this.settings = settings;
        this.length = profile.distances[profile.distances.length - 1];
    }

    /**Starts the steering from the rudder input the vehicle already has, so it doesn't jump.**/
    void setRudder(double rudder) {
        lastRudder = rudder;
    }

    /**Works out what to do with the vehicle's origin at x,z, facing the yaw, going at the speed in m/s.**/
    Command update(double x, double z, double yaw, double speed) {
        //Find the nearest point on the route, from the segment it was last nearest on.
        int last = poses.size() - 1;
        double nearestDistance = Double.MAX_VALUE;
        double fraction = 0;
        int nearest = segment;
        for (int i = segment; i < Math.min(last, segment + settings.trackWindow); ++i) {
            SummonRoute.Pose from = poses.get(i);
            SummonRoute.Pose to = poses.get(i + 1);
            double segmentX = to.x - from.x;
            double segmentZ = to.z - from.z;
            double segmentLengthSquared = segmentX * segmentX + segmentZ * segmentZ;
            double t = segmentLengthSquared > 0 ? ((x - from.x) * segmentX + (z - from.z) * segmentZ) / segmentLengthSquared : 0;
            //Only the last segment runs on past its end, for when the vehicle's past the end of the route.
            t = Math.max(0, i == last - 1 ? t : Math.min(1, t));
            double distance = Math.hypot(x - (from.x + segmentX * t), z - (from.z + segmentZ * t));
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = i;
                fraction = t;
            }
        }
        segment = nearest;
        SummonRoute.Pose from = poses.get(segment);
        SummonRoute.Pose to = poses.get(segment + 1);
        double segmentLength = profile.distances[segment + 1] - profile.distances[segment];
        command.along = profile.distances[segment] + fraction * segmentLength;
        command.remaining = length - command.along;

        //How far off the route it is to the side, and in heading.
        double pathYaw = fraction >= 1 ? to.yaw : from.yaw + VehicleSummonFlight.clampAngle(to.yaw - from.yaw) * fraction;
        double pathX = from.x + (to.x - from.x) * fraction;
        double pathZ = from.z + (to.z - from.z) * fraction;
        double leftX = Math.cos(Math.toRadians(pathYaw));
        double leftZ = -Math.sin(Math.toRadians(pathYaw));
        command.crossTrack = (x - pathX) * leftX + (z - pathZ) * leftZ;
        double headingError = Math.toRadians(VehicleSummonFlight.clampAngle(yaw - pathYaw));

        //Steer for the route's curvature a moment ahead, and to bring it back onto the route over the correction distance.
        double curvature = command.remaining > 0 ? getCurvatureAt(command.along + Math.max(0, speed) * settings.previewTime) : 0;
        double correction = Math.max(settings.minCorrectionDistance, speed * settings.correctionTime);
        double error = Math.max(-settings.maxCorrectedError, Math.min(settings.maxCorrectedError, command.crossTrack));
        curvature -= 2 * Math.sin(headingError) / correction + error / (correction * correction);
        double rudder = kinematics.getRudderInput(curvature, Math.max(speed, 1));
        lastRudder += Math.max(-settings.maxSteeringRate, Math.min(settings.maxSteeringRate, rudder - lastRudder));
        command.rudder = lastRudder;

        //Keep to the planned speed, slower if well off the route, and no faster than stops it right at the end.
        double targetSpeed = Math.min(getSpeedAt(command.along), Math.sqrt(2 * settings.braking * Math.max(0, command.remaining)));
        double offRoute = Math.abs(command.crossTrack);
        if (offRoute > settings.offRouteDistance) {
            targetSpeed *= settings.offRouteDistance / offRoute;
        }
        command.targetSpeed = targetSpeed;
        command.arrived = command.remaining <= settings.arriveTolerance;
        return command;
    }

    /**The route's curvature the distance along it, or 0 past its end.**/
    double getCurvatureAt(double along) {
        int index = getIndexAt(along);
        if (index >= poses.size() - 1) {
            return 0;
        }
        double t = getFraction(index, along);
        return poses.get(index).curvature * (1 - t) + poses.get(index + 1).curvature * t;
    }

    /**The planned speed the distance along the route.**/
    double getSpeedAt(double along) {
        int index = getIndexAt(along);
        if (index >= poses.size() - 1) {
            return 0;
        }
        double t = getFraction(index, along);
        return profile.speeds[index] * (1 - t) + profile.speeds[index + 1] * t;
    }

    /**The last pose at or before the distance along the route.**/
    private int getIndexAt(double along) {
        int index = segment;
        while (index < poses.size() - 1 && profile.distances[index + 1] <= along) {
            ++index;
        }
        return index;
    }

    private double getFraction(int index, double along) {
        double gap = profile.distances[index + 1] - profile.distances[index];
        return gap > 0 ? Math.max(0, Math.min(1, (along - profile.distances[index]) / gap)) : 0;
    }
}
