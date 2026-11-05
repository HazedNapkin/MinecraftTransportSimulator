package minecrafttransportsimulator.systems.summoning;

import java.util.List;

import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;

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
 * <p>
 * A route that changes direction is followed a way at a time.  It only looks for where it is along the way it's
 * going, stops where that ends just as it stops at the end, and then carries on along the next way.  Backing up it
 * steers by the same route, but corrects heading errors the other way, as backing up turns the vehicle the other way
 * for the same steering.  Changing gear is up to whoever's driving: the command says which way to be going.
 * <p>
 * Where the route turns on the spot, it stops, then steers to turn it to the yaw the turn ends at, the more lock the
 * further it has to go, until it's near enough that.  The command says it's turning on the spot, which a vehicle that
 * skid steers does stopped and in neutral, and the steering is what turns it.
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
        double correctionTime = 0.5;
        double minCorrectionDistance = 2;
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
        /**How near where it changes direction, in blocks, and how slow, in m/s, counts as stopped there to go the other way.**/
        double cuspTolerance = 0.5;
        double stoppedSpeed = 0.5;
        /**Least speed to aim for setting off again after changing direction, in m/s, or it'd aim for the stop it set off from.**/
        double minSpeed = 1;
        /**
         * How fast the vehicle turns on the spot at full lock, in degrees a tick, and how near the yaw it's turning to
         * counts as there, in degrees.  MTS turns skid steering vehicles a twentieth of the steering angle a tick.
         */
        double pivotRate = EntityVehicleF_Physics.MAX_RUDDER_ANGLE / 20;
        double pivotTolerance = 1;
    }

    /**What to do this tick.**/
    static final class Command {
        /**The rudder input to steer with.**/
        double rudder;
        /**The speed to be doing, in m/s.**/
        double targetSpeed;
        /**True once the vehicle's at the end.**/
        boolean arrived;
        /**True if it should be backing up, so in reverse gear.**/
        boolean reverse;
        /**True if it just got to where it changes direction, this tick, or started or finished turning on the spot.**/
        boolean changedDirection;
        /**True if it should be turning on the spot, stopped and in neutral, steering with the rudder.**/
        boolean pivot;
        /**How far the vehicle is to the left of the route, in blocks, negative to the right.**/
        double crossTrack;
        /**How far along the route the vehicle is, and how far is left, to the end and to where it next changes direction, in blocks.**/
        double along;
        double remaining;
        double legRemaining;
    }

    private final SummonRoute route;
    private final List<SummonRoute.Pose> poses;
    private final SpeedProfile profile;
    private final VehicleKinematics kinematics;
    private final Settings settings;
    private final double length;
    /**The segment, from pose segment to segment + 1, the vehicle was last nearest.**/
    private int segment;
    /**Each way it goes, as {@link SummonRoute#getLegs()} has them, and which it's on.**/
    private final List<int[]> legs;
    private int leg;
    private double lastRudder;
    private final Command command = new Command();

    PathTracker(SummonRoute route, SpeedProfile profile, VehicleKinematics kinematics, Settings settings) {
        this.route = route;
        this.poses = route.poses;
        this.profile = profile;
        this.kinematics = kinematics;
        this.settings = settings;
        this.length = profile.distances[profile.distances.length - 1];
        this.legs = route.getLegs();
    }

    /**How many times the route changes direction.**/
    int getCusps() {
        return route.getCusps().size();
    }

    /**Starts the steering from the rudder input the vehicle already has, so it doesn't jump.**/
    void setRudder(double rudder) {
        lastRudder = rudder;
    }

    /**Works out what to do with the vehicle's origin at x,z, facing the yaw, going at the speed in m/s.**/
    Command update(double x, double z, double yaw, double speed) {
        int legStart = legs.get(leg)[0];
        int legEnd = legs.get(leg)[1];
        if (legs.get(leg)[2] == SummonRoute.LEG_PIVOT) {
            return pivot(legStart, legEnd, yaw, speed);
        }
        command.pivot = false;
        //Find the nearest point on the route, from the segment it was last nearest on, along the way it's going.
        boolean reverse = route.leavesInReverse(legStart);
        double nearestDistance = Double.MAX_VALUE;
        double fraction = 0;
        int nearest = segment;
        for (int i = segment; i < Math.min(legEnd, segment + settings.trackWindow); ++i) {
            SummonRoute.Pose from = poses.get(i);
            SummonRoute.Pose to = poses.get(i + 1);
            double segmentX = to.x - from.x;
            double segmentZ = to.z - from.z;
            double segmentLengthSquared = segmentX * segmentX + segmentZ * segmentZ;
            double t = segmentLengthSquared > 0 ? ((x - from.x) * segmentX + (z - from.z) * segmentZ) / segmentLengthSquared : 0;
            //Only the last segment of the way runs on past its end, for when the vehicle's past where it stops.
            t = Math.max(0, i == legEnd - 1 ? t : Math.min(1, t));
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
        command.legRemaining = profile.distances[legEnd] - command.along;

        //How far off the route it is to the side, and in heading.
        double pathYaw = fraction >= 1 ? to.yaw : from.yaw + VehicleSummonFlight.clampAngle(to.yaw - from.yaw) * fraction;
        double pathX = from.x + (to.x - from.x) * fraction;
        double pathZ = from.z + (to.z - from.z) * fraction;
        double leftX = Math.cos(Math.toRadians(pathYaw));
        double leftZ = -Math.sin(Math.toRadians(pathYaw));
        command.crossTrack = (x - pathX) * leftX + (z - pathZ) * leftZ;
        double headingError = Math.toRadians(VehicleSummonFlight.clampAngle(yaw - pathYaw));

        //Steer for the route's curvature a moment ahead, and to bring it back onto the route over the correction distance.
        //It only looks ahead as far as the way it's going goes, as the steering after it is for going the other way.
        double previewLimit = leg == legs.size() - 1 ? Double.MAX_VALUE : profile.distances[Math.max(legStart, legEnd - 1)];
        double curvature = command.legRemaining > 0 ? getCurvatureAt(Math.min(previewLimit, command.along + Math.max(0, speed) * settings.previewTime)) : 0;
        double correction = Math.max(settings.minCorrectionDistance, speed * settings.correctionTime);
        double error = Math.max(-settings.maxCorrectedError, Math.min(settings.maxCorrectedError, command.crossTrack));
        //Facing left of the route turns it further left driving forward, but back right backing up, so that's
        //corrected the other way.  Being left of the route needs steering right either way, as backing up both
        //turns it and moves it the other way.
        double headingCorrection = 2 * Math.sin(headingError) / correction;
        curvature += (reverse ? headingCorrection : -headingCorrection) - error / (correction * correction);
        double rudder = kinematics.getRudderInput(curvature, Math.max(speed, 1));
        lastRudder += Math.max(-settings.maxSteeringRate, Math.min(settings.maxSteeringRate, rudder - lastRudder));
        command.rudder = lastRudder;

        //Keep to the planned speed, slower if well off the route, and no faster than stops it right where it stops.
        double plannedSpeed = getSpeedAt(command.along);
        if (leg > 0) {
            plannedSpeed = Math.max(plannedSpeed, settings.minSpeed);
        }
        double targetSpeed = Math.min(plannedSpeed, Math.sqrt(2 * settings.braking * Math.max(0, command.legRemaining)));
        double offRoute = Math.abs(command.crossTrack);
        if (offRoute > settings.offRouteDistance) {
            targetSpeed *= settings.offRouteDistance / offRoute;
        }
        command.targetSpeed = targetSpeed;
        command.reverse = reverse;
        command.changedDirection = false;
        command.arrived = leg == legs.size() - 1 && command.remaining <= settings.arriveTolerance;

        //Once it's stopped where it changes direction or turns on the spot, or gone past, go on to that.
        if (leg < legs.size() - 1 && (command.legRemaining <= 0 || (command.legRemaining <= settings.cuspTolerance && speed < settings.stoppedSpeed))) {
            ++leg;
            segment = legEnd;
            command.reverse = legs.get(leg)[2] == SummonRoute.LEG_REVERSE;
            command.changedDirection = true;
            command.targetSpeed = 0;
        }
        return command;
    }

    /**
     * Turns the vehicle on the spot from the first pose to the yaw of the last, once it's stopped.  The steering is
     * what turns it this tick, held to full lock and to how fast the wheel turns, and it's done once it's near enough.
     */
    private Command pivot(int start, int end, double yaw, double speed) {
        double error = VehicleSummonFlight.clampAngle(poses.get(end).yaw - yaw);
        command.pivot = true;
        command.reverse = false;
        command.changedDirection = false;
        command.targetSpeed = 0;
        command.crossTrack = 0;
        command.along = profile.distances[start];
        command.remaining = length - command.along;
        command.legRemaining = 0;
        command.arrived = false;
        double rudder = 0;
        if (speed < settings.stoppedSpeed) {
            //Positive rudder turns it right, which is yaw falling, by the rate in proportion to the lock a tick.
            rudder = -Math.max(-1, Math.min(1, error / settings.pivotRate)) * EntityVehicleF_Physics.MAX_RUDDER_ANGLE;
        }
        lastRudder += Math.max(-settings.maxSteeringRate, Math.min(settings.maxSteeringRate, rudder - lastRudder));
        command.rudder = lastRudder;
        if (Math.abs(error) <= settings.pivotTolerance) {
            //Lined up.  Straighten the wheel and drive on.
            ++leg;
            segment = end;
            lastRudder = 0;
            command.rudder = 0;
            command.pivot = false;
            command.changedDirection = true;
            if (leg >= legs.size()) {
                leg = legs.size() - 1;
                command.arrived = true;
            } else {
                command.reverse = legs.get(leg)[2] == SummonRoute.LEG_REVERSE;
            }
        }
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
