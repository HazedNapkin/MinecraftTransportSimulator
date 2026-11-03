package minecrafttransportsimulator.systems.summoning;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.systems.summoning.VehicleShape.ShapeBox;

/**
 * How a summoned vehicle's drive in went, for tuning.  It's kept tick by tick as the vehicle drives, and told to the
 * player as it stops when summon debugging is on: how far off where it should have pulled up it is, and how well it
 * kept to the route and the planned speeds on the way.
 */
final class SummonDriveReport {
    /**Rudder input at or over this counts as full lock.**/
    private static final double FULL_LOCK = EntityVehicleF_Physics.MAX_RUDDER_ANGLE - 1;
    /**Which tick of the drive the speed it set off at is taken from.  It takes a moment for a placed vehicle's motion to settle.**/
    private static final int SET_OFF_TICK = 2;

    private final SummonRoute route;
    private final double routeLength;
    private final double plannedSeconds;
    private final double cruiseSpeed;
    private final double plannedStartSpeed;
    private int ticks;
    private double startSpeed = Double.NaN;
    private double topSpeed;
    private double worstCrossTrack;
    private double totalCrossTrack;
    private double worstSpeedError;
    private int fullLockTicks;
    private int slowTicks;

    /**Starts a report on driving the route, with the speeds planned along it, planning to set off at plannedStartSpeed.**/
    SummonDriveReport(SummonRoute route, SpeedProfile profile, double cruiseSpeed, double plannedStartSpeed) {
        this.route = route;
        this.cruiseSpeed = cruiseSpeed;
        this.plannedStartSpeed = plannedStartSpeed;
        double seconds = 0;
        for (int i = 1; i < profile.speeds.length; ++i) {
            double averageSpeed = Math.max(0.3, (profile.speeds[i - 1] + profile.speeds[i]) / 2);
            seconds += (profile.distances[i] - profile.distances[i - 1]) / averageSpeed;
        }
        this.plannedSeconds = seconds;
        this.routeLength = profile.distances[profile.distances.length - 1];
    }

    /**Records a tick of driving: what the tracker said, and the speed the vehicle's doing.**/
    void record(PathTracker.Command command, double speed) {
        ++ticks;
        if (ticks == SET_OFF_TICK) {
            startSpeed = speed;
        }
        topSpeed = Math.max(topSpeed, speed);
        double crossTrack = Math.abs(command.crossTrack);
        worstCrossTrack = Math.max(worstCrossTrack, crossTrack);
        totalCrossTrack += crossTrack;
        if (Math.abs(command.rudder) >= FULL_LOCK) {
            ++fullLockTicks;
        }
        //Only count it as too slow once it's had time to get going.
        if (ticks > 20 && speed < command.targetSpeed - 1.5) {
            ++slowTicks;
        }
        worstSpeedError = Math.max(worstSpeedError, speed - command.targetSpeed);
    }

    /**
     * Where the vehicle stopped, compared with where it should have: which side the player sees and how far they are
     * from it, compared with the arrival angle and gap it wanted, and how far short or past, to the side and crooked it is.
     */
    String getStop(String reason, PathTracker.Command command, EntityVehicleF_Physics vehicle, VehicleShape shape, Point3D player, double arrivalAngle, double gap, double speed) {
        double[] align = ArrivalPlanner.getAlignPoint(vehicle, arrivalAngle);
        Point3D local = player.copy().subtract(vehicle.position).reOrigin(vehicle.orientation);
        //Angles go round from the vehicle's left (+X) towards its front (+Z).
        double seenAngle = Math.toDegrees(Math.atan2(local.z - align[1], local.x - align[0]));
        double seenGap = Double.MAX_VALUE;
        for (ShapeBox box : shape.boxes) {
            seenGap = Math.min(seenGap, Math.hypot(Math.max(0, Math.abs(local.x - box.x) - box.radius), Math.max(0, Math.abs(local.z - box.z) - box.radius)));
        }
        double headingError = VehicleSummonFlight.clampAngle(vehicle.orientation.angles.y - route.goal.yaw);
        return String.format("Summon stop (%s): side %.0f deg off (saw %.0f, wanted %.0f), gap %.2f (wanted %.2f), %.2f blocks %s, %.2f to the %s, heading %.1f deg off, still at %.1f m/s", reason, VehicleSummonFlight.clampAngle(seenAngle - arrivalAngle), VehicleSummonFlight.clampAngle(seenAngle), arrivalAngle, seenGap, gap, Math.abs(command.remaining), command.remaining >= 0 ? "short" : "past", Math.abs(command.crossTrack), command.crossTrack >= 0 ? "left" : "right", headingError, speed);
    }

    /**
     * How the drive went: how long it took against the plan, how far off the route it got, how fast it went and set off
     * against the plan, how often it was at full lock (the route turned too tight for it) or well under the planned
     * speed (it couldn't keep up), and how many times it had to plan again.
     */
    String getDrive(int replans) {
        double seconds = ticks / 20D;
        return String.format("Summon drive: %.1f blocks in %.1f s (planned %.1f s), off route avg %.2f max %.2f, top %.1f of %.1f m/s, worst over speed %.1f m/s, set off at %.1f m/s (planned %.1f), full lock %.0f%%, behind speed %.0f%%, replans %d", routeLength, seconds, plannedSeconds, ticks > 0 ? totalCrossTrack / ticks : 0, worstCrossTrack, topSpeed, cruiseSpeed, worstSpeedError, Double.isNaN(startSpeed) ? 0 : startSpeed, plannedStartSpeed, getPercent(fullLockTicks), getPercent(slowTicks), replans);
    }

    private double getPercent(int count) {
        return ticks > 0 ? 100D * count / ticks : 0;
    }
}
