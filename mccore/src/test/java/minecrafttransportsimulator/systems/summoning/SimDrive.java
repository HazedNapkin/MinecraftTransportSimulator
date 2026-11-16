package minecrafttransportsimulator.systems.summoning;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Drives a route the way the {@link VehicleSummonJob} does, with the same {@link SpeedProfile}, {@link PathTracker}
 * and {@link DriveControls} and the same settings for them, but on a {@link SimVehicle} rather than a real one, and
 * measures how it went.  This is the nearest a test gets to a summon in game: the route is driven with the throttle,
 * brake and gears, by a vehicle that doesn't do exactly as it's told.
 * <p>
 * What it measures is what a player notices: whether the vehicle got there, how far from where it should have stopped
 * and how squarely, how near it came to them, whether it cut through anything on the way, how long it took, and how
 * smoothly it was driven.  It also counts the control changes the job would send to clients.
 */
final class SimDrive {
    /**As the job has them: how often it checks the vehicle's getting somewhere, in ticks, and how far it must have got, in blocks.**/
    private static final int STUCK_CHECK_INTERVAL = 100;
    private static final double STUCK_DISTANCE = 1.5;
    /**Longest a drive may take, in ticks, and longest it may roll on once the parking brake's on.**/
    private static final int MAX_TICKS = 20 * 120;
    private static final int MAX_ROLL_TICKS = 20 * 10;
    /**The least a control must change for the job to send it to clients.**/
    private static final double SEND_THRESHOLD = 0.005;

    /**How a drive ended.**/
    enum Outcome {
        ARRIVED,
        /**It came to rest just short of the end, which the job counts as arriving.**/
        STALLED_SHORT,
        STUCK,
        TIMED_OUT
    }

    /**Where a drive happens, and how it sets off.**/
    static final class Setup {
        /**The world, to check the vehicle keeps clear of it, or null not to check.**/
        TerrainQuery terrain;
        /**The vehicle's shape, for checking it keeps clear of the world and the player.  Needed if either is set.**/
        VehicleShape shape;
        /**The player it's coming to, to measure how near it comes to them, or null not to.**/
        Point3D player;
        /**How far to the side of the start of the route, and how far turned from it, the vehicle sets off.**/
        double sideOffset;
        double yawOffset;
        double cruiseSpeed = SummonScenarios.getCruiseSpeed();
        /**Where to print what happens each tick, for working out why a drive went as it did, or null not to.**/
        java.io.PrintStream trace;
    }

    /**What happened on a drive.**/
    static final class Result {
        Outcome outcome;
        /**Where it came to rest.**/
        double x;
        double z;
        double yaw;
        /**How far from the end of the route it came to rest, across the ground, in blocks.**/
        double stopError;
        /**How far past the end it came to rest, along the way the route ends, negative if short, and how far to the side.**/
        double overshoot;
        double sideError;
        /**How far its heading is from the end of the route's, in degrees.**/
        double yawError;
        /**How fast it was going when the parking brake went on, in m/s, and how far it rolled after.**/
        double parkSpeed;
        double rollOut;
        /**The furthest it strayed from the route to the side, in blocks.**/
        double worstCrossTrack;
        /**How long it took, and how long the speeds planned for the route would have taken, in seconds.**/
        double seconds;
        double plannedSeconds;
        double topSpeed;
        /**The hardest it slowed down, in m/s per second.**/
        double peakBraking;
        /**How many times it went from throttle to brake or back, and how far the steering was turned in all, in degrees of rudder.**/
        int pedalSwitches;
        double steeringTravel;
        /**How many control changes the job would have sent to clients.**/
        int controlsSent;
        /**How many ticks any part of it was where the planner says it doesn't fit, if a world was given.**/
        int clippedTicks;
        /**The nearest any part of it came to the player on the way, and where it came to rest, in blocks, if a player was given.**/
        double nearestToPlayer = Double.MAX_VALUE;
        double restGap = Double.MAX_VALUE;
        /**How near the player the vehicle would be stopped exactly at the end of the route, in blocks, if a player was given.**/
        double plannedGap = Double.MAX_VALUE;
        int shifts;

        boolean arrived() {
            return outcome == Outcome.ARRIVED || outcome == Outcome.STALLED_SHORT;
        }
    }

    private SimDrive() {
    }

    /**Drives the route with a vehicle that steers by the kinematics and speeds up and stops by the spec.**/
    static Result run(SummonRoute route, VehicleKinematics kinematics, SimVehicle.Spec spec, Setup setup) {
        //Planned and tracked as the job does.
        SpeedProfile.Settings profileSettings = new SpeedProfile.Settings();
        profileSettings.cruiseSpeed = setup.cruiseSpeed;
        profileSettings.steeringMargin = VehicleSummonJob.TURN_RADIUS_MARGIN;
        profileSettings.braking = VehicleSummonJob.BRAKING;
        profileSettings.minSpeed = VehicleSummonJob.MIN_SPEED;
        SpeedProfile profile = SpeedProfile.plan(route, kinematics, profileSettings, route.placed ? Double.MAX_VALUE : 0);
        PathTracker.Settings trackerSettings = new PathTracker.Settings();
        trackerSettings.braking = VehicleSummonJob.BRAKING;
        PathTracker tracker = new PathTracker(route, profile, kinematics, trackerSettings);

        //A placed vehicle sets off already moving at the speed planned for there, its engine just started, in neutral.
        SummonRoute.Pose start = route.getStart();
        double startYaw = start.yaw + setup.yawOffset;
        SimVehicle vehicle = new SimVehicle(spec, kinematics, start.x + Math.cos(Math.toRadians(start.yaw)) * setup.sideOffset, start.z - Math.sin(Math.toRadians(start.yaw)) * setup.sideOffset, startYaw, route.placed ? profile.speeds[0] : 0);

        Result result = new Result();
        result.plannedSeconds = getPlannedSeconds(profile);
        PoseChecker checker = setup.terrain != null ? new PoseChecker(new TerrainMap(setup.terrain), setup.shape) : null;
        ArrivalPlanner gaps = setup.player != null ? new ArrivalPlanner(new PoseChecker(new TerrainMap(setup.terrain != null ? setup.terrain : FakeTerrain.flat(1)), setup.shape), new ArrivalPlanner.Settings(), setup.player, 0) : null;
        int groundY = start.y;
        DriveControls driver = new DriveControls(VehicleSummonJob.BRAKING);
        double sentRudder = 0;
        double sentThrottle = 0;
        double sentBrake = 0;
        int pedal = 0;
        double lastProgressX = vehicle.x;
        double lastProgressZ = vehicle.z;
        int directionChangeTick = 0;
        result.outcome = Outcome.TIMED_OUT;
        int ticks;
        for (ticks = 1; ticks <= MAX_TICKS; ++ticks) {
            double speed = vehicle.getSpeed();
            PathTracker.Command command = tracker.update(vehicle.x, vehicle.z, vehicle.yaw, speed);
            result.worstCrossTrack = Math.max(result.worstCrossTrack, Math.abs(command.crossTrack));
            if (command.changedDirection) {
                directionChangeTick = ticks;
                lastProgressX = vehicle.x;
                lastProgressZ = vehicle.z;
            }
            boolean shifting = false;
            if (!command.pivot) {
                shifting = DriveControls.isShifting(vehicle.gear, command.reverse);
                DriveControls.Shift shift = DriveControls.getShift(vehicle.gear, command.reverse, speed);
                if (shift != DriveControls.Shift.NONE) {
                    vehicle.shift(shift);
                    ++result.shifts;
                }
            }
            DriveControls.Controls controls = driver.update(command, speed, vehicle.goingInReverse, shifting);
            if (controls.park) {
                result.outcome = "arrived".equals(controls.parkReason) ? Outcome.ARRIVED : Outcome.STALLED_SHORT;
                break;
            }
            if (command.pivot) {
                directionChangeTick = ticks;
                if (controls.neutral) {
                    vehicle.parkingBrake = false;
                    vehicle.gear = 0;
                }
            }
            if (controls.straightened) {
                tracker.setRudder(0);
            }

            //What the job would send, and how smoothly it's being driven.
            if (Math.abs(controls.rudder - sentRudder) > SEND_THRESHOLD) {
                result.steeringTravel += Math.abs(controls.rudder - sentRudder);
                sentRudder = controls.rudder;
                ++result.controlsSent;
            }
            if (Math.abs(controls.throttle - sentThrottle) > SEND_THRESHOLD) {
                sentThrottle = controls.throttle;
                ++result.controlsSent;
            }
            if (Math.abs(controls.brake - sentBrake) > SEND_THRESHOLD) {
                sentBrake = controls.brake;
                ++result.controlsSent;
            }
            int pedalNow = sentBrake > 0 ? -1 : sentThrottle > 0 ? 1 : 0;
            if (pedalNow != 0 && pedal != 0 && pedalNow != pedal) {
                ++result.pedalSwitches;
            }
            pedal = pedalNow != 0 ? pedalNow : pedal;

            if (setup.trace != null) {
                setup.trace.println(String.format("TRACE %4d along %6.2f left %6.2f speed %5.2f target %5.2f wanted %6.2f throttle %4.2f brake %4.2f rudder %6.2f cross %5.2f learnt brake %5.2f coast %5.2f", ticks, command.along, command.legRemaining, speed, command.targetSpeed, driver.getWantedAcceleration(command, speed), sentThrottle, sentBrake, sentRudder, command.crossTrack, driver.getBraking(), driver.getCoasting()));
            }
            vehicle.step(sentRudder, sentThrottle, sentBrake);
            result.topSpeed = Math.max(result.topSpeed, vehicle.getSpeed());
            result.peakBraking = Math.max(result.peakBraking, (speed - vehicle.getSpeed()) / SimVehicle.TICK);
            groundY = measure(result, vehicle, checker, gaps, groundY);

            if (ticks % STUCK_CHECK_INTERVAL == 0) {
                boolean stuck = ticks - directionChangeTick >= STUCK_CHECK_INTERVAL && Math.hypot(vehicle.x - lastProgressX, vehicle.z - lastProgressZ) < STUCK_DISTANCE;
                lastProgressX = vehicle.x;
                lastProgressZ = vehicle.z;
                if (stuck) {
                    result.outcome = Outcome.STUCK;
                    break;
                }
            }
        }
        result.seconds = ticks * SimVehicle.TICK;

        //Parked: the controls are let go of and the parking brake's put on, and it rolls to rest.
        result.parkSpeed = vehicle.getSpeed();
        double parkX = vehicle.x;
        double parkZ = vehicle.z;
        vehicle.parkingBrake = true;
        for (int roll = 0; roll < MAX_ROLL_TICKS && vehicle.getSpeed() > 0; ++roll) {
            vehicle.step(0, 0, 0);
            groundY = measure(result, vehicle, checker, gaps, groundY);
        }
        result.rollOut = Math.hypot(vehicle.x - parkX, vehicle.z - parkZ);
        SummonRoute.Pose end = route.getEnd();
        double endSin = Math.sin(Math.toRadians(end.yaw));
        double endCos = Math.cos(Math.toRadians(end.yaw));
        result.x = vehicle.x;
        result.z = vehicle.z;
        result.yaw = vehicle.yaw;
        result.stopError = Math.hypot(vehicle.x - end.x, vehicle.z - end.z);
        result.overshoot = (vehicle.x - end.x) * endSin + (vehicle.z - end.z) * endCos;
        result.sideError = Math.abs((vehicle.x - end.x) * endCos - (vehicle.z - end.z) * endSin);
        result.yawError = Math.abs(VehicleSummonFlight.clampAngle(vehicle.yaw - end.yaw));
        if (gaps != null) {
            result.restGap = gaps.getNearestDistance(vehicle.x, vehicle.z, vehicle.yaw);
            result.plannedGap = gaps.getNearestDistance(end.x, end.z, end.yaw);
        }
        return result;
    }

    /**Notes how near the player the vehicle is and whether it fits where it is, and returns the height of the ground under it.**/
    private static int measure(Result result, SimVehicle vehicle, PoseChecker checker, ArrivalPlanner gaps, int groundY) {
        if (gaps != null) {
            result.nearestToPlayer = Math.min(result.nearestToPlayer, gaps.getNearestDistance(vehicle.x, vehicle.z, vehicle.yaw));
        }
        if (checker != null) {
            int y = checker.terrain.findSurface((int) Math.floor(vehicle.x), (int) Math.floor(vehicle.z), groundY, 1, 2, checker.shape.top - checker.shape.bottom);
            if (y == TerrainMap.NO_SURFACE) {
                ++result.clippedTicks;
            } else {
                groundY = y;
                if (!checker.canDrive(vehicle.x, y, vehicle.z, vehicle.yaw)) {
                    ++result.clippedTicks;
                }
            }
        }
        return groundY;
    }

    /**How long the speeds planned along the route take to drive, in seconds, not counting stops to change direction.**/
    private static double getPlannedSeconds(SpeedProfile profile) {
        double seconds = 0;
        for (int i = 1; i < profile.distances.length; ++i) {
            double speed = (profile.speeds[i] + profile.speeds[i - 1]) / 2;
            if (speed > 0) {
                seconds += (profile.distances[i] - profile.distances[i - 1]) / speed;
            }
        }
        return seconds;
    }
}
