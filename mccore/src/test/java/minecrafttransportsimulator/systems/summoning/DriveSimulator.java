package minecrafttransportsimulator.systems.summoning;

/**
 * Drives a route with a simple vehicle that steers the way MTS ground vehicles do, for testing how well routes can be
 * followed.  It speeds up and brakes at fixed rates, and turns by the curvature its rudder input gives at its speed.
 * It backs up in reverse gear, turning the other way for the same steering as MTS vehicles do, and has to stop to
 * shift between forward and reverse, which takes a moment.  A vehicle that skid steers turns on the spot when it's
 * told to and all but stopped, a twentieth of the steering angle a tick, as MTS turns them.
 */
final class DriveSimulator {
    /**How fast the simulated vehicle speeds up and brakes, in m/s per second, and the length of a tick in seconds.**/
    private static final double ACCELERATION = 2.5;
    private static final double BRAKING = 4;
    private static final double TICK = 0.05;
    /**Longest a drive may take, in ticks.**/
    private static final int MAX_TICKS = 20 * 90;
    /**How slow it must be to shift between forward and reverse, in m/s, and how many ticks shifting takes.**/
    private static final double SHIFT_SPEED = 0.1;
    private static final int SHIFT_TICKS = 10;
    /**How slow a skid steering vehicle must be to turn on the spot, in m/s: MTS's ground velocity of 0.05 at its usual speed factor.**/
    private static final double PIVOT_SPEED = 0.05 * 20 * 0.35;

    /**What happened driving a route.**/
    static final class Drive {
        double x;
        double z;
        double yaw;
        double speed;
        double topSpeed;
        double worstCrossTrack;
        int ticks;
        boolean arrived;
        /**How many times it shifted between forward and reverse, and the furthest it ran past where it should have stopped to.**/
        int shifts;
        double worstOverrun;
        /**How far it turned on the spot in all, in degrees, and how far it moved while doing so, in blocks.**/
        double pivoted;
        double pivotDrift;

        /**How far from the end of the route it stopped, across the ground.**/
        double getEndError(SummonRoute route) {
            return Math.hypot(x - route.getEnd().x, z - route.getEnd().z);
        }

        /**How far its heading is from the end of the route's, in degrees.**/
        double getYawError(SummonRoute route) {
            return Math.abs(VehicleSummonFlight.clampAngle(yaw - route.getEnd().yaw));
        }
    }

    private DriveSimulator() {
    }

    /**
     * Drives the route, starting off to the side and turned by the offsets, and at the speed it was planned to set off
     * at.  Errors are only counted once it's had settleDistance blocks to get onto the route.
     */
    static Drive drive(SummonRoute route, VehicleKinematics kinematics, double startSpeed, double sideOffset, double yawOffset, double settleDistance) {
        return drive(route, kinematics, startSpeed, sideOffset, yawOffset, settleDistance, false);
    }

    /**As {@link #drive(SummonRoute, VehicleKinematics, double, double, double, double)}, turning on the spot if it skid steers.**/
    static Drive drive(SummonRoute route, VehicleKinematics kinematics, double startSpeed, double sideOffset, double yawOffset, double settleDistance, boolean skidSteer) {
        SpeedProfile profile = SpeedProfile.plan(route, kinematics, new SpeedProfile.Settings(), startSpeed);
        PathTracker tracker = new PathTracker(route, profile, kinematics, new PathTracker.Settings());
        SummonRoute.Pose start = route.getStart();
        Drive drive = new Drive();
        drive.yaw = start.yaw + yawOffset;
        drive.x = start.x + Math.cos(Math.toRadians(start.yaw)) * sideOffset;
        drive.z = start.z - Math.sin(Math.toRadians(start.yaw)) * sideOffset;
        drive.speed = profile.speeds[0];
        boolean inReverse = start.reverse;
        int shifting = 0;
        for (drive.ticks = 0; drive.ticks < MAX_TICKS; ++drive.ticks) {
            PathTracker.Command command = tracker.update(drive.x, drive.z, drive.yaw, drive.speed);
            if (command.along > settleDistance) {
                drive.worstCrossTrack = Math.max(drive.worstCrossTrack, Math.abs(command.crossTrack));
            }
            drive.worstOverrun = Math.max(drive.worstOverrun, -command.legRemaining);
            if (command.arrived) {
                drive.arrived = true;
                break;
            }
            double targetSpeed = command.targetSpeed;
            if (command.pivot) {
                //Stop, then turn on the spot in neutral.  Only vehicles that skid steer can.
                drive.speed = Math.max(0, drive.speed - BRAKING * TICK);
                if (skidSteer && drive.speed < PIVOT_SPEED) {
                    double turn = -command.rudder / 20;
                    drive.yaw += turn;
                    drive.pivoted += Math.abs(turn);
                    inReverse = false;
                } else {
                    double moved = (inReverse ? -1 : 1) * drive.speed * TICK;
                    drive.x += Math.sin(Math.toRadians(drive.yaw)) * moved;
                    drive.z += Math.cos(Math.toRadians(drive.yaw)) * moved;
                    drive.pivotDrift += Math.abs(moved);
                }
                continue;
            }
            if (command.reverse != inReverse) {
                //Stop, then shift.
                targetSpeed = 0;
                if (drive.speed < SHIFT_SPEED && ++shifting >= SHIFT_TICKS) {
                    inReverse = command.reverse;
                    shifting = 0;
                    ++drive.shifts;
                }
            }
            double change = targetSpeed - drive.speed;
            drive.speed = Math.max(0, drive.speed + Math.max(-BRAKING * TICK, Math.min(ACCELERATION * TICK, change)));
            drive.topSpeed = Math.max(drive.topSpeed, drive.speed);
            double moved = (inReverse ? -1 : 1) * drive.speed * TICK;
            double turn = Math.toDegrees(kinematics.getCurvature(command.rudder, drive.speed) * moved);
            double midYaw = Math.toRadians(drive.yaw + turn / 2);
            drive.x += Math.sin(midYaw) * moved;
            drive.z += Math.cos(midYaw) * moved;
            drive.yaw += turn;
        }
        return drive;
    }
}
