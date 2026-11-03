package minecrafttransportsimulator.systems.summoning;

/**
 * Drives a route with a simple vehicle that steers the way MTS ground vehicles do, for testing how well routes can be
 * followed.  It speeds up and brakes at fixed rates, and turns by the curvature its rudder input gives at its speed.
 */
final class DriveSimulator {
    /**How fast the simulated vehicle speeds up and brakes, in m/s per second, and the length of a tick in seconds.**/
    private static final double ACCELERATION = 2.5;
    private static final double BRAKING = 4;
    private static final double TICK = 0.05;
    /**Longest a drive may take, in ticks.**/
    private static final int MAX_TICKS = 20 * 90;

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
        SpeedProfile profile = SpeedProfile.plan(route, kinematics, new SpeedProfile.Settings(), startSpeed);
        PathTracker tracker = new PathTracker(route, profile, kinematics, new PathTracker.Settings());
        SummonRoute.Pose start = route.getStart();
        Drive drive = new Drive();
        drive.yaw = start.yaw + yawOffset;
        drive.x = start.x + Math.cos(Math.toRadians(start.yaw)) * sideOffset;
        drive.z = start.z - Math.sin(Math.toRadians(start.yaw)) * sideOffset;
        drive.speed = profile.speeds[0];
        for (drive.ticks = 0; drive.ticks < MAX_TICKS; ++drive.ticks) {
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
            drive.topSpeed = Math.max(drive.topSpeed, drive.speed);
            double moved = drive.speed * TICK;
            double turn = Math.toDegrees(kinematics.getCurvature(command.rudder, drive.speed) * moved);
            double midYaw = Math.toRadians(drive.yaw + turn / 2);
            drive.x += Math.sin(midYaw) * moved;
            drive.z += Math.cos(midYaw) * moved;
            drive.yaw += turn;
        }
        return drive;
    }
}
