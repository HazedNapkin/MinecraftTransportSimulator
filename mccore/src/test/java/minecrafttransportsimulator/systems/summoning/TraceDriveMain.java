package minecrafttransportsimulator.systems.summoning;

/**
 * Prints a drive tick by tick, for working out why it went as it did.  Run it with a stop, as the
 * {@link SummonScorecardTest} names them without "stop/", and placed or standing: "car placed".  Or with a place and a
 * vehicle, as {@link SummonScenarios} names them, to plan a summon there and print its drive: "bay bike", "fuzz12 car".
 */
public final class TraceDriveMain {
    private TraceDriveMain() {
    }

    public static void main(String[] args) {
        String first = args.length > 0 ? args[0] : "car";
        String second = args.length > 1 ? args[1] : "placed";
        SimDrive.Setup setup = new SimDrive.Setup();
        setup.trace = System.out;
        SimDrive.Result result = null;
        if (second.equals("placed") || second.equals("standing")) {
            for (SummonScorecardTest.Stop stop : SummonScorecardTest.getStops()) {
                if (stop.name.equals(first)) {
                    result = SimDrive.run(SummonScorecardTest.getStraightRoute(30, second.equals("placed")), stop.kinematics, stop.dynamics, setup);
                }
            }
        } else {
            SummonScenarios.Place place = null;
            for (SummonScenarios.Place each : SummonScenarios.places()) {
                if (each.name.equals(first)) {
                    place = each;
                }
            }
            if (place == null && first.startsWith("fuzz")) {
                place = SummonScenarios.fuzz(Long.parseLong(first.substring(4)));
            }
            if (place == null) {
                System.out.println("No place called " + first);
                return;
            }
            SummonScenarios.Vehicle vehicle = SummonScenarios.vehicle(second);
            SummonScenarios.Plan plan = SummonScenarios.plan(place, vehicle, Long.MAX_VALUE);
            if (plan.route == null) {
                System.out.println("No route");
                return;
            }
            for (int i = 0; i < plan.route.poses.size(); ++i) {
                SummonRoute.Pose pose = plan.route.poses.get(i);
                System.out.println(String.format("POSE  %3d at %.2f, %d, %.2f facing %.1f curvature %.3f%s%s", i, pose.x, pose.y, pose.z, pose.yaw, pose.curvature, pose.reverse ? " reverse" : "", pose.pivot ? " pivot" : ""));
            }
            setup.terrain = place.terrain;
            setup.shape = vehicle.shape;
            setup.player = place.player;
            result = SimDrive.run(plan.route, vehicle.kinematics, vehicle.dynamics.copy().skidSteer(place.skidSteer), setup);
        }
        if (result == null) {
            System.out.println("Nothing called " + first + " " + second);
            return;
        }
        System.out.println(String.format("%s after %.2fs: overshoot %.3f, to the side %.3f, yaw off %.2f, parked at %.2f m/s, worst stray %.2f, clipped %d ticks", result.outcome, result.seconds, result.overshoot, result.sideError, result.yawError, result.parkSpeed, result.worstCrossTrack, result.clippedTicks));
    }
}
