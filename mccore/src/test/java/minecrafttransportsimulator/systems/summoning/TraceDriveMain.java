package minecrafttransportsimulator.systems.summoning;

/**
 * Prints a drive tick by tick, for working out why it went as it did.  Run it with the stop to drive, as the
 * {@link SummonScorecardTest} names them without "stop/", and placed or standing: car placed, weakbrakes standing.
 */
public final class TraceDriveMain {
    private TraceDriveMain() {
    }

    public static void main(String[] args) {
        String name = args.length > 0 ? args[0] : "car";
        boolean placed = args.length < 2 || args[1].equals("placed");
        for (SummonScorecardTest.Stop stop : SummonScorecardTest.getStops()) {
            if (stop.name.equals(name)) {
                SimDrive.Setup setup = new SimDrive.Setup();
                setup.trace = System.out;
                SimDrive.Result result = SimDrive.run(SummonScorecardTest.getStraightRoute(30, placed), stop.kinematics, stop.dynamics, setup);
                System.out.println(String.format("%s after %.2fs: overshoot %.3f, parked at %.2f m/s", result.outcome, result.seconds, result.overshoot, result.parkSpeed));
                return;
            }
        }
        System.out.println("No stop called " + name);
    }
}
