package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.junit.Test;

/**
 * Scores summoning over every vehicle at every place, and fails if anything it guards has got worse than the scores
 * kept from before, in src/test/resources/summoning/scorecard-baseline.tsv.  It's what to run before and after a
 * change to summoning to see what the change did: what got cheaper, what got more exact, and what it cost elsewhere.
 * <p>
 * Each summon is planned all at once, so what's counted doesn't depend on timing, then driven by a {@link SimVehicle}
 * with the controls the job uses.  Planning is scored by what it found and what finding it cost: states searched and
 * questions asked of the world, which is what costs time in game, and by how many places along the route the vehicle
 * brushes through something between the poses it was checked at.  Driving is scored by how it'd look to the player:
 * whether it got there, how far off where it should stop and how squarely, how fast it was still going when it was
 * parked, how near it came to them, whether it cut through anything, how long it took and how smoothly it was driven.
 * The memory planning allocated is kept to look at, but never fails it, as it varies a little from run to run and
 * from JVM to JVM.  Times vary far more, so they're not kept at all: the {@link SummonSweepTest} prints them.
 * <p>
 * Besides the places, the stops are scored on their own: a straight drive to a stop, by vehicles with weak brakes,
 * slow engines, on slopes and that creep, as stopping in the right place is what the controls find hardest.  And a
 * batch of made-up places is scored together, for what no place was written for.
 * <p>
 * The scores from each run are written to build/summon-scorecard.tsv, or wherever -Dsummon.scorecard.out says.  When a
 * change makes things better, or worse in a way that's meant, copy that over the baseline to keep it.  Scores are
 * exact for the same code on the same JVM, and the slack each metric has covers the small differences between JVMs.
 */
public class SummonScorecardTest {
    private static final String BASELINE = "/summoning/scorecard-baseline.tsv";
    /**How many made-up places are scored, and the vehicles that are summoned to each.**/
    private static final int FUZZ_PLACES = 30;
    private static final String[] FUZZ_VEHICLES = { "car", "truck" };
    /**What's added up over every scenario to show at a glance, and what the worst of is shown.**/
    private static final String[] SUMMED = { "found", "hidden", "arrived", "expansions", "worldCalls", "misfits", "clipped", "controlsSent", "planMb" };
    private static final String[] WORST = { "stopError", "parkSpeed", "yawError", "gapLost", "stray" };

    private static Scorecard newScorecard() {
        return new Scorecard()
                //Planning: what it found, and what finding it cost.
                .higher("found", 0, 0).higher("hidden", 0, 0).lower("expansions", 20, 0.15).lower("worldCalls", 200, 0.1).lower("raycasts", 10, 0.15).lower("cusps", 0, 0).lower("misfits", 0, 0)
                .info("length").info("planMb")
                //Driving: where it ended up.
                .higher("arrived", 0, 0).lower("stopError", 0.03, 0).lower("yawError", 0.3, 0).lower("parkSpeed", 0.1, 0).lower("gapLost", 0.05, 0)
                .info("overshoot").info("sideError").info("plannedGap")
                //Driving: how it got there.
                .lower("stray", 0.03, 0).lower("clipped", 0, 0).higher("nearestToPlayer", 0.05, 0).lower("seconds", 0.3, 0.03).lower("pedalSwitches", 1, 0).lower("steeringTravel", 5, 0.1).lower("controlsSent", 5, 0.05)
                .info("late").info("peakBraking");
    }

    @Test
    public void scorecard() throws IOException {
        Scorecard scorecard = newScorecard();
        scorePlaces(scorecard);
        scoreStops(scorecard);
        scoreFuzz(scorecard);

        File out = new File(System.getProperty("summon.scorecard.out", "build/summon-scorecard.tsv")).getAbsoluteFile();
        if (out.getParentFile().isDirectory() || out.getParentFile().mkdirs()) {
            Files.write(out.toPath(), scorecard.format().getBytes(StandardCharsets.UTF_8));
        }
        System.out.println("SCORE written to " + out);
        for (String metric : SUMMED) {
            System.out.println(String.format("SCORE total %-12s %10.0f", metric, sum(scorecard.getAll(metric))));
        }
        for (String metric : WORST) {
            System.out.println(String.format("SCORE worst %-12s %10.3f", metric, Collections.max(scorecard.getAll(metric))));
        }

        Map<String, Map<String, Double>> baseline = Scorecard.read(SummonScorecardTest.class.getResourceAsStream(BASELINE));
        assertNotNull("No baseline at src/test/resources" + BASELINE + ": copy " + out + " there to start one", baseline);
        Scorecard.Comparison comparison = scorecard.compare(baseline);
        for (String line : comparison.improvements) {
            System.out.println("SCORE better: " + line);
        }
        for (String line : comparison.added) {
            System.out.println("SCORE new: " + line);
        }
        for (String line : comparison.regressions) {
            System.out.println("SCORE WORSE: " + line);
        }
        if (!comparison.improvements.isEmpty() || !comparison.added.isEmpty()) {
            System.out.println("SCORE to keep these, copy " + out + " over src/test/resources" + BASELINE);
        }
        assertTrue(comparison.regressions.size() + " scores got worse than the baseline, the first of them: " + String.join("; ", comparison.regressions.subList(0, Math.min(8, comparison.regressions.size()))), comparison.regressions.isEmpty());
    }

    /**Every vehicle summoned at every place: planned, then driven if there's a route.**/
    private static void scorePlaces(Scorecard scorecard) {
        for (SummonScenarios.Place place : SummonScenarios.places()) {
            for (SummonScenarios.Vehicle vehicle : SummonScenarios.vehicles()) {
                if (place.summons(vehicle)) {
                    score(scorecard, place.name + "/" + vehicle.name, place, vehicle);
                }
            }
        }
    }

    /**Plans and drives one summon, and scores it under the name.**/
    private static void score(Scorecard scorecard, String name, SummonScenarios.Place place, SummonScenarios.Vehicle vehicle) {
        SummonScenarios.Plan plan = SummonScenarios.plan(place, vehicle, Long.MAX_VALUE);
        scorecard.put(name, "found", plan.route != null ? 1 : 0);
        scorecard.put(name, "expansions", plan.planner.getSearchExpansions());
        scorecard.put(name, "worldCalls", plan.counter.getWorldCalls());
        scorecard.put(name, "raycasts", plan.counter.lines);
        //To a tenth of a megabyte, as it varies a little from run to run.
        scorecard.put(name, "planMb", Math.round(plan.allocatedBytes / 104857.6) / 10D);
        if (plan.route == null) {
            return;
        }
        SummonRoute route = plan.route;
        if (route.placed) {
            SummonRoute.Pose first = route.getStart();
            scorecard.put(name, "hidden", SpawnSelector.isVisible(new PoseChecker(new TerrainMap(place.terrain), vehicle.shape), place.player, first.x, first.y, first.z) ? 0 : 1);
        }
        scorecard.put(name, "length", route.getLength());
        scorecard.put(name, "cusps", route.getCusps().size());
        scorecard.put(name, "misfits", SummonScenarios.countMisfitsBetweenPoses(place, vehicle, route));
        SimDrive.Setup setup = new SimDrive.Setup();
        setup.terrain = place.terrain;
        setup.shape = vehicle.shape;
        setup.player = place.player;
        SimDrive.Result drive = SimDrive.run(route, vehicle.kinematics, vehicle.dynamics.copy().skidSteer(place.skidSteer), setup);
        scoreDrive(scorecard, name, drive);
        scorecard.put(name, "clipped", drive.clippedTicks);
        scorecard.put(name, "nearestToPlayer", drive.nearestToPlayer);
        scorecard.put(name, "plannedGap", drive.plannedGap);
        scorecard.put(name, "gapLost", Math.max(0, drive.plannedGap - drive.restGap));
    }

    private static void scoreDrive(Scorecard scorecard, String name, SimDrive.Result drive) {
        scorecard.put(name, "arrived", drive.arrived() ? 1 : 0);
        scorecard.put(name, "stopError", drive.stopError);
        scorecard.put(name, "overshoot", drive.overshoot);
        scorecard.put(name, "sideError", drive.sideError);
        scorecard.put(name, "yawError", drive.yawError);
        scorecard.put(name, "parkSpeed", drive.parkSpeed);
        scorecard.put(name, "stray", drive.worstCrossTrack);
        scorecard.put(name, "seconds", drive.seconds);
        scorecard.put(name, "late", drive.seconds - drive.plannedSeconds);
        scorecard.put(name, "pedalSwitches", drive.pedalSwitches);
        scorecard.put(name, "steeringTravel", drive.steeringTravel);
        scorecard.put(name, "controlsSent", drive.controlsSent);
        scorecard.put(name, "peakBraking", drive.peakBraking);
    }

    /**
     * A straight drive to a stop, placed moving and from a standstill, by each vehicle and by cars that are hard to
     * stop in the right place one way or another.
     */
    private static void scoreStops(Scorecard scorecard) {
        for (Stop stop : getStops()) {
            for (boolean placed : new boolean[] { true, false }) {
                scoreDrive(scorecard, "stop/" + stop.name + (placed ? "/placed" : "/standing"), stop.drive(placed));
            }
        }
    }

    /**A vehicle to try stopping, by how it steers and how it speeds up and stops.**/
    static final class Stop {
        final String name;
        final VehicleKinematics kinematics;
        final SimVehicle.Spec dynamics;

        Stop(String name, VehicleKinematics kinematics, SimVehicle.Spec dynamics) {
            this.name = name;
            this.kinematics = kinematics;
            this.dynamics = dynamics;
        }

        /**Drives it 30 blocks straight to a stop, placed moving at the start or setting off from a standstill.**/
        SimDrive.Result drive(boolean placed) {
            return SimDrive.run(getStraightRoute(30, placed), kinematics, dynamics, new SimDrive.Setup());
        }
    }

    /**Every vehicle, and cars that are hard to stop in the right place one way or another.**/
    static List<Stop> getStops() {
        List<Stop> stops = new ArrayList<>();
        for (SummonScenarios.Vehicle vehicle : SummonScenarios.vehicles()) {
            stops.add(new Stop(vehicle.name, vehicle.kinematics, vehicle.dynamics));
        }
        SummonScenarios.Vehicle car = SummonScenarios.vehicle("car");
        stops.add(new Stop("weakbrakes", car.kinematics, car.dynamics.copy().brakes(1.5)));
        stops.add(new Stop("strongbrakes", car.kinematics, car.dynamics.copy().brakes(12)));
        stops.add(new Stop("laggy", car.kinematics, car.dynamics.copy().lag(0.8)));
        stops.add(new Stop("gutless", car.kinematics, car.dynamics.copy().power(0.8)));
        stops.add(new Stop("draggy", car.kinematics, car.dynamics.copy().drag(1.5)));
        stops.add(new Stop("creeper", car.kinematics, car.dynamics.copy().creep(0.8)));
        stops.add(new Stop("downhill", car.kinematics, car.dynamics.copy().grade(1.5)));
        stops.add(new Stop("uphill", car.kinematics, car.dynamics.copy().grade(-1.5)));
        return stops;
    }

    /**A straight route along +z, the length in blocks, to the origin, the vehicle placed at its start or already there.**/
    static SummonRoute getStraightRoute(double length, boolean placed) {
        List<SummonRoute.Pose> poses = new ArrayList<>();
        for (double back = length; back >= -1e-9; back -= 0.5) {
            poses.add(new SummonRoute.Pose(0.5, 0, 0.5 - back, 0, 0));
        }
        return new SummonRoute(Collections.unmodifiableList(poses), placed, new ArrivalPlanner.Goal(0.5, 0, 0.5, 0, 8, 0));
    }

    /**The made-up places, scored together for each vehicle, as any one of them says little.**/
    private static void scoreFuzz(Scorecard scorecard) {
        for (String vehicleName : FUZZ_VEHICLES) {
            SummonScenarios.Vehicle vehicle = SummonScenarios.vehicle(vehicleName);
            Scorecard each = newScorecard();
            for (int seed = 1; seed <= FUZZ_PLACES; ++seed) {
                score(each, "fuzz" + seed, SummonScenarios.fuzz(seed), vehicle);
            }
            String name = "fuzz/" + vehicleName;
            for (String metric : new String[] { "found", "hidden", "arrived", "expansions", "worldCalls", "raycasts", "misfits", "clipped", "controlsSent", "planMb" }) {
                scorecard.put(name, metric, sum(each.getAll(metric)));
            }
            if (!each.getAll("arrived").isEmpty()) {
                for (String metric : WORST) {
                    scorecard.put(name, metric, Collections.max(each.getAll(metric)));
                }
                scorecard.put(name, "nearestToPlayer", Collections.min(each.getAll("nearestToPlayer")));
            }
        }
    }

    private static double sum(List<Double> values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum;
    }
}
