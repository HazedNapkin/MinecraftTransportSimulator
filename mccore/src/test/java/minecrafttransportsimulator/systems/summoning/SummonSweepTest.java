package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

/**
 * Plans and drives summons for a spread of vehicles over a spread of terrain, and prints how each went, for tuning:
 * how long planning took, how much it searched, how many questions it asked the world, and how the route drove.
 * It fails only on things that should never happen: a route that can't be driven, or one not found where there's
 * plenty of room.
 */
public class SummonSweepTest {
    /**What planning and driving one summon came to.**/
    private static final class Result {
        String line;
        boolean found;
        boolean hidden;
        int worldCalls;
        int raycasts;
        int expansions;
        double millis;
    }

    private static Result run(SummonScenarios.Place place, SummonScenarios.Vehicle vehicle, List<String> failures) {
        //In slices of 10ms, as the summon manager gives a lone summon each tick.
        SummonScenarios.Plan plan = SummonScenarios.plan(place, vehicle, 10_000_000);
        RoutePlanner planner = plan.planner;
        CountingTerrain counter = plan.counter;
        int ticks = plan.slices - 1;
        Result result = new Result();
        result.millis = plan.millis;
        result.worldCalls = counter.getWorldCalls();
        result.raycasts = counter.lines;
        result.expansions = planner.getSearchExpansions();
        SummonRoute route = planner.getRoute();
        String line = String.format("SWEEP %-5s %-7s | %5.0f %3d %5d %5d %5d %4d | %6d %6d %6d %5d |", vehicle.name, place.name, result.millis, ticks + 1, result.expansions, planner.getFirstRouteExpansion(), planner.getGoalSideExpansions(), planner.getFailedJoins(), counter.blocks, counter.clear, result.worldCalls, counter.lines);
        if (route == null) {
            line += String.format(" none (%d goals)", planner.getGoals().size());
            if (place.roomy) {
                failures.add(vehicle.name + " found no route on " + place.name);
            }
        } else {
            result.found = true;
            String seen = "    ";
            if (route.placed) {
                SummonRoute.Pose first = route.getStart();
                result.hidden = !SpawnSelector.isVisible(new PoseChecker(new TerrainMap(place.terrain), vehicle.shape), place.player, first.x, first.y, first.z);
                seen = result.hidden ? "hid " : "seen";
            }
            DriveSimulator.Drive drive = DriveSimulator.drive(route, vehicle.kinematics, route.placed ? Double.MAX_VALUE : 0, 0, 0, 0, place.skidSteer);
            line += String.format(" found %5.1f %s %d %4.1f %3.0f | %s %4.2f %4.2f %4.1f %5.1f %4.1f", route.getLength(), seen, route.getCusps().size(), route.getReverseLength(), route.getPivotDegrees(), drive.arrived ? "ok  " : "FAIL", drive.worstCrossTrack, drive.getEndError(route), drive.getYawError(route), drive.ticks / 20D, drive.topSpeed);
            if (!drive.arrived || drive.worstCrossTrack > 0.6 || drive.getEndError(route) > 0.5 || drive.getYawError(route) > 5) {
                failures.add(vehicle.name + " drove " + place.name + " badly");
            }
        }
        result.line = line;
        return result;
    }

    @Test
    public void sweep() {
        List<SummonScenarios.Place> places = SummonScenarios.places();
        List<SummonScenarios.Vehicle> vehicles = SummonScenarios.vehicles();
        //Once round to warm up, so the times are nearer what a running server sees.
        for (SummonScenarios.Place place : places) {
            for (SummonScenarios.Vehicle vehicle : vehicles) {
                if (place.summons(vehicle)) {
                    run(place, vehicle, new ArrayList<>());
                }
            }
        }
        List<String> failures = new ArrayList<>();
        System.out.println("SWEEP veh   place   |    ms tck expnd first  near fail | blocks  clear  world  rays | route   len seen c back piv | drive stray end  yaw   secs  top");
        int found = 0;
        int hidden = 0;
        long worldCalls = 0;
        long raycasts = 0;
        long expansions = 0;
        double millis = 0;
        for (SummonScenarios.Place place : places) {
            for (SummonScenarios.Vehicle vehicle : vehicles) {
                if (place.summons(vehicle)) {
                    Result result = run(place, vehicle, failures);
                    System.out.println(result.line);
                    found += result.found ? 1 : 0;
                    hidden += result.hidden ? 1 : 0;
                    worldCalls += result.worldCalls;
                    raycasts += result.raycasts;
                    expansions += result.expansions;
                    millis += result.millis;
                }
            }
        }
        System.out.println(String.format("SWEEP total: %d found (%d placed out of sight), %.0f ms, %d expansions, %d world calls, %d raycasts", found, hidden, millis, expansions, worldCalls, raycasts));
        assertTrue(String.join(", ", failures), failures.isEmpty());
    }
}
