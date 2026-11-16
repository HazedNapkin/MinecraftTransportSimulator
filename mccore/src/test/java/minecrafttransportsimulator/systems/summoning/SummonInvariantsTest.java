package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

/**
 * What must be true of every summon, whatever the vehicle and wherever it is, checked over every place there is and
 * a batch of made-up ones.  Where the {@link SummonScorecardTest} says how good summoning is, this says what it may
 * never do, so a change that makes it quicker or smoother can't quietly make it wrong.
 * <p>
 * Every route planned must be one the vehicle can drive: it fits at every pose, never turns tighter than it can, never
 * climbs or drops more than it can, keeps off the player, and ends exactly where and how it was meant to pull up.
 * Whether it fits between the poses too is scored rather than checked here, as routes that brush a corner between
 * them are found today: see {@link SummonScenarios#countMisfitsBetweenPoses}.
 * Planning must find the same route however its time is sliced, as in game that depends on what else the server is
 * doing.  And driven by a vehicle that does as the drive plans, it must get there without touching the player.
 */
public class SummonInvariantsTest {
    /**How many made-up places are checked, and the vehicles summoned to each.**/
    private static final int FUZZ_PLACES = 40;
    private static final String[] FUZZ_VEHICLES = { "car", "bus", "tank" };
    /**Furthest apart poses may be, in blocks: a little over the half block routes are checked at.**/
    private static final double MAX_POSE_SPACING = 0.75;
    /**How much tighter than its tightest turn a route may seem to turn between poses, as they're a chord apart.**/
    private static final double CURVATURE_TOLERANCE = 1.05;
    /**Half the width of the player, which no part of the vehicle may come within of where they stand.**/
    private static final double PLAYER_RADIUS = 0.3;

    /**A summon to check: a vehicle at a place.**/
    private static final class Case {
        final SummonScenarios.Place place;
        final SummonScenarios.Vehicle vehicle;

        Case(SummonScenarios.Place place, SummonScenarios.Vehicle vehicle) {
            this.place = place;
            this.vehicle = vehicle;
        }

        String name() {
            return vehicle.name + " at " + place.name;
        }
    }

    private static List<Case> getCases() {
        List<Case> cases = new ArrayList<>();
        for (SummonScenarios.Place place : SummonScenarios.places()) {
            for (SummonScenarios.Vehicle vehicle : SummonScenarios.vehicles()) {
                if (place.summons(vehicle)) {
                    cases.add(new Case(place, vehicle));
                }
            }
        }
        for (int seed = 1; seed <= FUZZ_PLACES; ++seed) {
            SummonScenarios.Place place = SummonScenarios.fuzz(seed);
            for (String vehicle : FUZZ_VEHICLES) {
                cases.add(new Case(place, SummonScenarios.vehicle(vehicle)));
            }
        }
        return cases;
    }

    @Test
    public void everyRouteCanBeDriven() {
        List<String> broken = new ArrayList<>();
        int routes = 0;
        for (Case each : getCases()) {
            SummonScenarios.Plan plan = SummonScenarios.plan(each.place, each.vehicle, Long.MAX_VALUE);
            if (plan.route != null) {
                ++routes;
                checkRoute(each, plan, broken);
            } else if (each.place.roomy) {
                broken.add(each.name() + ": no route, though there's plenty of room");
            }
        }
        System.out.println("INVARIANTS checked " + routes + " routes");
        assertTrue(broken.size() + " things wrong with routes: " + String.join("; ", broken.subList(0, Math.min(10, broken.size()))), broken.isEmpty());
    }

    /**Adds what's wrong with the route planned for the case, if anything, to the list.**/
    private static void checkRoute(Case each, SummonScenarios.Plan plan, List<String> broken) {
        SummonRoute route = plan.route;
        List<SummonRoute.Pose> poses = route.poses;
        String name = each.name();
        PoseChecker checker = new PoseChecker(new TerrainMap(each.place.terrain), each.vehicle.shape);
        ArrivalPlanner gaps = new ArrivalPlanner(new PoseChecker(new TerrainMap(each.place.terrain), each.vehicle.shape), new ArrivalPlanner.Settings(), each.place.player, 0);
        double maxCurvature = 1 / plan.settings.turnRadius;

        //Where it sets off and pulls up.
        SummonRoute.Pose start = route.getStart();
        SummonRoute.Pose end = route.getEnd();
        if (each.place.start != null) {
            if (route.placed || Math.hypot(start.x - each.place.start.x, start.z - each.place.start.z) > 1e-6 || Math.abs(VehicleSummonFlight.clampAngle(start.yaw - each.place.start.yaw)) > 1e-6) {
                broken.add(name + ": doesn't set off from where the vehicle is");
            }
        } else if (!route.placed || !checker.canPark(start.x, start.y, start.z, start.yaw)) {
            broken.add(name + ": can't be placed where it sets off");
        }
        if (route.goal == null || Math.hypot(end.x - route.goal.x, end.z - route.goal.z) > 1e-6 || end.y != route.goal.y || Math.abs(VehicleSummonFlight.clampAngle(end.yaw - route.goal.yaw)) > 1e-6) {
            broken.add(name + ": doesn't end at its goal");
        } else if (!plan.planner.getGoals().contains(route.goal)) {
            broken.add(name + ": ends at a goal that isn't one of the planner's");
        }
        if (end.reverse) {
            broken.add(name + ": backs up to where it pulls up");
        }
        if (!checker.canPark(end.x, end.y, end.z, end.yaw)) {
            broken.add(name + ": can't park where it pulls up");
        }
        if (gaps.getNearestDistance(end.x, end.z, end.yaw) < ArrivalPlanner.TIGHT_GAP - 0.1) {
            broken.add(String.format("%s: pulls up %.2f from the player", name, gaps.getNearestDistance(end.x, end.z, end.yaw)));
        }
        if (route.getCusps().size() + (route.hasPivots() ? 1 : 0) > new HybridSearch.Settings().maxCusps + 1) {
            broken.add(name + ": changes direction too often");
        }
        if (!plan.settings.allowReverse && route.getReverseLength() > 0) {
            broken.add(name + ": backs up, though it may not");
        }
        if (!plan.settings.allowPivot && route.hasPivots()) {
            broken.add(name + ": turns on the spot, though it can't");
        }

        //All the way along.
        for (int i = 0; i < poses.size(); ++i) {
            SummonRoute.Pose pose = poses.get(i);
            if (!checker.canDrive(pose.x, pose.y, pose.z, pose.yaw)) {
                broken.add(String.format("%s: doesn't fit at pose %d of %d", name, i, poses.size()));
                break;
            }
            if (gaps.getNearestDistance(pose.x, pose.z, pose.yaw) < PLAYER_RADIUS) {
                broken.add(String.format("%s: drives through the player at pose %d of %d", name, i, poses.size()));
                break;
            }
            if (i == 0) {
                continue;
            }
            SummonRoute.Pose last = poses.get(i - 1);
            double spacing = Math.hypot(pose.x - last.x, pose.z - last.z);
            double turn = VehicleSummonFlight.clampAngle(pose.yaw - last.yaw);
            if (pose.pivot) {
                if (spacing > 1e-6) {
                    broken.add(String.format("%s: moves while turning on the spot at pose %d", name, i));
                    break;
                }
                continue;
            }
            if (spacing > MAX_POSE_SPACING) {
                broken.add(String.format("%s: poses %d and %d are %.2f apart", name, i - 1, i, spacing));
                break;
            }
            if (spacing > 1e-6 && Math.toRadians(Math.abs(turn)) / spacing > maxCurvature * CURVATURE_TOLERANCE) {
                broken.add(String.format("%s: turns at a radius of %.2f at pose %d, tighter than its %.2f", name, spacing / Math.toRadians(Math.abs(turn)), i, plan.settings.turnRadius));
                break;
            }
            //Driving forward it climbs to the later pose, and backing up it climbs the other way.
            int climb = pose.reverse ? last.y - pose.y : pose.y - last.y;
            if (climb > plan.settings.maxUp || -climb > plan.settings.maxDown) {
                broken.add(String.format("%s: goes from height %d to %d at pose %d", name, last.y, pose.y, i));
                break;
            }
        }
    }

    @Test
    public void planningDoesNotDependOnHowItsTimeIsSliced() {
        //Slices short enough that every stage is cut into many.
        long slice = 200_000;
        for (Case each : getCases().subList(0, 45)) {
            SummonScenarios.Plan whole = SummonScenarios.plan(each.place, each.vehicle, Long.MAX_VALUE);
            SummonScenarios.Plan again = SummonScenarios.plan(each.place, each.vehicle, Long.MAX_VALUE);
            SummonScenarios.Plan sliced = SummonScenarios.plan(each.place, each.vehicle, slice);
            assertSameRoute(each.name() + " planned twice", whole, again);
            assertSameRoute(each.name() + " planned in slices", whole, sliced);
        }
    }

    private static void assertSameRoute(String name, SummonScenarios.Plan expected, SummonScenarios.Plan actual) {
        assertEquals(name + ": found a route", expected.route != null, actual.route != null);
        assertEquals(name + ": states searched", expected.planner.getSearchExpansions(), actual.planner.getSearchExpansions());
        assertEquals(name + ": questions asked of the world", expected.counter.getWorldCalls(), actual.counter.getWorldCalls());
        if (expected.route == null) {
            return;
        }
        assertEquals(name + ": poses", expected.route.poses.size(), actual.route.poses.size());
        for (int i = 0; i < expected.route.poses.size(); ++i) {
            SummonRoute.Pose a = expected.route.poses.get(i);
            SummonRoute.Pose b = actual.route.poses.get(i);
            assertTrue(name + ": pose " + i + " differs", a.x == b.x && a.y == b.y && a.z == b.z && a.yaw == b.yaw && a.curvature == b.curvature && a.reverse == b.reverse && a.pivot == b.pivot);
        }
    }

    @Test
    public void everyDriveArrivesWithoutTouchingThePlayer() {
        List<String> broken = new ArrayList<>();
        for (Case each : getCases()) {
            SummonScenarios.Plan plan = SummonScenarios.plan(each.place, each.vehicle, Long.MAX_VALUE);
            if (plan.route == null) {
                continue;
            }
            SimDrive.Setup setup = new SimDrive.Setup();
            setup.shape = each.vehicle.shape;
            setup.player = each.place.player;
            SimDrive.Result drive = SimDrive.run(plan.route, each.vehicle.kinematics, each.vehicle.dynamics.copy().skidSteer(each.place.skidSteer), setup);
            if (!drive.arrived()) {
                broken.add(each.name() + ": " + drive.outcome);
            } else if (drive.nearestToPlayer < PLAYER_RADIUS) {
                broken.add(String.format("%s: came %.2f from where the player stands", each.name(), drive.nearestToPlayer));
            }
        }
        assertTrue(broken.size() + " drives went wrong: " + String.join("; ", broken.subList(0, Math.min(10, broken.size()))), broken.isEmpty());
    }
}
