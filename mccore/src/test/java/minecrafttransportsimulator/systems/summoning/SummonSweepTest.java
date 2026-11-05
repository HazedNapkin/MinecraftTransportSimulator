package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Plans and drives summons for a spread of vehicles over a spread of terrain, and prints how each went, for tuning:
 * how long planning took, how much it searched, how many questions it asked the world, and how the route drove.
 * It fails only on things that should never happen: a route that can't be driven, or one not found where there's
 * plenty of room.
 */
public class SummonSweepTest {
    /**How much wider than its tightest turn at walking pace a vehicle's planned turns are, as the summon job plans them.**/
    private static final double TURN_RADIUS_MARGIN = 1.25;
    private static final double TURN_SPEED = 3;

    /**A kind of vehicle to summon.**/
    private static final class Vehicle {
        final String name;
        final VehicleShape shape;
        final VehicleKinematics kinematics;

        /**
         * A box of a vehicle, length by width by height, its middle centerZ ahead of its origin, with a wheel near each
         * corner, wheelbase apart, that steers with the wheelbase.
         */
        Vehicle(String name, double length, double width, double height, double centerZ, double wheelbase) {
            this.name = name;
            double radius = width / 2;
            int boxes = Math.max(1, (int) Math.ceil(length / width));
            List<double[]> raw = new ArrayList<>();
            for (int i = 0; i < boxes; ++i) {
                double z = boxes == 1 ? 0 : -length / 2 + radius + (length - width) * i / (boxes - 1);
                raw.add(new double[] { 0, 0.3 + (height - 0.3) / 2, centerZ + z, radius, (height - 0.3) / 2 });
            }
            double wheelX = Math.max(0.1, width / 2 - 0.25);
            List<double[]> wheels = Arrays.asList(new double[] { wheelX, 0.3, centerZ + wheelbase / 2, 0.25, 0.3 }, new double[] { -wheelX, 0.3, centerZ + wheelbase / 2, 0.25, 0.3 }, new double[] { wheelX, 0.3, centerZ - wheelbase / 2, 0.25, 0.3 }, new double[] { -wheelX, 0.3, centerZ - wheelbase / 2, 0.25, 0.3 });
            this.shape = VehicleShape.of(raw, wheels);
            this.kinematics = new VehicleKinematics(wheelbase, 0.35, 0, false);
        }
    }

    private static final Vehicle[] VEHICLES = {
            new Vehicle("car", 4.2, 1.9, 1.5, 0, 2.6),
            new Vehicle("bus", 11, 2.5, 3, 0, 6),
            new Vehicle("bike", 2, 0.8, 1.4, 0, 1.4),
            //Tracked vehicles skid steer, which MTS treats as a wheelbase of 1.
            new Vehicle("tank", 6, 3.2, 2.4, 0, 1),
            new Vehicle("truck", 7, 2.4, 3, 2.1, 4.2) };

    /**A place to summon to, with the player standing at the origin, looking along +Z.**/
    private static final class Place {
        final String name;
        final FakeTerrain terrain;
        final Point3D player;
        /**True if there's plenty of room, so every vehicle should find a route.**/
        final boolean roomy;
        /**Where the vehicle already is, if it sets off from there rather than being placed, or null.**/
        SummonRoute.Pose start;
        /**The vehicles to summon here, or null for all of them.**/
        List<String> only;
        /**True if the vehicles skid steer, so turn on the spot rather than backing up.**/
        boolean skidSteer;

        Place(String name, FakeTerrain terrain, double playerY, boolean roomy) {
            this.name = name;
            this.terrain = terrain;
            this.player = new Point3D(0.5, playerY, 0.5);
            this.roomy = roomy;
        }

        /**Makes the vehicle set off from where it is, for only the vehicles named.**/
        Place from(SummonRoute.Pose start, String... only) {
            this.start = start;
            this.only = Arrays.asList(only);
            return this;
        }

        /**Makes the vehicles skid steer.**/
        Place skidSteering() {
            skidSteer = true;
            return this;
        }

        boolean summons(Vehicle vehicle) {
            return only == null || only.contains(vehicle.name);
        }
    }

    private static List<Place> getPlaces() {
        List<Place> places = new ArrayList<>();
        places.add(new Place("flat", FakeTerrain.flat(80), 0, true));

        //A hill stepping up a block every 4 blocks in towards the player, 4 steps high.
        FakeTerrain hill = FakeTerrain.flat(80);
        for (int step = 0; step < 4; ++step) {
            int edge = 24 - 4 * step;
            hill.fill(-edge, step, -edge, edge, step, edge);
        }
        places.add(new Place("hill", hill, 4, true));

        //A town of 8 block square buildings 5 high, with 6 block wide streets, the player at a crossroads.
        FakeTerrain town = FakeTerrain.flat(80);
        for (int x = -84; x <= 80; x += 14) {
            for (int z = -84; z <= 80; z += 14) {
                town.fill(x + 3, 0, z + 3, x + 10, 4, z + 10);
            }
        }
        places.add(new Place("town", town, 0, false));

        //A forest of single trunks, kept off the player.
        FakeTerrain forest = FakeTerrain.flat(80);
        Random random = new Random(7);
        for (int i = 0; i < 500; ++i) {
            int x = random.nextInt(121) - 60;
            int z = random.nextInt(121) - 60;
            if (Math.abs(x) > 3 || Math.abs(z) > 3) {
                forest.fill(x, 0, z, x, 4, z);
            }
        }
        places.add(new Place("forest", forest, 0, false));

        //Walls 12 out all round the player, with one gap on the +X side.
        places.add(new Place("yard", FakeTerrain.flat(80).fill(-12, 0, -12, 12, 2, -12).fill(-12, 0, 12, 12, 2, 12).fill(-12, 0, -12, -12, 2, 12).fill(12, 0, -12, 12, 2, -4).fill(12, 0, 4, 12, 2, 12), 0, false));

        //The player at the closed end of a short passage, with vehicles placed to drive in.
        places.add(new Place("passage", passage(), 0, false));

        //Vehicles already nearby: in the open, nosed into an alley, in a street too narrow to turn round in, and out in
        //the open coming to a player at the end of a short passage.
        places.add(new Place("open", FakeTerrain.flat(80), 0, true).from(new SummonRoute.Pose(-14.5, 0, 16.5, 90, 0), "car", "bus", "tank"));
        places.add(new Place("alley", RoutePlannerTest.alley(), 0, true).from(new SummonRoute.Pose(-27.5, 0, 17, -90, 0), "car"));
        places.add(new Place("street", RoutePlannerTest.narrowStreet(), 0, true).from(new SummonRoute.Pose(-40.5, 0, 14.5, -90, 0), "car"));
        places.add(new Place("cramped", passage(), 0, true).from(new SummonRoute.Pose(0.5, 0, 34.5, 0, 0), "car", "bike"));

        //Skid steering vehicles nosed into a bay too narrow to turn round in, and out in the open.
        places.add(new Place("bay", FakeTerrain.flat(80).fill(-31, 0, 13, -14, 2, 13).fill(-31, 0, 21, -14, 2, 21).fill(-31, 0, 13, -31, 2, 21), 0, true).from(new SummonRoute.Pose(-26.5, 0, 17.5, -90, 0), "car", "bike").skidSteering());
        places.add(new Place("skidopen", FakeTerrain.flat(80), 0, true).from(new SummonRoute.Pose(-14.5, 0, 16.5, -90, 0), "car", "tank").skidSteering());
        return places;
    }

    /**A passage 7 blocks wide and 14 long, closed behind the player at the origin, opening out along +Z onto flat ground.**/
    static FakeTerrain passage() {
        return FakeTerrain.flat(80).fill(-4, 0, -3, -4, 3, 12).fill(4, 0, -3, 4, 3, 12).fill(-4, 0, -3, 4, 3, -3);
    }

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

    private static Result run(Place place, Vehicle vehicle, List<String> failures) {
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.turnRadius = TURN_RADIUS_MARGIN * vehicle.kinematics.getTurnRadius(TURN_SPEED);
        settings.allowReverse = place.start != null && !place.skidSteer;
        settings.allowPivot = place.skidSteer;
        CountingTerrain counter = new CountingTerrain(place.terrain);
        RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(counter), vehicle.shape), vehicle.kinematics, settings, place.player, 0);
        if (place.start != null) {
            planner.setStart(place.start.x, place.start.y, place.start.z, place.start.yaw);
        }
        long start = System.nanoTime();
        int ticks = 0;
        while (!planner.step(10_000_000)) {
            ++ticks;
        }
        Result result = new Result();
        result.millis = (System.nanoTime() - start) / 1e6;
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
        List<Place> places = getPlaces();
        //Once round to warm up, so the times are nearer what a running server sees.
        for (Place place : places) {
            for (Vehicle vehicle : VEHICLES) {
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
        for (Place place : places) {
            for (Vehicle vehicle : VEHICLES) {
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
