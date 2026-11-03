package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Plans and drives summons for a spread of vehicles over a spread of terrain, and prints how each went, for tuning.
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

        Place(String name, FakeTerrain terrain, double playerY, boolean roomy) {
            this.name = name;
            this.terrain = terrain;
            this.player = new Point3D(0.5, playerY, 0.5);
            this.roomy = roomy;
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
        return places;
    }

    @Test
    public void sweep() {
        List<String> failures = new ArrayList<>();
        System.out.println("SWEEP vehicle place   | plan ms  ticks expand | route  length | drive stray   end    yaw   secs   top");
        for (Place place : getPlaces()) {
            for (Vehicle vehicle : VEHICLES) {
                RoutePlanner.Settings settings = new RoutePlanner.Settings();
                settings.turnRadius = TURN_RADIUS_MARGIN * vehicle.kinematics.getTurnRadius(TURN_SPEED);
                RoutePlanner planner = new RoutePlanner(new PoseChecker(new TerrainMap(place.terrain), vehicle.shape), vehicle.kinematics, settings, place.player, 0);
                long start = System.nanoTime();
                int ticks = 0;
                while (!planner.step(10_000_000)) {
                    ++ticks;
                }
                double planMillis = (System.nanoTime() - start) / 1e6;
                SummonRoute route = planner.getRoute();
                String line = String.format("SWEEP %-7s %-7s | %7.0f %6d %6d |", vehicle.name, place.name, planMillis, ticks + 1, planner.getSearchExpansions());
                if (route == null) {
                    line += String.format(" none (%d goals)", planner.getGoals().size());
                    if (place.roomy) {
                        failures.add(vehicle.name + " found no route on " + place.name);
                    }
                } else {
                    DriveSimulator.Drive drive = DriveSimulator.drive(route, vehicle.kinematics, route.placed ? Double.MAX_VALUE : 0, 0, 0, 0);
                    line += String.format(" %s %6.1f | %s %5.2f %6.2f %6.1f %6.1f %5.1f", "found", route.getLength(), drive.arrived ? "ok   " : "FAIL ", drive.worstCrossTrack, drive.getEndError(route), drive.getYawError(route), drive.ticks / 20D, drive.topSpeed);
                    if (!drive.arrived || drive.worstCrossTrack > 0.6 || drive.getEndError(route) > 0.5 || drive.getYawError(route) > 5) {
                        failures.add(vehicle.name + " drove " + place.name + " badly");
                    }
                }
                System.out.println(line);
            }
        }
        assertTrue(String.join(", ", failures), failures.isEmpty());
    }
}
