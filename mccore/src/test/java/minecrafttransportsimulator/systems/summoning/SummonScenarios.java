package minecrafttransportsimulator.systems.summoning;

import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * The vehicles and places summon tests share, and how a summon is planned for one of each, so every test measures
 * the same things the same way.  Add a vehicle or a place here and the sweep, the scorecard and the invariants all
 * pick it up.
 */
final class SummonScenarios {
    private SummonScenarios() {
    }

    /**A kind of vehicle to summon: its shape, how it steers, and how it speeds up and stops.**/
    static final class Vehicle {
        final String name;
        final VehicleShape shape;
        final VehicleKinematics kinematics;
        final SimVehicle.Spec dynamics;

        /**
         * A box of a vehicle, length by width by height, its middle centerZ ahead of its origin, with a wheel near each
         * corner, wheelbase apart, that steers with the wheelbase.
         */
        Vehicle(String name, double length, double width, double height, double centerZ, double wheelbase, SimVehicle.Spec dynamics) {
            this.name = name;
            this.dynamics = dynamics;
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

    /**
     * The vehicles.  How each speeds up and stops is a stand-in for MTS's physics, not taken from it: a spread from
     * nippy with strong brakes to heavy and slow to respond, all able to do what the drive plans on.  Vehicles that
     * can't are tried on their own, in the scorecard's stops.
     */
    static List<Vehicle> vehicles() {
        return Arrays.asList(
                new Vehicle("car", 4.2, 1.9, 1.5, 0, 2.6, new SimVehicle.Spec().power(3.5).brakes(7).lag(0.15).drag(0.3)),
                new Vehicle("bus", 11, 2.5, 3, 0, 6, new SimVehicle.Spec().power(1.5).brakes(4.5).lag(0.4).drag(0.25)),
                new Vehicle("bike", 2, 0.8, 1.4, 0, 1.4, new SimVehicle.Spec().power(5).brakes(8).lag(0.1).drag(0.4)),
                //Tracked vehicles skid steer, which MTS treats as a wheelbase of 1.
                new Vehicle("tank", 6, 3.2, 2.4, 0, 1, new SimVehicle.Spec().power(2.2).brakes(4.5).lag(0.4).drag(0.5)),
                new Vehicle("truck", 7, 2.4, 3, 2.1, 4.2, new SimVehicle.Spec().power(1.4).brakes(4).lag(0.5).drag(0.2)));
    }

    static Vehicle vehicle(String name) {
        for (Vehicle vehicle : vehicles()) {
            if (vehicle.name.equals(name)) {
                return vehicle;
            }
        }
        throw new IllegalArgumentException(name);
    }

    /**A place to summon to, with the player standing at the origin, looking along +Z.**/
    static final class Place {
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

    static List<Place> places() {
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

    /**
     * A made-up place from the seed, the same every time for the same seed: flat ground scattered with buildings,
     * walls, pillars and platforms a block high to drive up onto, all kept a little off the player at the origin.
     * These are for finding what nobody thought to write a place for.
     */
    static Place fuzz(long seed) {
        Random random = new Random(seed);
        FakeTerrain terrain = FakeTerrain.flat(80);
        for (int i = random.nextInt(4); i > 0; --i) {
            //Platforms go in first, so what's built after stands on or beside them.
            fillClear(terrain, random, 8 + random.nextInt(10), 8 + random.nextInt(10), 0, 0);
        }
        for (int i = 5 + random.nextInt(25); i > 0; --i) {
            fillClear(terrain, random, 2 + random.nextInt(7), 2 + random.nextInt(7), 0, 3);
        }
        for (int i = random.nextInt(8); i > 0; --i) {
            boolean alongX = random.nextBoolean();
            fillClear(terrain, random, alongX ? 6 + random.nextInt(20) : 1, alongX ? 1 : 6 + random.nextInt(20), 0, 2);
        }
        for (int i = random.nextInt(120); i > 0; --i) {
            fillClear(terrain, random, 1, 1, 0, 4);
        }
        return new Place("fuzz" + seed, terrain, 0, false);
    }

    /**Fills a box of the size somewhere at random, from y bottom to top, unless it'd come within 5 blocks of the player.**/
    private static void fillClear(FakeTerrain terrain, Random random, int sizeX, int sizeZ, int bottom, int top) {
        int x = random.nextInt(121) - 60;
        int z = random.nextInt(121) - 60;
        if (x > 5 || x + sizeX < -5 || z > 5 || z + sizeZ < -5) {
            terrain.fill(x, bottom, z, x + sizeX - 1, top, z + sizeZ - 1);
        }
    }

    /**How the summon job sets planning up for the vehicle at the place.**/
    static RoutePlanner.Settings settings(Place place, Vehicle vehicle) {
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.cruiseSpeed = getCruiseSpeed();
        settings.turnRadius = VehicleSummonJob.TURN_RADIUS_MARGIN * vehicle.kinematics.getTurnRadius(VehicleSummonJob.TURN_SPEED);
        settings.allowReverse = place.start != null && !place.skidSteer;
        settings.allowPivot = place.skidSteer;
        return settings;
    }

    /**The speed vehicles drive in at unless they say otherwise, in m/s.**/
    static double getCruiseSpeed() {
        return VehicleSummonJob.DEFAULT_SUMMON_SPEED / 3.6;
    }

    /**A summon planned, and what planning it took.**/
    static final class Plan {
        RoutePlanner planner;
        RoutePlanner.Settings settings;
        /**The route, or null if none was found.**/
        SummonRoute route;
        CountingTerrain counter;
        double millis;
        /**How many slices planning took, and the longest any took, in milliseconds.**/
        int slices;
        double worstSliceMillis;
        /**Memory planning allocated, in bytes, or -1 if this JVM can't say.**/
        long allocatedBytes;
    }

    /**
     * Plans the summon of the vehicle at the place, in slices of sliceNanos as the summon manager gives it, or all at
     * once for {@link Long#MAX_VALUE}.  What it finds mustn't depend on the slices.
     */
    static Plan plan(Place place, Vehicle vehicle, long sliceNanos) {
        Plan plan = new Plan();
        plan.settings = settings(place, vehicle);
        plan.counter = new CountingTerrain(place.terrain);
        long allocatedBefore = getAllocatedBytes();
        long start = System.nanoTime();
        plan.planner = new RoutePlanner(new PoseChecker(new TerrainMap(plan.counter), vehicle.shape), vehicle.kinematics, plan.settings, place.player, 0);
        if (place.start != null) {
            plan.planner.setStart(place.start.x, place.start.y, place.start.z, place.start.yaw);
        }
        boolean done = false;
        while (!done) {
            long sliceStart = System.nanoTime();
            done = plan.planner.step(sliceNanos);
            plan.worstSliceMillis = Math.max(plan.worstSliceMillis, (System.nanoTime() - sliceStart) / 1e6);
            ++plan.slices;
        }
        plan.millis = (System.nanoTime() - start) / 1e6;
        long allocatedAfter = getAllocatedBytes();
        plan.allocatedBytes = allocatedBefore >= 0 && allocatedAfter >= 0 ? allocatedAfter - allocatedBefore : -1;
        plan.route = plan.planner.getRoute();
        return plan;
    }

    /**
     * How many of the gaps between the route's poses the vehicle doesn't fit somewhere in, checked every eighth of a
     * block.  Planning only checks it fits at the poses, half a block apart, so a corner of it can brush through a
     * block between two of them.  Between poses at different heights it may be at either, as the step could be anywhere.
     */
    static int countMisfitsBetweenPoses(Place place, Vehicle vehicle, SummonRoute route) {
        PoseChecker checker = new PoseChecker(new TerrainMap(place.terrain), vehicle.shape);
        int misfits = 0;
        for (int i = 1; i < route.poses.size(); ++i) {
            SummonRoute.Pose last = route.poses.get(i - 1);
            SummonRoute.Pose pose = route.poses.get(i);
            double turn = VehicleSummonFlight.clampAngle(pose.yaw - last.yaw);
            int steps = (int) Math.ceil(Math.hypot(pose.x - last.x, pose.z - last.z) / 0.125);
            for (int step = 1; step < steps; ++step) {
                double t = (double) step / steps;
                double x = last.x + (pose.x - last.x) * t;
                double z = last.z + (pose.z - last.z) * t;
                if (!checker.canDrive(x, last.y, z, last.yaw + turn * t) && !checker.canDrive(x, pose.y, z, last.yaw + turn * t)) {
                    ++misfits;
                    break;
                }
            }
        }
        return misfits;
    }

    /**Memory this thread has allocated so far, in bytes, or -1 if this JVM can't say.**/
    private static long getAllocatedBytes() {
        try {
            java.lang.management.ThreadMXBean threads = ManagementFactory.getThreadMXBean();
            if (threads instanceof com.sun.management.ThreadMXBean) {
                return ((com.sun.management.ThreadMXBean) threads).getThreadAllocatedBytes(Thread.currentThread().getId());
            }
        } catch (Throwable unsupported) {
            //Not every JVM has it.
        }
        return -1;
    }
}
