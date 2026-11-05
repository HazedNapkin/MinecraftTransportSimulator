package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Plans a ground vehicle's drive in to the player, a slice at a time so it never stalls the server.
 * <p>
 * It runs in stages: find where the vehicle pulls up ({@link ArrivalPlanner}), flood out from there to see how far
 * the drive in is from everywhere near ({@link FlowField}), pick good hidden places about the summon distance out to
 * set off from ({@link SpawnSelector}), then search forward from those for the route ({@link HybridSearch}), guided by
 * the flood.  A vehicle that's already nearby skips picking places and searches from where it is, backing up if it can
 * and that's what gets it out, and the flood stops once it's reached it.  If it never does, there's no way in.
 */
final class RoutePlanner {
    /**How much further than the longest drive in the flood spreads from the player, in blocks.**/
    private static final double FIELD_MARGIN = 8;
    /**The least straight run-in a route ends with, in blocks, however short the vehicle.**/
    private static final double MIN_RUN_IN = 4;
    /**The longest drive in to set off from, as a multiple of the summon distance.**/
    private static final double MAX_LENGTH_FACTOR = 1.4;

    /**What the vehicle wants, and how it drives.**/
    static final class Settings {
        final ArrivalPlanner.Settings arrival = new ArrivalPlanner.Settings();
        /**How far the drive in should be, in blocks.**/
        double summonDistance = 30;
        /**Speed it drives in at, in m/s.**/
        double cruiseSpeed = 8.3;
        /**The radius of its tightest turns, with any margin for steering error, in blocks.**/
        double turnRadius = 4;
        int maxUp = 1;
        int maxDown = 2;
        int maxExpansions = 20000;
        /**
         * True if the vehicle can back up.  Only routes from where it already is do, as placed vehicles can be placed
         * facing whichever way they need: see {@link HybridSearch.Settings#allowReverse}.
         */
        boolean allowReverse;
        /**How many times what driving forward costs backing up costs, and the cost of each change of direction, in blocks.**/
        double reverseFactor = 3;
        double cuspCost = 5;
        /**True if the vehicle can turn on the spot, as vehicles that skid steer can, and how fast, in degrees a second.**/
        boolean allowPivot;
        double pivotRate = 45;
    }

    private enum Stage {
        ARRIVAL,
        GOAL_FIELD,
        SPAWNS,
        SEARCH,
        SMOOTH,
        DONE
    }

    private final PoseChecker checker;
    private final VehicleKinematics kinematics;
    private final Settings settings;
    private final Point3D player;
    private final double lookYaw;
    private final double fieldRadius;
    private final ArrivalPlanner arrival;
    private Stage stage = Stage.ARRIVAL;
    private SummonRoute.Pose start;
    private List<ArrivalPlanner.Goal> goals = Collections.emptyList();
    private FlowField goalField;
    private SpawnSelector spawns;
    private HybridSearch search;
    private SummonRoute route;
    /**Time spent in each stage, in nanoseconds, and how many times it's been stepped, for tuning.**/
    private final long[] stageNanos = new long[Stage.values().length];
    private int steps;

    /**Plans for a vehicle of the checker's shape, kinematics and settings to come to the player, who's looking along the yaw.**/
    RoutePlanner(PoseChecker checker, VehicleKinematics kinematics, Settings settings, Point3D player, double lookYaw) {
        this.checker = checker;
        this.kinematics = kinematics;
        this.settings = settings;
        this.player = player.copy();
        this.lookYaw = lookYaw;
        this.fieldRadius = getMaxLength() + FIELD_MARGIN;
        settings.arrival.turnRadius = settings.turnRadius;
        settings.arrival.maxUp = settings.maxUp;
        settings.arrival.maxDown = settings.maxDown;
        settings.arrival.canPivot = settings.allowPivot;
        this.arrival = new ArrivalPlanner(checker, settings.arrival, player, lookYaw);
    }

    /**Makes the route start where the vehicle already is, rather than anywhere it could be placed.  Call before stepping.**/
    void setStart(double x, int y, double z, double yaw) {
        start = new SummonRoute.Pose(x, y, z, yaw, 0);
    }

    /**Plans until it's done or maxNanos pass.  Returns true once it's done, whether it found a route or not.**/
    boolean step(long maxNanos) {
        long startTime = System.nanoTime();
        ++steps;
        while (stage != Stage.DONE) {
            long stageStart = System.nanoTime();
            long left = maxNanos - (stageStart - startTime);
            if (left <= 0) {
                break;
            }
            Stage stepped = stage;
            switch (stage) {
                case ARRIVAL: {
                    if (arrival.step(left)) {
                        goals = arrival.getGoals();
                        if (goals.isEmpty()) {
                            stage = Stage.DONE;
                        } else {
                            startGoalField();
                        }
                    }
                    break;
                }
                case GOAL_FIELD: {
                    if (goalField.step(left)) {
                        if (start != null && goalField.getTargetCell() == null) {
                            //There's no way from where the vehicle is to anywhere it could pull up.
                            stage = Stage.DONE;
                        } else if (start != null) {
                            startSearch(Collections.singletonList(new HybridSearch.Start(start.x, start.y, start.z, start.yaw, 0)));
                        } else {
                            spawns = new SpawnSelector(checker, goalField, goals, getSpawnSettings(), player, lookYaw);
                            stage = Stage.SPAWNS;
                        }
                    }
                    break;
                }
                case SPAWNS: {
                    if (spawns.step(left)) {
                        startSearch(getStarts(spawns.getTargets()));
                    }
                    break;
                }
                case SEARCH: {
                    if (search.step(Integer.MAX_VALUE, left)) {
                        route = search.getRoute();
                        stage = route != null ? Stage.SMOOTH : Stage.DONE;
                    }
                    break;
                }
                case SMOOTH: {
                    //Smooth the run-in no further than it's straight.
                    route = PathSmoother.smooth(route, checker, 1 / settings.turnRadius, Math.min(getSearchSettings().runIn, route.goal.runIn));
                    stage = Stage.DONE;
                    break;
                }
                default:
                    break;
            }
            stageNanos[stepped.ordinal()] += System.nanoTime() - stageStart;
        }
        return stage == Stage.DONE;
    }

    /**How long each stage took and how much it did, for tuning.**/
    String getTimings() {
        StringBuilder timings = new StringBuilder();
        long total = 0;
        for (Stage each : Stage.values()) {
            if (each != Stage.DONE) {
                timings.append(String.format("%s %.0fms, ", each.name().toLowerCase(), stageNanos[each.ordinal()] / 1e6));
                total += stageNanos[each.ordinal()];
            }
        }
        timings.append(String.format("total %.0fms over %d ticks", total / 1e6, steps));
        timings.append(String.format("; cells %d, expansions %d (%d near the goals), joins %d (%d didn't fit)", goalField != null ? goalField.getCellCount() : 0, getSearchExpansions(), getGoalSideExpansions(), search != null ? search.getJoinTries() : 0, getFailedJoins()));
        return timings.toString();
    }

    void finish() {
        while (!step(Long.MAX_VALUE)) {
        }
    }

    /**The route found, or null if there isn't one or it's not done.**/
    SummonRoute getRoute() {
        return route;
    }

    /**Where the vehicle could pull up, best first.  The best is where to bring it if it can't drive in.  Empty if it fits nowhere.**/
    List<ArrivalPlanner.Goal> getGoals() {
        return goals;
    }

    /**What found where the vehicle pulls up.  For debugging.**/
    ArrivalPlanner getArrival() {
        return arrival;
    }

    /**How the player was looking when planning started, as a yaw.  For debugging.**/
    double getLookYaw() {
        return lookYaw;
    }

    /**The places picked to set off from, or null if none were picked.  For debugging.**/
    List<SpawnSelector.Target> getSpawnTargets() {
        return spawns != null ? spawns.getTargets() : null;
    }

    /**How many states the route search expanded, for tuning.**/
    int getSearchExpansions() {
        return search != null ? search.getExpansions() : 0;
    }

    /**How many states the route search expanded near the goals, for tuning.**/
    int getGoalSideExpansions() {
        return search != null ? search.getGoalSideExpansions() : 0;
    }

    /**How many states the route search had expanded when it found its first route, or -1, for tuning.**/
    int getFirstRouteExpansion() {
        return search != null ? search.getFirstRouteExpansion() : -1;
    }

    /**How many joins onto the goals' run-ins the search checked that something was in the way of, for tuning.**/
    int getFailedJoins() {
        return search != null ? search.getFailedJoins() : 0;
    }

    /**The longest drive in to set off from, in blocks.  The flood only spreads this far, as that's most of its cost.**/
    private double getMaxLength() {
        return settings.summonDistance * MAX_LENGTH_FACTOR;
    }

    /**Floods out from where each goal's run-in starts, which is where the routes join them.**/
    private void startGoalField() {
        FlowField.Settings field = new FlowField.Settings();
        field.clearance = Math.max(0, checker.shape.side - 0.5);
        field.headroom = checker.shape.top - checker.shape.bottom;
        field.maxUp = settings.maxUp;
        field.maxDown = settings.maxDown;
        field.towardSources = true;
        field.radius = fieldRadius;
        //Room only matters to whether the vehicle fits, and to keeping it a block off things, so look no further.
        field.clearanceRange = Math.ceil(checker.shape.side + 1);
        //It only needs to reach as far as a drive in can be, or for a vehicle that's nearby, a little past where it is.
        field.maxDistance = start != null ? Double.POSITIVE_INFINITY : getMaxLength();
        goalField = new FlowField(checker.terrain, field, (int) Math.floor(player.x), (int) Math.floor(player.z));
        if (start != null) {
            //The vehicle's origin may be well into somewhere tight, like a garage, with its front out where there's room.
            goalField.setTarget((int) Math.floor(start.x), start.y, (int) Math.floor(start.z), (int) Math.ceil(checker.shape.bodyLength / 2) + 1, FIELD_MARGIN + checker.shape.bodyLength);
        }
        double runIn = getSearchSettings().runIn;
        for (ArrivalPlanner.Goal goal : goals) {
            double length = Math.min(runIn, goal.runIn);
            double[] runInStart = HybridSearch.arc(goal.x, goal.z, goal.yaw, 0, -length);
            int y = checker.terrain.findSurface((int) Math.floor(runInStart[0]), (int) Math.floor(runInStart[1]), goal.y, settings.maxDown, settings.maxUp, checker.shape.top - checker.shape.bottom);
            if (y != TerrainMap.NO_SURFACE) {
                goalField.addSource((int) Math.floor(runInStart[0]), y, (int) Math.floor(runInStart[1]), length);
            }
        }
        stage = Stage.GOAL_FIELD;
    }

    private SpawnSelector.Settings getSpawnSettings() {
        SpawnSelector.Settings spawn = new SpawnSelector.Settings();
        spawn.minDistance = settings.summonDistance / 2;
        spawn.minLength = settings.summonDistance * 0.6;
        spawn.targetLength = settings.summonDistance;
        spawn.maxLength = getMaxLength();
        spawn.maxUp = settings.maxUp;
        spawn.maxDown = settings.maxDown;
        return spawn;
    }

    /**
     * Where a placed vehicle may set off from: each place picked, facing the way the drive goes from there, costing
     * what's wrong with it as a place to set off from.
     */
    private List<HybridSearch.Start> getStarts(List<SpawnSelector.Target> targets) {
        List<HybridSearch.Start> starts = new ArrayList<>();
        for (SpawnSelector.Target target : targets) {
            double offFront = Math.abs(VehicleSummonFlight.clampAngle(Math.toDegrees(Math.atan2(target.x - player.x, target.z - player.z)) - lookYaw));
            starts.add(new HybridSearch.Start(target.x, target.y, target.z, target.yaw, HybridSearch.getStartCost(target.length, settings.summonDistance, target.visible, offFront)));
        }
        return starts;
    }

    private HybridSearch.Settings getSearchSettings() {
        HybridSearch.Settings search = new HybridSearch.Settings();
        search.turnRadius = settings.turnRadius;
        search.cruiseSpeed = settings.cruiseSpeed;
        search.maxUp = settings.maxUp;
        search.maxDown = settings.maxDown;
        //Drive at least a vehicle's length straight in, and never less than a few blocks, or small vehicles can't settle straight.
        search.runIn = Math.max(MIN_RUN_IN, checker.shape.bodyLength);
        search.targetLength = settings.summonDistance;
        search.maxExpansions = settings.maxExpansions;
        search.allowReverse = settings.allowReverse && start != null;
        search.reverseFactor = settings.reverseFactor;
        search.cuspCost = settings.cuspCost;
        search.allowPivot = settings.allowPivot;
        search.pivotRate = settings.pivotRate;
        search.pivotPlayerClearance = ArrivalPlanner.getPivotPlayerClearance(settings.arrival);
        //Each way it goes should be a fair part of its length, or the changes of direction are just shuffling.
        search.minGearLength = Math.max(search.minGearLength, checker.shape.bodyLength / 2);
        return search;
    }

    private void startSearch(List<HybridSearch.Start> starts) {
        search = new HybridSearch(checker, kinematics, getSearchSettings(), player, goalField, goals, starts, start == null);
        stage = Stage.SEARCH;
    }
}
