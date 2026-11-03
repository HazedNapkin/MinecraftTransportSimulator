package minecrafttransportsimulator.systems.summoning;

import java.util.Collections;
import java.util.List;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Plans a ground vehicle's drive in to the player, a slice at a time so it never stalls the server.
 * <p>
 * It runs in stages: find where the vehicle pulls up ({@link ArrivalPlanner}), flood out from there to see how far
 * the drive in is from everywhere near ({@link FlowField}), pick good hidden places about the summon distance out to
 * set off from ({@link SpawnSelector}), flood out from those to guide the search, then search backwards from where it
 * pulls up for the route ({@link HybridSearch}).  A vehicle that's already nearby skips picking places, and floods
 * out from where it is instead.
 */
final class RoutePlanner {
    /**How much further than the longest drive in the floods spread from the player, in blocks.**/
    private static final double FIELD_MARGIN = 8;
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
    }

    private enum Stage {
        ARRIVAL,
        GOAL_FIELD,
        SPAWNS,
        SPAWN_FIELD,
        SEARCH,
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
    private FlowField spawnField;
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
                        if (start != null) {
                            startSpawnField(null);
                        } else {
                            spawns = new SpawnSelector(checker, goalField, getSpawnSettings(), player, lookYaw);
                            stage = Stage.SPAWNS;
                        }
                    }
                    break;
                }
                case SPAWNS: {
                    if (spawns.step(left)) {
                        startSpawnField(spawns.getTargets());
                    }
                    break;
                }
                case SPAWN_FIELD: {
                    if (spawnField.step(left)) {
                        startSearch();
                    }
                    break;
                }
                case SEARCH: {
                    if (search.step(Integer.MAX_VALUE, left)) {
                        route = search.getRoute();
                        stage = Stage.DONE;
                    }
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
        timings.append(String.format("; cells %d + %d, expansions %d", goalField != null ? goalField.getCellCount() : 0, spawnField != null ? spawnField.getCellCount() : 0, getSearchExpansions()));
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

    /**The places picked to set off from, or null if none were picked.  For debugging.**/
    List<SpawnSelector.Target> getSpawnTargets() {
        return spawns != null ? spawns.getTargets() : null;
    }

    /**How many states the route search expanded, for tuning.**/
    int getSearchExpansions() {
        return search != null ? search.getExpansions() : 0;
    }

    /**The longest drive in to set off from, in blocks.  The floods only spread this far, as that's most of their cost.**/
    private double getMaxLength() {
        return settings.summonDistance * MAX_LENGTH_FACTOR;
    }

    private FlowField.Settings getFieldSettings(boolean towardSources) {
        FlowField.Settings field = new FlowField.Settings();
        field.clearance = Math.max(0, checker.shape.side - 0.5);
        field.headroom = checker.shape.top - checker.shape.bottom;
        field.maxUp = settings.maxUp;
        field.maxDown = settings.maxDown;
        field.towardSources = towardSources;
        field.radius = fieldRadius;
        //Room only matters to whether the vehicle fits, and to keeping it a block off things, so look no further.
        field.clearanceRange = Math.ceil(checker.shape.side + 1);
        //The flood to the goals only needs to reach as far as a drive in can be.
        field.maxDistance = getMaxLength();
        return field;
    }

    /**Floods out from where each goal's run-in starts, which is where the route search starts from too.**/
    private void startGoalField() {
        goalField = new FlowField(checker.terrain, getFieldSettings(true), (int) Math.floor(player.x), (int) Math.floor(player.z));
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
        return spawn;
    }

    /**Floods out from the places to set off from, or from where the vehicle is if it's already nearby, to guide the search.**/
    private void startSpawnField(List<SpawnSelector.Target> targets) {
        //It only has to reach back as far as the goals, which are as far as the furthest drive in from where it sets off.
        double reach = 0;
        if (start != null) {
            reach = goalField.getDistance((int) Math.floor(start.x), start.y, (int) Math.floor(start.z), 1);
        } else if (targets != null) {
            for (SpawnSelector.Target target : targets) {
                reach = Math.max(reach, target.length);
            }
        }
        FlowField.Settings fieldSettings = getFieldSettings(false);
        fieldSettings.maxDistance = (Double.isInfinite(reach) || reach == 0 ? getMaxLength() : reach) + FIELD_MARGIN;
        spawnField = new FlowField(checker.terrain, fieldSettings, (int) Math.floor(player.x), (int) Math.floor(player.z));
        spawnField.shareRoom(goalField);
        if (start != null) {
            spawnField.addSource((int) Math.floor(start.x), start.y, (int) Math.floor(start.z), 0);
        } else if (targets != null) {
            for (SpawnSelector.Target target : targets) {
                spawnField.addSource((int) Math.floor(target.x), target.y, (int) Math.floor(target.z), 0);
            }
        }
        stage = Stage.SPAWN_FIELD;
    }

    private HybridSearch.Settings getSearchSettings() {
        HybridSearch.Settings search = new HybridSearch.Settings();
        search.turnRadius = settings.turnRadius;
        search.cruiseSpeed = settings.cruiseSpeed;
        search.maxUp = settings.maxUp;
        search.maxDown = settings.maxDown;
        search.runIn = checker.shape.front + checker.shape.back;
        search.minLength = settings.summonDistance * 0.6;
        search.minDistance = settings.summonDistance / 2;
        search.targetLength = settings.summonDistance;
        search.maxExpansions = settings.maxExpansions;
        return search;
    }

    private void startSearch() {
        search = new HybridSearch(checker, kinematics, getSearchSettings(), player, spawnField, goals, start);
        stage = Stage.SEARCH;
    }
}
