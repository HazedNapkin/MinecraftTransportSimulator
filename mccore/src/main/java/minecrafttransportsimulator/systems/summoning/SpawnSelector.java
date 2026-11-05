package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Picks good places for a vehicle to be placed to drive in from, for the route search to head for.
 * <p>
 * It looks over every cell a {@link FlowField} towards the goals reached, as that says how far the drive in from
 * each is.  The best are where the drive is about the summon distance, somewhere in front of the player, and hidden
 * from them, so the vehicle comes round a corner or over a rise into view rather than popping into being.  A cell
 * also has to have room to place the vehicle facing the way the drive sets off.  The picks are kept apart, so the
 * search has a spread to head for.
 * <p>
 * The search sets off from all the picks at once.  Each goal has its own picks first, the best place whose way in
 * leads to it, as a goal may only be reachable from some sides: a long vehicle that can't turn a tight corner onto its
 * run-in has to come at it straight.  The rest of the picks are the best of what's left.
 */
final class SpawnSelector {
    /**Costs, in blocks of route: for each block the drive is off the summon distance, for being seen, and for being behind.**/
    private static final double LENGTH_COST = 0.2;
    static final double VISIBLE_COST = 15;
    private static final double BEHIND_COST = 6;
    /**How far apart picks must be, in blocks.**/
    private static final double MIN_SPACING = 4;
    /**How high above the player's feet their eyes are.**/
    static final double EYE_HEIGHT = 1.62;

    /**What makes a good place to set off from.**/
    static final class Settings {
        /**Nearest to the player, in a straight line, the vehicle may be placed, in blocks.**/
        double minDistance = 15;
        /**Shortest, best and longest drive in, in blocks.**/
        double minLength = 18;
        double targetLength = 30;
        double maxLength = 48;
        /**Only every this many'th column each way is looked at, as neighbours are much alike.**/
        int stride = 2;
        int maxTargets = 16;
        /**Most the vehicle can climb and drop between neighbouring blocks.**/
        int maxUp = 1;
        int maxDown = 2;
    }

    private final PoseChecker checker;
    private final Settings settings;
    private final Point3D player;
    private final double lookYaw;
    private final List<FlowField.Cell> cells;
    private int cellIndex;
    /**
     * Places scored so far, cheapest first.  Each is first queued at its cost if it can't be seen, as finding whether
     * it can be seen takes raycasts, and whether there's room to place it world lookups.  Those are only found once it
     * comes to the front, when it's queued again at its full cost, so they're only found for places that could be picked.
     */
    private final PriorityQueue<Candidate> queue = new PriorityQueue<>((a, b) -> Double.compare(a.cost, b.cost));
    private final List<Target> picked = new ArrayList<>();
    /**The goals, by flood source, that have a pick, and the places put off as their goal had one, until each has.**/
    private final Set<Integer> sourcesPicked = new HashSet<>();
    private final List<Candidate> deferred = new ArrayList<>();
    private boolean eachGoalPicked;
    private boolean picksDone;
    /**The goals to look straight back from, and the next to.**/
    private final List<ArrivalPlanner.Goal> goals;
    private int goalIndex;
    private List<Target> targets;

    SpawnSelector(PoseChecker checker, FlowField goalField, List<ArrivalPlanner.Goal> goals, Settings settings, Point3D player, double lookYaw) {
        this.checker = checker;
        this.settings = settings;
        this.player = player.copy();
        this.lookYaw = lookYaw;
        this.cells = goalField.getCells();
        this.goals = goals;
    }

    /**Looks over cells, picks, then looks straight back from each goal, until it's done or maxNanos pass.  Returns true once it's done.**/
    boolean step(long maxNanos) {
        long startTime = System.nanoTime();
        while (cellIndex < cells.size() && System.nanoTime() - startTime < maxNanos) {
            score(cells.get(cellIndex++));
        }
        while (cellIndex >= cells.size() && !picksDone && System.nanoTime() - startTime < maxNanos) {
            pickNext();
        }
        while (picksDone && goalIndex < goals.size() && System.nanoTime() - startTime < maxNanos) {
            addStraightApproach(goals.get(goalIndex++));
        }
        if (picksDone && goalIndex >= goals.size()) {
            targets = picked;
        }
        return targets != null;
    }

    void finish() {
        while (!step(Long.MAX_VALUE)) {
        }
    }

    /**The places picked, best first.  Empty if there's nowhere, and null until it's done.**/
    List<Target> getTargets() {
        return targets;
    }

    /**Queues the cell if it could be a place to set off from, at its cost if it can't be seen.**/
    private void score(FlowField.Cell cell) {
        if (Math.floorMod(cell.x, settings.stride) != 0 || Math.floorMod(cell.z, settings.stride) != 0 || cell.distance < settings.minLength || cell.distance > settings.maxLength) {
            return;
        }
        double x = cell.x + 0.5;
        double z = cell.z + 0.5;
        double distance = Math.hypot(x - player.x, z - player.z);
        double yaw = cell.getYawToNext();
        if (distance < settings.minDistance || Double.isNaN(yaw)) {
            return;
        }
        double offFront = Math.abs(VehicleSummonFlight.clampAngle(Math.toDegrees(Math.atan2(x - player.x, z - player.z)) - lookYaw));
        double cost = Math.abs(cell.distance - settings.targetLength) * LENGTH_COST + offFront / 180 * BEHIND_COST;
        queue.add(new Candidate(x, cell.y, z, yaw, cell.distance, cost, cell.source));
    }

    /**
     * Takes the next best, skipping any too near a better one already taken, and until each goal has one, any whose
     * goal has.  The cheapest queued is checked first if it hasn't been, and queued again at its full cost, as it may
     * no longer be the cheapest.
     */
    private void pickNext() {
        Candidate candidate = queue.poll();
        if (candidate == null && !eachGoalPicked) {
            //Every goal that could have a pick has.  Fill up with the best of the rest.
            eachGoalPicked = true;
            queue.addAll(deferred);
            deferred.clear();
            return;
        }
        if (candidate == null || picked.size() >= settings.maxTargets) {
            picksDone = true;
            return;
        }
        for (Target taken : picked) {
            if (Math.hypot(candidate.x - taken.x, candidate.z - taken.z) < MIN_SPACING) {
                return;
            }
        }
        if (!eachGoalPicked && sourcesPicked.contains(candidate.source)) {
            deferred.add(candidate);
            return;
        }
        if (!candidate.checked) {
            candidate.checked = true;
            if (checker.canPark(candidate.x, candidate.y, candidate.z, candidate.yaw)) {
                candidate.visible = isVisible(checker, player, candidate.x, candidate.y, candidate.z);
                if (candidate.visible) {
                    candidate.cost += VISIBLE_COST;
                }
                queue.add(candidate);
            }
            return;
        }
        picked.add(new Target(candidate.x, candidate.y, candidate.z, candidate.yaw, candidate.length, candidate.cost, candidate.visible));
        sourcesPicked.add(candidate.source);
    }

    /**
     * Adds the best place to set off from straight back along the way the goal pulls up, if there's room to drive
     * straight in from far enough and it's not too near a pick already.  The flood says nothing of which way the
     * vehicle faces, so its picks may need turns the vehicle can't make to line up with the goal.  Every goal with a
     * clear straight in has this one it can always get to.  Walking back, each point's ground is found from the one
     * after it, the way driving in climbs and drops.
     */
    private void addStraightApproach(ArrivalPlanner.Goal goal) {
        double forwardX = Math.sin(Math.toRadians(goal.yaw));
        double forwardZ = Math.cos(Math.toRadians(goal.yaw));
        double headroom = checker.shape.top - checker.shape.bottom;
        List<Candidate> points = new ArrayList<>();
        int laterY = goal.y;
        for (double back = 0.5; back <= settings.maxLength + 1e-9; back += 0.5) {
            double x = goal.x - forwardX * back;
            double z = goal.z - forwardZ * back;
            int y = checker.terrain.findSurface((int) Math.floor(x), (int) Math.floor(z), laterY, settings.maxDown, settings.maxUp, headroom);
            if (y == TerrainMap.NO_SURFACE || !checker.canDrive(x, y, z, goal.yaw)) {
                break;
            }
            laterY = y;
            if (back >= settings.minLength - 1e-9 && Math.floorMod((int) Math.round(back * 2), 4) == 0 && Math.hypot(x - player.x, z - player.z) >= settings.minDistance) {
                double offFront = Math.abs(VehicleSummonFlight.clampAngle(Math.toDegrees(Math.atan2(x - player.x, z - player.z)) - lookYaw));
                points.add(new Candidate(x, y, z, goal.yaw, back, HybridSearch.getStartCost(back, settings.targetLength, false, offFront), -1));
            }
        }
        //The best of them, checking whether each can be seen only once it could be best.
        PriorityQueue<Candidate> best = new PriorityQueue<>((a, b) -> Double.compare(a.cost, b.cost));
        best.addAll(points);
        Candidate point;
        while ((point = best.poll()) != null) {
            if (point.checked) {
                for (Target taken : picked) {
                    if (Math.hypot(point.x - taken.x, point.z - taken.z) < MIN_SPACING) {
                        return;
                    }
                }
                picked.add(new Target(point.x, point.y, point.z, point.yaw, point.length, point.cost, point.visible));
                return;
            }
            point.checked = true;
            if (checker.canPark(point.x, point.y, point.z, point.yaw)) {
                point.visible = isVisible(checker, player, point.x, point.y, point.z);
                if (point.visible) {
                    point.cost += VISIBLE_COST;
                }
                best.add(point);
            }
        }
    }

    /**
     * Returns true if the player standing at their feet can see a vehicle with its origin over x,z on the ground
     * at y: there's a clear line from their eyes to the middle or the top of it.
     */
    static boolean isVisible(PoseChecker checker, Point3D player, double x, int y, double z) {
        VehicleShape shape = checker.shape;
        Point3D eye = player.copy().add(0, EYE_HEIGHT, 0);
        double height = shape.top - shape.bottom;
        return checker.terrain.isLineClear(eye, new Point3D(x, y + height / 2, z)) || checker.terrain.isLineClear(eye, new Point3D(x, y + height - 0.1, z));
    }

    /**A place that may be picked, and its cost so far.**/
    private static final class Candidate {
        final double x;
        final int y;
        final double z;
        final double yaw;
        final double length;
        /**The goal the way in from here leads to, as its flood source.**/
        final int source;
        double cost;
        boolean checked;
        boolean visible;

        Candidate(double x, int y, double z, double yaw, double length, double cost, int source) {
            this.source = source;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.length = length;
            this.cost = cost;
        }
    }

    /**A place picked to set off from, facing the way the drive goes.**/
    static final class Target {
        final double x;
        final int y;
        final double z;
        final double yaw;
        final double cost;
        final boolean visible;
        /**How far the drive in from here is, at least, in blocks.**/
        final double length;

        Target(double x, int y, double z, double yaw, double length, double cost, boolean visible) {
            this.length = length;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.cost = cost;
            this.visible = visible;
        }
    }
}
