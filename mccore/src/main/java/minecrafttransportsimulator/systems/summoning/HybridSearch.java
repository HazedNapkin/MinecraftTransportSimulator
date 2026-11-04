package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Finds the route a ground vehicle drives in along, working backwards from where it pulls up.
 * <p>
 * This is a hybrid A* search: its states are exact poses, any position and any yaw, but two poses in the same block
 * column, at the same height and within {@link #YAW_BIN} degrees of yaw count as the same, so it doesn't search
 * near-copies.  From each pose it steps back along an arc of {@link Settings#stepLength} blocks, straight or curving
 * either way at half or full lock for its turn radius, so every route is one the vehicle can actually steer.  It
 * checks the vehicle's whole shape fits at every half block along each arc, so long vehicles and overhangs swinging
 * wide are accounted for, and finds the ground under each point, climbing and dropping as driving it would.
 * <p>
 * It starts from each goal with its straight run-in already driven, so every route ends by driving straight in to
 * pull up.  Working backwards means it ends facing exactly the right way however it gets there.
 * <p>
 * Routes cost their time to drive more than their length: a tight turn costs more as the vehicle has to slow for
 * it, and so do changes of steering, S-bends, steps and passing close to things.  That's what makes them smooth.
 * <p>
 * If it may, it also steps back along arcs the vehicle backs along.  MTS turns a vehicle going in reverse the other
 * way for the same steering, which is just the same arc driven backwards.  Backing up costs more than driving
 * forward, and each change of direction costs a lot, so it only backs up where it must: out of where it's nosed in,
 * or in a three-point turn where it hasn't room to turn round.  Each way it goes must be a little way, so it doesn't
 * shuffle, and it changes direction only so many times.  The straight run-in is always driven forward.  And as the
 * goals have a straight run-in to end with, where the vehicle is has a straight run-out it may set off backing
 * along: the route may set off from anywhere on it, so the search needn't find its way exactly down an alley.
 * <p>
 * Where it stops depends on where the vehicle is coming from.  If it'll be placed to drive in, any pose that's far
 * enough along, far enough from the player and has room to place it will do, the cheaper still if the player can't
 * see it there.  A {@link FlowField} from good places to set off from guides the search there.  If the vehicle is
 * already nearby, the route has to start where it is, and a flow field from there guides the search to it.
 */
final class HybridSearch {
    /**Yaws within this many degrees count as the same state.**/
    static final double YAW_BIN = 5;
    /**How far apart the vehicle is checked along each arc, in blocks.**/
    static final double SAMPLE_SPACING = 0.5;
    /**
     * How much the cost left counts against the cost so far: the estimate of it from a state, and the extra cost of
     * setting off from one.  Over 1 finds a route faster, if not quite the cheapest.  Both are weighted alike, or
     * setting off would look cheaper than carrying on to somewhere better, and it'd set off from the first place it could.
     */
    private static final double HEURISTIC_WEIGHT = 1.2;
    /**
     * The same, for searches that may back up.  The estimate is how far it is to drive, whichever way the vehicle
     * faces, so it says nothing of the turning round that backing up is for, and without leaning on it far harder
     * the search looks everywhere near the goals before it tries turning round where the vehicle is.
     */
    private static final double REVERSE_HEURISTIC_WEIGHT = 2.5;
    /**Costs, in blocks of route, for: changing steering by full lock, steering one way then the other, and each step up or down.**/
    private static final double CURVATURE_CHANGE_COST = 1;
    private static final double SWITCH_COST = 2;
    private static final double STEP_COST = 1;
    /**Cost of each block driven with no room to spare either side, falling to nothing with a block of room.**/
    private static final double CLEARANCE_COST = 0.5;
    /**
     * Costs of setting off from a place, for each block the route is short of or over the summon distance.  Short
     * costs more than a block of driving, or the shortest route allowed would always be cheapest.
     */
    private static final double SHORT_LENGTH_COST = 1.5;
    private static final double LONG_LENGTH_COST = 0.2;
    /**How near the vehicle's current pose a route must start, in blocks and degrees, if it's already nearby.**/
    private static final double START_RADIUS = 1;
    private static final double START_YAW = 15;
    /**
     * How many more states to expand once there's somewhere to set off from, looking for somewhere cheaper, before
     * settling for the cheapest found.  Without this, where nowhere is hidden from the player, it'd search everywhere
     * near for a hidden place before settling for one in view.
     */
    private static final int FINISH_GRACE = 500;
    /**Estimate of the cost left from somewhere the guiding flow field didn't reach.**/
    private static final double UNREACHED_ESTIMATE = 30;
    /**
     * How many times what driving it forward would cost backing up costs, and the cost of each change of direction,
     * for stopping, shifting and setting off again.  Backing up is slower than this, but costing it all would have
     * the search look everywhere it could drive forward before it backed up even a little.
     */
    private static final double REVERSE_FACTOR = 2;
    private static final double CUSP_COST = 5;

    /**How the vehicle drives, and what makes a place to set off from.**/
    static final class Settings {
        /**The radius of the vehicle's tightest turns, with any margin for steering error, in blocks.**/
        double turnRadius = 4;
        double stepLength = 1.5;
        /**Speed it drives in at, in m/s.  Turns it has to slow below this for cost more.**/
        double cruiseSpeed = 8.3;
        int maxUp = 1;
        int maxDown = 2;
        /**Longest straight run-in driven before searching from each goal, in blocks.  Shorter if the goal's is.**/
        double runIn = 4;
        /**Shortest route, and nearest the player in a straight line, a placed vehicle may set off from, in blocks.**/
        double minLength = 18;
        double minDistance = 15;
        /**Best route length, in blocks.**/
        double targetLength = 30;
        int maxExpansions = 20000;
        /**
         * True if the route may back up, so it can reverse out of where it's stuck or make a three-point turn where it
         * hasn't room to turn round.  Only routes that set off from where the vehicle already is may, as a placed
         * vehicle can be placed facing whichever way it needs, and sets off moving forward.
         */
        boolean allowReverse;
        /**
         * Longest it may back straight out from where it is, in blocks.  Backing straight out is checked directly
         * rather than searched for, as the search would have to find its way exactly down the line to get there.
         */
        double maxRunOut = 24;
        /**Shortest it drives either way between changes of direction, in blocks, so it doesn't shuffle back and forth.**/
        double minGearLength = 2;
        /**Most times it may change direction.**/
        int maxCusps = 3;
    }

    private final PoseChecker checker;
    private final VehicleKinematics kinematics;
    private final Settings settings;
    private final Point3D player;
    private final FlowField guide;
    private final double headroom;
    private final double maxCurvature;
    private final double[] curvatures;
    private final double heuristicWeight;
    private final PriorityQueue<Entry> open = new PriorityQueue<>((a, b) -> Double.compare(a.f, b.f));
    private final Map<Long, Node> nodes = new HashMap<>();
    private final Map<Long, Boolean> visibleCache = new HashMap<>();
    private final int originX;
    private final int originZ;
    /**The pose the vehicle is at if it's already nearby and has to set off from there, or null if it'll be placed.**/
    private final SummonRoute.Pose start;
    private int expansions;
    private boolean finished;
    private Node found;
    /**The cheapest place to set off from found so far, its cost, and how many states had been expanded when the first was found.**/
    private Node bestFinish;
    private double bestFinishCost = Double.MAX_VALUE;
    private int firstFinishExpansion = -1;

    /**
     * Makes a search for a route to any of the goals, guided by the flow field, which may be null.  For a vehicle
     * that's already nearby, start is where it is, which the route must set off from; otherwise it's null.
     */
    HybridSearch(PoseChecker checker, VehicleKinematics kinematics, Settings settings, Point3D player, FlowField guide, List<ArrivalPlanner.Goal> goals, SummonRoute.Pose start) {
        this.checker = checker;
        this.kinematics = kinematics;
        this.settings = settings;
        this.player = player.copy();
        this.guide = guide;
        this.start = start;
        this.headroom = checker.shape.top - checker.shape.bottom;
        this.maxCurvature = 1 / settings.turnRadius;
        this.heuristicWeight = settings.allowReverse && start != null ? REVERSE_HEURISTIC_WEIGHT : HEURISTIC_WEIGHT;
        this.curvatures = Double.isInfinite(settings.turnRadius) ? new double[] { 0 } : new double[] { 0, maxCurvature / 2, -maxCurvature / 2, maxCurvature, -maxCurvature };
        this.originX = (int) Math.floor(player.x);
        this.originZ = (int) Math.floor(player.z);
        for (ArrivalPlanner.Goal goal : goals) {
            seed(goal);
        }
        if (open.isEmpty()) {
            finished = true;
        }
    }

    /**
     * Adds the goal, with its straight run-in driven back from it, as somewhere to search from.  Nothing is added if
     * the run-in doesn't fit, though the arrival planner should have made sure it does.
     */
    private void seed(ArrivalPlanner.Goal goal) {
        Node end = new Node(goal.x, goal.y, goal.z, goal.yaw, 0, false, null, goal, goal.cost, 0, null);
        double runIn = Math.min(settings.runIn, goal.runIn);
        Node runInStart = runIn > 0 ? stepBack(end, 0, false, runIn) : end;
        if (runInStart != null) {
            add(runInStart);
        }
    }

    /**Expands states until it's done, budget states have been expanded, or maxNanos pass.  Returns true once it's done.**/
    boolean step(int budget, long maxNanos) {
        long startTime = System.nanoTime();
        while (!finished && budget-- > 0 && System.nanoTime() - startTime < maxNanos) {
            Entry entry = open.poll();
            if (entry == null || ++expansions > settings.maxExpansions || (firstFinishExpansion >= 0 && expansions - firstFinishExpansion > FINISH_GRACE)) {
                //Out of states or time.  Settle for the cheapest place to set off from found, if any.
                found = bestFinish;
                finished = true;
                break;
            }
            Node current = entry.node;
            if (entry.finish) {
                //The cheapest way to finish, now nothing left could beat it.
                found = current;
                finished = true;
                break;
            }
            if (current.closed || nodes.get(current.key) != current) {
                continue;
            }
            current.closed = true;
            double finishCost = getFinishCost(current);
            if (!Double.isNaN(finishCost)) {
                double f = current.g + heuristicWeight * finishCost;
                open.add(new Entry(current, f, true));
                if (f < bestFinishCost) {
                    bestFinish = current;
                    bestFinishCost = f;
                }
                if (firstFinishExpansion < 0) {
                    firstFinishExpansion = expansions;
                }
            }
            for (double curvature : curvatures) {
                Node earlier = stepBack(current, curvature, false, settings.stepLength);
                if (earlier != null) {
                    add(earlier);
                }
            }
            if (settings.allowReverse) {
                for (double curvature : curvatures) {
                    Node earlier = stepBack(current, curvature, true, settings.stepLength);
                    if (earlier != null) {
                        add(earlier);
                    }
                }
            }
        }
        return finished;
    }

    void finish() {
        while (!step(Integer.MAX_VALUE, Long.MAX_VALUE)) {
        }
    }

    boolean isFinished() {
        return finished;
    }

    /**How many states have been expanded, for tuning.**/
    int getExpansions() {
        return expansions;
    }

    /**
     * The extra cost of the route setting off from the state, or NaN if it can't.  Placed vehicles can set off from
     * anywhere far enough out with room to place them, and vehicles already nearby only from where they are.
     */
    private double getFinishCost(Node node) {
        if (start != null) {
            boolean atStart = Math.hypot(node.x - start.x, node.z - start.z) <= START_RADIUS && Math.abs(VehicleSummonFlight.clampAngle(node.yaw - start.yaw)) <= START_YAW && Math.abs(node.y - start.y) <= 1;
            if (atStart && (node.cusps == 0 || node.segmentLength >= settings.minGearLength - 1e-6)) {
                return 0;
            }
            return settings.allowReverse ? getRunOutCost(node) : Double.NaN;
        }
        if (node.reverse || node.length < settings.minLength || Math.hypot(node.x - player.x, node.z - player.z) < settings.minDistance || !checker.canPark(node.x, node.y, node.z, node.yaw)) {
            return Double.NaN;
        }
        boolean visible = visibleCache.computeIfAbsent(node.key, k -> SpawnSelector.isVisible(checker, player, node.x, node.y, node.z));
        double off = node.length - settings.targetLength;
        return (off < 0 ? -off * SHORT_LENGTH_COST : off * LONG_LENGTH_COST) + (visible ? SpawnSelector.VISIBLE_COST : 0);
    }

    /**
     * The extra cost of the route setting off by backing straight out from where the vehicle is to the state, or NaN
     * if it can't.  The state must be about on the line it backs along, facing about the same way.  If it carries on
     * forward from there, that's a change of direction.  The poses backed through are kept on the state for the route.
     */
    private double getRunOutCost(Node node) {
        if (Math.abs(VehicleSummonFlight.clampAngle(node.yaw - start.yaw)) > START_YAW) {
            return Double.NaN;
        }
        double headingX = Math.sin(Math.toRadians(start.yaw));
        double headingZ = Math.cos(Math.toRadians(start.yaw));
        double dx = node.x - start.x;
        double dz = node.z - start.z;
        double along = -(dx * headingX + dz * headingZ);
        double side = Math.abs(dx * headingZ - dz * headingX);
        if (along <= START_RADIUS || along > settings.maxRunOut || side > START_RADIUS + along * Math.sin(Math.toRadians(START_YAW))) {
            return Double.NaN;
        }
        boolean cusp = !node.reverse;
        if (cusp && (node.cusps >= settings.maxCusps || node.segmentLength < settings.minGearLength - 1e-6 || along < settings.minGearLength)) {
            return Double.NaN;
        }
        //Walk out from the state along the straight line it backed in along, to where it set off.
        int samples = Math.max(1, (int) Math.ceil(along / SAMPLE_SPACING));
        double[][] poses = new double[samples][];
        int laterY = node.y;
        for (int i = 1; i <= samples; ++i) {
            double[] pose = arc(node.x, node.z, node.yaw, 0, along * i / samples);
            int y = checker.terrain.findSurface((int) Math.floor(pose[0]), (int) Math.floor(pose[1]), laterY, settings.maxDown, settings.maxUp, headroom);
            if (y == TerrainMap.NO_SURFACE || !checker.canDrive(pose[0], y, pose[1], pose[2])) {
                return Double.NaN;
            }
            laterY = y;
            poses[i - 1] = new double[] { pose[0], pose[1], pose[2], y };
        }
        double[] first = poses[samples - 1];
        if (Math.hypot(first[0] - start.x, first[1] - start.z) > START_RADIUS || Math.abs(first[3] - start.y) > 1) {
            return Double.NaN;
        }
        node.runOut = poses;
        return along * REVERSE_FACTOR + (cusp ? CUSP_COST : 0);
    }

    /**
     * Returns the state the vehicle drives from, along an arc of the curvature and length, to get to the later state,
     * or null if it doesn't fit all the way or may not go that way.  It drives forward along the arc, or backs along
     * it if reverse is set.  Walking back, each point's ground is found from the one after it, the way driving it
     * climbs and drops.
     */
    private Node stepBack(Node later, double curvature, boolean reverse, double length) {
        boolean cusp = later.later != null && reverse != later.reverse;
        if (cusp && (later.cusps >= settings.maxCusps || later.segmentLength < settings.minGearLength - 1e-6)) {
            return null;
        }
        int samples = Math.max(1, (int) Math.ceil(length / SAMPLE_SPACING));
        double[][] poses = new double[samples][];
        int laterY = later.y;
        int steps = 0;
        //Backing along the arc to the later state means it starts that far ahead along it.
        double direction = reverse ? 1 : -1;
        for (int i = 1; i <= samples; ++i) {
            double[] pose = arc(later.x, later.z, later.yaw, curvature, direction * length * i / samples);
            int y = checker.terrain.findSurface((int) Math.floor(pose[0]), (int) Math.floor(pose[1]), laterY, settings.maxDown, settings.maxUp, headroom);
            if (y == TerrainMap.NO_SURFACE || !checker.canDrive(pose[0], y, pose[1], pose[2])) {
                return null;
            }
            if (y != laterY) {
                ++steps;
            }
            laterY = y;
            poses[i - 1] = new double[] { pose[0], pose[1], pose[2], y };
        }
        double[] earliest = poses[samples - 1];
        double cost = getMoveCost(later, curvature, reverse, cusp, length, steps, earliest);
        Node earlier = new Node(earliest[0], (int) earliest[3], earliest[1], earliest[2], curvature, reverse, later, later.goal, later.g + cost, later.length + length, poses);
        earlier.cusps = later.cusps + (cusp ? 1 : 0);
        earlier.segmentLength = (cusp ? 0 : later.segmentLength) + length;
        return earlier;
    }

    /**
     * The cost of driving the move: its length, more for turns too tight to take at cruise speed and for backing up
     * slowly, and extra for changing steering from the move after it, for S-bends, for changing direction, for steps,
     * and for little room either side.
     */
    private double getMoveCost(Node later, double curvature, boolean reverse, boolean cusp, double length, int steps, double[] earliest) {
        double slowdown;
        if (kinematics != null) {
            double speed = Math.min(settings.cruiseSpeed, kinematics.getMaxSpeed(curvature));
            slowdown = settings.cruiseSpeed / Math.max(0.5, speed);
        } else {
            slowdown = 1 + Math.abs(curvature) / maxCurvature;
        }
        double cost = length * slowdown * (reverse ? REVERSE_FACTOR : 1) + steps * STEP_COST;
        if (maxCurvature > 0 && !Double.isInfinite(maxCurvature)) {
            cost += CURVATURE_CHANGE_COST * Math.abs(curvature - later.curvature) / maxCurvature;
        }
        if (cusp) {
            //Turning the wheel the other way while stopped to change direction is no S-bend.
            cost += CUSP_COST;
        } else if (curvature * later.curvature < 0) {
            cost += SWITCH_COST;
        }
        if (guide != null) {
            double room = guide.getClearance((int) Math.floor(earliest[0]), (int) earliest[3], (int) Math.floor(earliest[1])) - (checker.shape.side - 0.5);
            cost += CLEARANCE_COST * Math.max(0, 1 - room) * length;
        }
        return cost;
    }

    private void add(Node node) {
        Node existing = nodes.get(node.key);
        if (existing == null || (!existing.closed && node.g < existing.g)) {
            nodes.put(node.key, node);
            open.add(new Entry(node, node.g + heuristicWeight * estimate(node), false));
        }
    }

    /**The estimated cost from the state to somewhere the route can set off from.**/
    private double estimate(Node node) {
        double distance = guide != null ? guide.getDistance((int) Math.floor(node.x), node.y, (int) Math.floor(node.z), 1) : Double.POSITIVE_INFINITY;
        if (!Double.isInfinite(distance)) {
            return distance;
        }
        if (start != null) {
            return Math.hypot(node.x - start.x, node.z - start.z);
        }
        return Math.max(0, settings.minLength - node.length) + (guide != null ? UNREACHED_ESTIMATE : 0);
    }

    boolean succeeded() {
        return found != null;
    }

    /**The route found, or null if there isn't one.**/
    SummonRoute getRoute() {
        if (found == null) {
            return null;
        }
        //Each state knows the poses back along the move from the state after it, ending with itself.  Walk them forwards.
        List<SummonRoute.Pose> poses = new ArrayList<>();
        if (found.runOut != null) {
            //It backs straight out from where it is first.
            for (int i = found.runOut.length - 1; i >= 0; --i) {
                double[] pose = found.runOut[i];
                poses.add(new SummonRoute.Pose(pose[0], (int) pose[3], pose[1], pose[2], 0, true));
            }
        }
        poses.add(new SummonRoute.Pose(found.x, found.y, found.z, found.yaw, found.curvature, found.runOut != null || found.reverse));
        for (Node node = found; node.later != null; node = node.later) {
            for (int i = node.poses.length - 2; i >= 0; --i) {
                double[] pose = node.poses[i];
                poses.add(new SummonRoute.Pose(pose[0], (int) pose[3], pose[1], pose[2], node.curvature, node.reverse));
            }
            Node later = node.later;
            poses.add(new SummonRoute.Pose(later.x, later.y, later.z, later.yaw, later.later != null ? later.curvature : 0, node.reverse));
        }
        return new SummonRoute(Collections.unmodifiableList(poses), start == null, found.goal);
    }

    /**
     * Where a vehicle at x,z facing the yaw ends up driving distance blocks along an arc of the curvature, as
     * {x, z, yaw}.  Negative distances go backwards.  Yaw rises as it turns left, and it heads along (sin, cos) of it.
     */
    static double[] arc(double x, double z, double yaw, double curvature, double distance) {
        double startYaw = Math.toRadians(yaw);
        if (Math.abs(curvature) < 1e-9) {
            return new double[] { x + Math.sin(startYaw) * distance, z + Math.cos(startYaw) * distance, yaw };
        }
        double endYaw = startYaw + curvature * distance;
        return new double[] { x + (Math.cos(startYaw) - Math.cos(endYaw)) / curvature, z + (Math.sin(endYaw) - Math.sin(startYaw)) / curvature, VehicleSummonFlight.clampAngle(Math.toDegrees(endYaw)) };
    }

    /**
     * The state the pose falls in.  Which way it's going counts too, as a state backing up and one driving forward
     * differ in what it costs to carry on from them.
     */
    private long getKey(double x, int y, double z, double yaw, boolean reverse) {
        long cellX = ((long) Math.floor(x) - originX) & 0xFFFFF;
        long cellZ = ((long) Math.floor(z) - originZ) & 0xFFFFF;
        long yawBin = Math.floorMod((long) Math.floor(yaw / YAW_BIN + 0.5), (long) Math.round(360 / YAW_BIN));
        return (reverse ? 1L << 60 : 0) | (cellX << 40) | (cellZ << 20) | ((y & 0xFFFL) << 8) | yawBin;
    }

    /**
     * A pose on a route.  Each knows the pose after it on the route, the move between them, and the poses along
     * that move, back from the later one to this one.
     */
    private final class Node {
        final long key;
        final double x;
        final int y;
        final double z;
        final double yaw;
        /**Curvature of the move from here to the later state.**/
        final double curvature;
        /**True if the move from here to the later state backs up.**/
        final boolean reverse;
        final Node later;
        final ArrivalPlanner.Goal goal;
        final double g;
        /**How far it is from here to the goal along the route, in blocks.**/
        final double length;
        /**The poses along the move, back from the later state, as {x, z, yaw, y}.  The last is this one.**/
        final double[][] poses;
        /**How many times the route changes direction from here on, and how far it is from here to the first, or to the goal if it doesn't.**/
        int cusps;
        double segmentLength;
        /**If the route sets off by backing straight out to here, the poses it backs through, out from here to where it sets off, as {x, z, yaw, y}.**/
        double[][] runOut;
        boolean closed;

        Node(double x, int y, double z, double yaw, double curvature, boolean reverse, Node later, ArrivalPlanner.Goal goal, double g, double length, double[][] poses) {
            this.key = getKey(x, y, z, yaw, reverse);
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.curvature = curvature;
            this.reverse = reverse;
            this.later = later;
            this.goal = goal;
            this.g = g;
            this.length = length;
            this.poses = poses;
        }
    }

    /**A state queued to expand, or to finish at.**/
    private static final class Entry {
        final Node node;
        final double f;
        final boolean finish;

        Entry(Node node, double f, boolean finish) {
            this.node = node;
            this.f = f;
            this.finish = finish;
        }
    }
}
