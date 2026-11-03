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
    /**Costs, in blocks of route, for: changing steering by full lock, steering one way then the other, and each step up or down.**/
    private static final double CURVATURE_CHANGE_COST = 1;
    private static final double SWITCH_COST = 2;
    private static final double STEP_COST = 1;
    /**Cost of each block driven with no room to spare either side, falling to nothing with a block of room.**/
    private static final double CLEARANCE_COST = 0.5;
    /**Costs of setting off from a place, for each block the route is off the summon distance.**/
    private static final double LENGTH_COST = 0.2;
    /**How near the vehicle's current pose a route must start, in blocks and degrees, if it's already nearby.**/
    private static final double START_RADIUS = 1;
    private static final double START_YAW = 15;
    /**Estimate of the cost left from somewhere the guiding flow field didn't reach.**/
    private static final double UNREACHED_ESTIMATE = 30;

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
    }

    private final PoseChecker checker;
    private final VehicleKinematics kinematics;
    private final Settings settings;
    private final Point3D player;
    private final FlowField guide;
    private final double headroom;
    private final double maxCurvature;
    private final double[] curvatures;
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
        Node end = new Node(goal.x, goal.y, goal.z, goal.yaw, 0, null, goal, goal.cost, 0, null);
        double runIn = Math.min(settings.runIn, goal.runIn);
        Node runInStart = runIn > 0 ? stepBack(end, 0, runIn) : end;
        if (runInStart != null) {
            add(runInStart);
        }
    }

    /**Expands states until it's done, budget states have been expanded, or maxNanos pass.  Returns true once it's done.**/
    boolean step(int budget, long maxNanos) {
        long startTime = System.nanoTime();
        while (!finished && budget-- > 0 && System.nanoTime() - startTime < maxNanos) {
            Entry entry = open.poll();
            if (entry == null || ++expansions > settings.maxExpansions) {
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
                open.add(new Entry(current, current.g + HEURISTIC_WEIGHT * finishCost, true));
            }
            for (double curvature : curvatures) {
                Node earlier = stepBack(current, curvature, settings.stepLength);
                if (earlier != null) {
                    add(earlier);
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
            return atStart ? 0 : Double.NaN;
        }
        if (node.length < settings.minLength || Math.hypot(node.x - player.x, node.z - player.z) < settings.minDistance || !checker.canPark(node.x, node.y, node.z, node.yaw)) {
            return Double.NaN;
        }
        boolean visible = visibleCache.computeIfAbsent(node.key, k -> SpawnSelector.isVisible(checker, player, node.x, node.y, node.z));
        return Math.abs(node.length - settings.targetLength) * LENGTH_COST + (visible ? SpawnSelector.VISIBLE_COST : 0);
    }

    /**
     * Returns the state the vehicle drives forward from, along an arc of the curvature and length, to get to the
     * later state, or null if it doesn't fit all the way.  Walking back, each point's ground is found from the one
     * after it, the way driving it climbs and drops.
     */
    private Node stepBack(Node later, double curvature, double length) {
        int samples = Math.max(1, (int) Math.ceil(length / SAMPLE_SPACING));
        double[][] poses = new double[samples][];
        int laterY = later.y;
        int steps = 0;
        for (int i = 1; i <= samples; ++i) {
            double[] pose = arc(later.x, later.z, later.yaw, curvature, -length * i / samples);
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
        double cost = getMoveCost(later, curvature, length, steps, earliest);
        return new Node(earliest[0], (int) earliest[3], earliest[1], earliest[2], curvature, later, later.goal, later.g + cost, later.length + length, poses);
    }

    /**
     * The cost of driving the move: its length, more for turns too tight to take at cruise speed, and extra for
     * changing steering from the move after it, for S-bends, for steps, and for little room either side.
     */
    private double getMoveCost(Node later, double curvature, double length, int steps, double[] earliest) {
        double slowdown;
        if (kinematics != null) {
            double speed = Math.min(settings.cruiseSpeed, kinematics.getMaxSpeed(curvature));
            slowdown = settings.cruiseSpeed / Math.max(0.5, speed);
        } else {
            slowdown = 1 + Math.abs(curvature) / maxCurvature;
        }
        double cost = length * slowdown + steps * STEP_COST;
        if (maxCurvature > 0 && !Double.isInfinite(maxCurvature)) {
            cost += CURVATURE_CHANGE_COST * Math.abs(curvature - later.curvature) / maxCurvature;
        }
        if (curvature * later.curvature < 0) {
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
            open.add(new Entry(node, node.g + HEURISTIC_WEIGHT * estimate(node), false));
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
        poses.add(new SummonRoute.Pose(found.x, found.y, found.z, found.yaw, found.curvature));
        for (Node node = found; node.later != null; node = node.later) {
            for (int i = node.poses.length - 2; i >= 0; --i) {
                double[] pose = node.poses[i];
                poses.add(new SummonRoute.Pose(pose[0], (int) pose[3], pose[1], pose[2], node.curvature));
            }
            Node later = node.later;
            poses.add(new SummonRoute.Pose(later.x, later.y, later.z, later.yaw, later.later != null ? later.curvature : 0));
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

    private long getKey(double x, int y, double z, double yaw) {
        long cellX = ((long) Math.floor(x) - originX) & 0xFFFFF;
        long cellZ = ((long) Math.floor(z) - originZ) & 0xFFFFF;
        long yawBin = Math.floorMod((long) Math.floor(yaw / YAW_BIN + 0.5), (long) Math.round(360 / YAW_BIN));
        return (cellX << 40) | (cellZ << 20) | ((y & 0xFFFL) << 8) | yawBin;
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
        final Node later;
        final ArrivalPlanner.Goal goal;
        final double g;
        /**How far it is from here to the goal along the route, in blocks.**/
        final double length;
        /**The poses along the move, back from the later state, as {x, z, yaw, y}.  The last is this one.**/
        final double[][] poses;
        boolean closed;

        Node(double x, int y, double z, double yaw, double curvature, Node later, ArrivalPlanner.Goal goal, double g, double length, double[][] poses) {
            this.key = getKey(x, y, z, yaw);
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.curvature = curvature;
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
