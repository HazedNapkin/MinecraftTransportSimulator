package minecrafttransportsimulator.systems.navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;

/**
 * Finds the route a ground vehicle drives in along, from where it sets off to where it pulls up.
 * <p>
 * This is a hybrid A* search: its states are exact poses, any position and any yaw, but two poses in the same block
 * column, at the same height and within {@link #YAW_BIN} degrees of yaw count as the same, so it doesn't search
 * near-copies.  From each pose it drives on along an arc of {@link Settings#stepLength} blocks, straight or curving
 * either way at half or full lock for its turn radius, so every route is one the vehicle can actually steer.  It
 * checks the vehicle's whole shape fits at every half block along each arc, so long vehicles and overhangs swinging
 * wide are accounted for, and finds the ground under each point, climbing and dropping as driving it does.
 * <p>
 * It searches forward, from where the vehicle sets off.  A vehicle that's already somewhere sets off from where it
 * is.  A vehicle that will be placed may set off from any of several starts, each facing the way the drive goes from
 * there, which start the search all at once, each already costing whatever the caller thinks is wrong with it as a
 * place to set off from.  A {@link FlowField} out from the goals says how far each state is from them, which is what
 * guides the search.  The search is laid out round a centre point, usually where the goals are, which is also kept
 * clear when turning on the spot, as a summon keeps clear of the player who called.
 * <p>
 * Every route ends with a goal's straight run-in, driven forward.  Once the search is near the start of one, it tries
 * every few states to join it exactly, with the shortest ways to drive there turning at full lock
 * ({@link AnalyticPaths}): forward only, or backing up too if it may.  The join must fit all along, like any other move.
 * <p>
 * Routes cost their time to drive more than their length: a tight turn costs more as the vehicle has to slow for
 * it, and so do changes of steering, S-bends, steps and passing close to things.  That's what makes them smooth.
 * <p>
 * If it may, it also backs up.  MTS turns a vehicle going in reverse the other way for the same steering, which is
 * just the same arc driven backwards.  Backing up costs more than driving forward, and each change of direction costs
 * a lot, so it only backs up where that saves a lot of driving: out of where it's nosed in, or in a three-point turn
 * where it hasn't room to turn round.  Each way it goes must be a little way, so it doesn't shuffle, and it changes
 * direction only so many times.  Placed vehicles never back up, as they can be placed facing the way they need.
 * <p>
 * The motion model is the same driven either way: an arc backed along is the arc driven forward the other way round.
 * So the search could run from the goals back to where it sets off too, which is cheaper where the goal end is the
 * cramped one.  {@link Settings#direction} is where that would be chosen, but only forward is built, as the arrival
 * planner seldom picks goals cramped enough to need it.
 *
 * @author HazedNapkin
 */
public final class HybridSearch {
    /**Yaws within this many degrees count as the same state.**/
    public static final double YAW_BIN = 5;
    /**How far apart the vehicle is checked along each arc, in blocks.**/
    public static final double SAMPLE_SPACING = 0.5;
    /**How much the estimate of the cost left counts against the cost so far.  Over 1 finds a route faster, if not quite the cheapest.**/
    public static final double HEURISTIC_WEIGHT = 1.5;
    /**Costs, in blocks of route, for: changing steering by full lock, steering one way then the other, and each step up or down.**/
    private static final double CURVATURE_CHANGE_COST = 1;
    private static final double SWITCH_COST = 2;
    private static final double STEP_COST = 1;
    /**Cost of each block driven with no room to spare either side, falling to nothing with a block of room.**/
    private static final double CLEARANCE_COST = 0.5;
    /**
     * Most states to expand once there's a route, looking for a cheaper one, before settling for the cheapest found.
     * States that can't beat it even as estimated are skipped without counting.  If the route costs far more than the
     * best place to set off from was estimated to, the estimate was badly wrong, as it is for long vehicles that can't
     * go the way the flood does, and a much cheaper route from another place may be close to being found, so a placed
     * vehicle looks for longer.  One already nearby has only the one place, and turning round is what costs more there.
     */
    private static final int FINISH_GRACE = 200;
    private static final int LONG_FINISH_GRACE = 1500;
    private static final double LONG_ROUTE_FACTOR = 2;
    /**Estimate of the cost left from somewhere the goal flow field didn't reach, on top of the distance to the centre.**/
    private static final double UNREACHED_ESTIMATE = 30;
    /**
     * Cost of each block driven only just clear of things, without the margin to spare, in blocks of route.  A
     * vehicle that strays a little off the route there scrapes them, so a route keeps clear if that's not far round.
     */
    public static final double TIGHT_COST = 2;
    /**How a pose fits: see {@link #fit}.**/
    private static final int CLEAR = 0;
    private static final int TIGHT = 1;
    private static final int NO_FIT = -1;
    /**How near the start of a run-in a state must be, in blocks, for the search to try joining it exactly.**/
    public static final double JOIN_RANGE = 15;
    /**How near the start of a run-in a state must be, in blocks, for the estimate to count which way it faces.**/
    private static final double HEADING_RANGE = 20;
    /**Least turn on the spot worth lining up with a run-in by, in degrees, rather than driving onto it lined up.**/
    private static final double MIN_PIVOT_TURN = 20;
    /**Turns on the spot the search tries, in degrees.**/
    private static final double[] PIVOT_TURNS = { 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180 };
    /**How many states in range apart joins are tried, as each costs a lot of checks.**/
    private static final int JOIN_INTERVAL = 3;
    /**Most joins checked for fit each try, cheapest first.**/
    private static final int JOIN_CHECKS = 3;

    /**Which end of the route the search starts from.**/
    public enum Direction {
        /**From where the vehicle sets off, ending with an exact join onto a goal's run-in.**/
        FORWARD
    }

    /**How the vehicle drives.**/
    public static final class Settings {
        /**The radius of the vehicle's tightest turns, with any margin for steering error, in blocks.**/
        public double turnRadius = 4;
        public double stepLength = 1.5;
        /**Speed it drives in at, in m/s.  Turns it has to slow below this for cost more.**/
        public double cruiseSpeed = 8.3;
        public int maxUp = 1;
        public int maxDown = 2;
        /**Longest straight run-in each route ends with, in blocks.  Shorter if the goal's is.**/
        public double runIn = 4;
        public int maxExpansions = 20000;
        /**
         * True if the route may back up, so it can reverse out of where it's stuck or make a three-point turn where it
         * hasn't room to turn round.  Only routes that set off from where the vehicle already is may, as a placed
         * vehicle can be placed facing whichever way it needs.
         */
        public boolean allowReverse;
        /**
         * How many times slower than cruising it backs up, so what each block backed costs, and the cost of each change
         * of direction, in blocks.  Backing up is slow however it steers, so a turn backing up costs no more than this.
         */
        public double reverseFactor = 3;
        public double cuspCost = 5;
        /**Shortest it drives either way between changes of direction, in blocks, so it doesn't shuffle back and forth.**/
        public double minGearLength = 2;
        /**Most times it may change direction, counting turns on the spot.**/
        public int maxCusps = 3;
        public Direction direction = Direction.FORWARD;
        /**
         * True if the vehicle can turn on the spot, as vehicles that skid steer do stopped and in neutral, how fast it
         * turns, in degrees a second, and how far the centre point must be outside the circle it sweeps doing so, in blocks.
         */
        public boolean allowPivot;
        public double pivotRate = 45;
        public double pivotClearance = 1.5;
        /**
         * How much further off things than its shape needs the vehicle would like to keep as it drives, in blocks, so it
         * still clears them when it strays a little off the route.  Driving any closer costs {@link #TIGHT_COST} a block.
         * Where it pulls up, its run-in and where it turns on the spot needn't, as it's slow there: see
         * {@link PoseChecker#canDrive(double, int, double, double, double)}.
         */
        public double margin;
    }

    /**Somewhere the route may set off from, facing the yaw, and what's wrong with setting off from there, in blocks of route.**/
    public static final class Start {
        public final double x;
        public final int y;
        public final double z;
        public final double yaw;
        public final double cost;

        public Start(double x, int y, double z, double yaw, double cost) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.cost = cost;
        }
    }

    private final PoseChecker checker;
    private final VehicleKinematics kinematics;
    private final Settings settings;
    private final Point3D center;
    private final FlowField guide;
    private final boolean placed;
    private final double headroom;
    private final double maxCurvature;
    private final double[] curvatures;
    private final List<RunIn> runIns = new ArrayList<>();
    private final PriorityQueue<Entry> open = new PriorityQueue<>((a, b) -> Double.compare(a.f, b.f));
    private final Map<Long, Node> nodes = new HashMap<>();
    private final int originX;
    private final int originZ;
    private int expansions;
    private boolean finished;
    private Node found;
    /**The cheapest route found so far, and how many states had been expanded when the first was found.**/
    private Node bestFinish;
    private int firstFinishExpansion = -1;
    /**What the cheapest place to set off from was estimated to cost to drive in from, to judge the routes found by.**/
    private double expectedCost = Double.MAX_VALUE;
    /**For tuning: states expanded near the goals, joins tried and joins that didn't fit.**/
    private int goalSideExpansions;
    private int joinTries;
    private int joinTicker;
    private int failedJoins;

    /**
     * Makes a search for a route from any of the starts to any of the goals, guided by the flow field out from the
     * goals and laid out round the centre.  Placed is true if the vehicle will be placed at whichever start the route sets off from, rather than
     * already being at the only one.
     */
    public HybridSearch(PoseChecker checker, VehicleKinematics kinematics, Settings settings, Point3D center, FlowField guide, List<RouteGoal> goals, List<Start> starts, boolean placed) {
        this.checker = checker;
        this.kinematics = kinematics;
        this.settings = settings;
        this.center = center.copy();
        this.guide = guide;
        this.placed = placed;
        this.headroom = checker.shape.top - checker.shape.bottom;
        this.maxCurvature = 1 / settings.turnRadius;
        this.curvatures = Double.isInfinite(settings.turnRadius) ? new double[] { 0 } : new double[] { 0, maxCurvature / 2, -maxCurvature / 2, maxCurvature, -maxCurvature };
        this.originX = (int) Math.floor(center.x);
        this.originZ = (int) Math.floor(center.z);
        for (RouteGoal goal : goals) {
            RunIn runIn = measureRunIn(goal);
            if (runIn != null) {
                runIns.add(runIn);
            }
        }
        if (!runIns.isEmpty()) {
            for (Start start : starts) {
                Node root = new Node(start.x, start.y, start.z, start.yaw, 0, false, null, start.cost, 0, null);
                root.root = true;
                add(root);
                expectedCost = Math.min(expectedCost, root.g + root.h);
                //Try driving straight onto a run-in from where it sets off, however far, as from some starts that's the only way.
                tryJoin(root, true);
            }
        }
        if (open.isEmpty()) {
            finished = true;
        }
    }

    /**
     * The goal's straight run-in, walked back from it, or null if it doesn't fit, though the arrival planner should
     * have made sure it does.  Walking back, each point's ground is found from the one after it, the way driving it
     * climbs and drops.
     */
    private RunIn measureRunIn(RouteGoal goal) {
        double length = Math.min(settings.runIn, goal.runIn);
        int samples = Math.max(1, (int) Math.ceil(length / SAMPLE_SPACING));
        double[][] poses = new double[samples + 1][];
        poses[samples] = new double[] { goal.x, goal.z, goal.yaw, goal.y, 0, 0 };
        int laterY = goal.y;
        for (int i = samples - 1; i >= 0; --i) {
            double[] pose = arc(goal.x, goal.z, goal.yaw, 0, -length * (samples - i) / samples);
            int y = length > 0 ? checker.terrain.findSurface((int) Math.floor(pose[0]), (int) Math.floor(pose[1]), laterY, settings.maxDown, settings.maxUp, headroom) : goal.y;
            if (y == TerrainMap.NO_SURFACE || !checker.canDrive(pose[0], y, pose[1], pose[2])) {
                return null;
            }
            laterY = y;
            poses[i] = new double[] { pose[0], pose[1], pose[2], y, 0, 0 };
        }
        return new RunIn(goal, length, poses);
    }

    /**Expands states until it's done, budget states have been expanded, or maxNanos pass.  Returns true once it's done.**/
    public boolean step(int budget, long maxNanos) {
        long startTime = System.nanoTime();
        while (!finished && budget-- > 0 && System.nanoTime() - startTime < maxNanos) {
            Entry entry = open.poll();
            if (entry == null) {
                //Out of states.  Settle for the cheapest route found, if any.
                found = bestFinish;
                finished = true;
                break;
            }
            Node current = entry.node;
            if (entry.finish) {
                //The cheapest route, now nothing left could beat it.
                found = current;
                finished = true;
                break;
            }
            if (current.closed || nodes.get(current.key) != current || (bestFinish != null && current.g + current.h >= bestFinish.g)) {
                //Already expanded, reached more cheaply since, or no way to beat the route already found from here.
                continue;
            }
            if (current.unchecked) {
                //A turn on the spot, only checked now it's come up, as most never do.
                Node earlier = current.earlier;
                if (!canPivot(earlier.x, earlier.y, earlier.z, earlier.yaw, current.yaw)) {
                    current.closed = true;
                    continue;
                }
                current.unchecked = false;
            }
            if (++expansions > settings.maxExpansions || (firstFinishExpansion >= 0 && expansions - firstFinishExpansion > (placed && bestFinish.g > LONG_ROUTE_FACTOR * expectedCost ? LONG_FINISH_GRACE : FINISH_GRACE))) {
                //Out of time.  Settle for the cheapest route found, if any.
                found = bestFinish;
                finished = true;
                break;
            }
            current.closed = true;
            if (isNearRunIn(current)) {
                ++goalSideExpansions;
                if (current.root || joinTicker++ % JOIN_INTERVAL == 0) {
                    tryJoin(current);
                }
            }
            for (double curvature : curvatures) {
                add(extend(current, curvature, false, settings.stepLength));
            }
            if (settings.allowReverse) {
                for (double curvature : curvatures) {
                    add(extend(current, curvature, true, settings.stepLength));
                }
            }
            //Turning on the spot again straight after would only undo some of it, and a placed vehicle can be placed facing any way.
            if (settings.allowPivot && !current.pivot && !(placed && current.root)) {
                for (double turn : PIVOT_TURNS) {
                    add(pivot(current, turn));
                }
            }
        }
        return finished;
    }

    /**
     * The state the vehicle gets to turning on the spot by the turn, in degrees, or null if it may not, as it's changed
     * direction as often as it may.  Whether it has room isn't checked until it comes up: see {@link #canPivot}.  It
     * costs the time it takes, as blocks it could have driven at cruise speed, and stopping to do it costs like a
     * change of direction, so it's only worth it where it saves a lot of driving, never in place of a gentle curve.
     */
    private Node pivot(Node earlier, double turn) {
        if (earlier.cusps >= settings.maxCusps) {
            return null;
        }
        double yaw = RotationMatrix.clampAngle(earlier.yaw + turn);
        double cost = getPivotCost(turn);
        Node later = new Node(earlier.x, earlier.y, earlier.z, yaw, 0, false, earlier, earlier.g + cost, earlier.length, getPivotPoses(earlier.x, earlier.y, earlier.z, earlier.yaw, turn), true);
        later.cusps = earlier.cusps + 1;
        later.unchecked = true;
        return later;
    }

    /**What turning on the spot by the turn, in degrees, costs: stopping, as for a change of direction, and the time it takes.**/
    private double getPivotCost(double turn) {
        return settings.cuspCost + Math.abs(turn) / settings.pivotRate * settings.cruiseSpeed;
    }

    /**
     * Returns true if the vehicle can turn on the spot at the pose by the turn from fromYaw to toYaw, the short way
     * round.  It turns about its origin, as MTS turns vehicles, so its shape must fit at every step through the turn,
     * close enough that no corner moves more than {@link #SAMPLE_SPACING} between them, and the centre point must be
     * well outside the circle its furthest point sweeps.
     */
    public boolean canPivot(double x, int y, double z, double fromYaw, double toYaw) {
        if (Math.hypot(center.x - x, center.z - z) < checker.shape.radius() + settings.pivotClearance) {
            return false;
        }
        double turn = RotationMatrix.clampAngle(toYaw - fromYaw);
        int steps = getPivotSteps(turn);
        for (int i = 1; i <= steps; ++i) {
            if (!checker.canDrive(x, y, z, fromYaw + turn * i / steps)) {
                return false;
            }
        }
        return true;
    }

    /**How many steps a turn on the spot by the turn, in degrees, is checked and driven in.**/
    private int getPivotSteps(double turn) {
        double step = Math.toDegrees(SAMPLE_SPACING / Math.max(0.5, checker.shape.radius()));
        return Math.max(1, (int) Math.ceil(Math.abs(turn) / step - 1e-9));
    }

    /**The poses through a turn on the spot by the turn from the yaw, as {x, z, yaw, y, curvature, reverse, pivot}.**/
    private double[][] getPivotPoses(double x, int y, double z, double fromYaw, double turn) {
        int steps = getPivotSteps(turn);
        double[][] poses = new double[steps][];
        for (int i = 1; i <= steps; ++i) {
            poses[i - 1] = new double[] { x, z, RotationMatrix.clampAngle(fromYaw + turn * i / steps), y, 0, 0, 1 };
        }
        return poses;
    }

    public void finish() {
        while (!step(Integer.MAX_VALUE, Long.MAX_VALUE)) {
        }
    }

    public boolean isFinished() {
        return finished;
    }

    /**How many states have been expanded, for tuning.**/
    public int getExpansions() {
        return expansions;
    }

    /**How many states were expanded within {@link #JOIN_RANGE} of a run-in, for tuning.**/
    public int getGoalSideExpansions() {
        return goalSideExpansions;
    }

    /**How many joins onto a run-in were tried, and how many joins checked were cut short by something in the way, for tuning.**/
    public int getJoinTries() {
        return joinTries;
    }

    public int getFailedJoins() {
        return failedJoins;
    }

    /**How many states had been expanded when the first route was found, or -1 if none has been, for tuning.**/
    public int getFirstRouteExpansion() {
        return firstFinishExpansion;
    }

    private boolean isNearRunIn(Node node) {
        for (RunIn runIn : runIns) {
            double[] start = runIn.poses[0];
            if (Math.hypot(node.x - start[0], node.z - start[1]) <= JOIN_RANGE && Math.abs(node.y - start[3]) <= 3) {
                return true;
            }
        }
        return false;
    }

    /**
     * Tries to join the state exactly onto the start of a run-in nearby, with the shortest ways to drive there.  The
     * cheapest few are checked, and the first that fits is queued as a route, at what it costs in all.
     */
    private void tryJoin(Node node) {
        tryJoin(node, false);
    }

    /**Like {@link #tryJoin(Node)}, but from any distance from the run-ins if anyRange is set.**/
    private void tryJoin(Node node, boolean anyRange) {
        ++joinTries;
        List<Join> joins = new ArrayList<>();
        double bestCost = bestFinish != null ? bestFinish.g : Double.MAX_VALUE;
        for (RunIn runIn : runIns) {
            double[] target = runIn.poses[0];
            if (!anyRange && Math.hypot(node.x - target[0], node.z - target[1]) > JOIN_RANGE) {
                continue;
            }
            addJoins(joins, node, runIn, target[2], Double.NaN, bestCost);
            if (settings.allowPivot && Math.hypot(node.x - target[0], node.z - target[1]) >= 1) {
                //Drive to where the run-in starts facing about the way there, then turn on the spot to line up with it.
                double arrivalYaw = Math.toDegrees(Math.atan2(target[0] - node.x, target[1] - node.z));
                if (Math.abs(RotationMatrix.clampAngle(arrivalYaw - target[2])) > MIN_PIVOT_TURN) {
                    addJoins(joins, node, runIn, arrivalYaw, arrivalYaw, bestCost);
                }
            }
        }
        joins.sort((a, b) -> Double.compare(a.g, b.g));
        for (int i = 0; i < Math.min(JOIN_CHECKS, joins.size()); ++i) {
            Node finish = drive(node, joins.get(i));
            if (finish != null) {
                open.add(new Entry(finish, finish.g, true));
                if (bestFinish == null || finish.g < bestFinish.g) {
                    bestFinish = finish;
                }
                if (firstFinishExpansion < 0) {
                    firstFinishExpansion = expansions;
                }
                return;
            }
            ++failedJoins;
        }
    }

    /**
     * Adds the ways to join the run-in from the state that could beat the cost, arriving at its start facing the yaw.
     * If pivotYaw isn't NaN, that's the yaw, and it turns on the spot from there to line up with the run-in.
     */
    private void addJoins(List<Join> joins, Node node, RunIn runIn, double arrivalYaw, double pivotYaw, double bestCost) {
        double[] target = runIn.poses[0];
        List<AnalyticPaths.Path> paths = settings.allowReverse ? AnalyticPaths.reedsShepp(node.x, node.z, node.yaw, target[0], target[1], arrivalYaw, settings.turnRadius) : AnalyticPaths.dubins(node.x, node.z, node.yaw, target[0], target[1], arrivalYaw, settings.turnRadius);
        for (AnalyticPaths.Path path : paths) {
            double cost = getJoinCost(node, path, runIn, pivotYaw);
            if (!Double.isNaN(cost) && node.g + cost < bestCost) {
                joins.add(new Join(path, runIn, node.g + cost, pivotYaw));
            }
        }
    }

    /**
     * What driving the path from the state, then turning on the spot from pivotYaw if it's not NaN, then the run-in,
     * costs, without what's in the way, or NaN if it breaks the rules on changing direction.  The run-in is always
     * driven forward.
     */
    private double getJoinCost(Node node, AnalyticPaths.Path path, RunIn runIn, double pivotYaw) {
        double cost = 0;
        double curvature = node.isStopped() ? 0 : node.curvature;
        boolean hasGear = !node.isStopped();
        boolean reverse = node.reverse;
        int cusps = node.cusps;
        double segmentLength = node.isStopped() ? 0 : node.segmentLength;
        for (int i = 0; i <= path.types.length; ++i) {
            if (i == path.types.length && !Double.isNaN(pivotYaw)) {
                //Turning on the spot comes before the run-in, and it sets off again from stopped.
                if (++cusps > settings.maxCusps) {
                    return Double.NaN;
                }
                cost += getPivotCost(RotationMatrix.clampAngle(runIn.poses[0][2] - pivotYaw));
                hasGear = false;
                curvature = 0;
            }
            //The run-in comes last.
            double length = i < path.types.length ? path.lengths[i] : runIn.length;
            double pieceCurvature = i < path.types.length ? path.getCurvature(i) : 0;
            if (Math.abs(length) < 1e-9 && i < path.types.length) {
                continue;
            }
            boolean pieceReverse = length < 0;
            boolean cusp = hasGear && pieceReverse != reverse;
            if (cusp) {
                if (++cusps > settings.maxCusps || segmentLength < settings.minGearLength - 1e-6) {
                    return Double.NaN;
                }
                cost += settings.cuspCost;
                segmentLength = 0;
            } else if (pieceCurvature * curvature < 0) {
                cost += SWITCH_COST;
            }
            cost += getDriveCost(pieceCurvature, Math.abs(length), pieceReverse);
            if (maxCurvature > 0 && !Double.isInfinite(maxCurvature)) {
                cost += CURVATURE_CHANGE_COST * Math.abs(pieceCurvature - curvature) / maxCurvature;
            }
            segmentLength += Math.abs(length);
            curvature = pieceCurvature;
            reverse = pieceReverse;
            hasGear = true;
        }
        return cost + runIn.goal.cost;
    }

    /**
     * Drives the join from the state, then the run-in, checking it fits all along, and returns the state at the end of
     * the route, or null if something's in the way.  Each way it goes is checked at even spacing, with a pose exactly
     * where it changes direction.
     */
    private Node drive(Node node, Join join) {
        AnalyticPaths.Path path = join.path;
        List<double[]> poses = new ArrayList<>();
        //How far it drives only just clearing things.
        double tight = 0;
        int laterY = node.y;
        double x = node.x;
        double z = node.z;
        double yaw = node.yaw;
        int piece = 0;
        double[] target = join.runIn.poses[0];
        while (piece < path.types.length) {
            //Gather the pieces driven the same way.
            boolean reverse = path.lengths[piece] < 0;
            int last = piece;
            double runLength = 0;
            while (last < path.types.length && (Math.abs(path.lengths[last]) < 1e-9 || (path.lengths[last] < 0) == reverse)) {
                runLength += Math.abs(path.lengths[last]);
                ++last;
            }
            if (runLength < 1e-9) {
                piece = last;
                continue;
            }
            int samples = Math.max(1, (int) Math.ceil(runLength / SAMPLE_SPACING - 1e-9));
            for (int i = 1; i <= samples; ++i) {
                //Where it is this far along the run, and the curvature of the piece it's on.
                double along = runLength * i / samples;
                double[] pose = { x, z, yaw };
                double curvature = 0;
                for (int j = piece; j < last && along > 1e-9; ++j) {
                    double length = Math.min(along, Math.abs(path.lengths[j]));
                    if (length > 0) {
                        curvature = path.getCurvature(j);
                        pose = arc(pose[0], pose[1], pose[2], curvature, reverse ? -length : length);
                        along -= length;
                    }
                }
                if (i == samples && last == path.types.length) {
                    //Land exactly on the run-in, or where it starts, facing the way it turns on the spot from.
                    pose = new double[] { target[0], target[1], Double.isNaN(join.pivotYaw) ? target[2] : join.pivotYaw };
                }
                int y = checker.terrain.findSurface((int) Math.floor(pose[0]), (int) Math.floor(pose[1]), laterY, settings.maxUp, settings.maxDown, headroom);
                int fit = y == TerrainMap.NO_SURFACE ? NO_FIT : fit(pose[0], y, pose[1], pose[2]);
                if (fit == NO_FIT) {
                    return null;
                }
                tight += fit == TIGHT ? runLength / samples : 0;
                laterY = y;
                poses.add(new double[] { pose[0], pose[1], pose[2], y, curvature, reverse ? 1 : 0 });
            }
            double[] end = poses.get(poses.size() - 1);
            x = end[0];
            z = end[1];
            yaw = end[2];
            piece = last;
        }
        if (laterY != (int) target[3]) {
            return null;
        }
        if (!Double.isNaN(join.pivotYaw)) {
            double turn = RotationMatrix.clampAngle(target[2] - join.pivotYaw);
            if (poses.isEmpty() || !canPivot(target[0], laterY, target[1], join.pivotYaw, target[2])) {
                return null;
            }
            for (double[] pose : getPivotPoses(target[0], laterY, target[1], join.pivotYaw, turn)) {
                poses.add(pose);
            }
        }
        double[][] runInPoses = join.runIn.poses;
        //It's already at the start of the run-in if it didn't need to drive there.
        for (int i = poses.isEmpty() ? 0 : 1; i < runInPoses.length; ++i) {
            poses.add(runInPoses[i]);
        }
        double[] end = poses.get(poses.size() - 1);
        Node finish = new Node(end[0], (int) end[3], end[1], end[2], 0, false, node, join.g + TIGHT_COST * tight, node.length + join.path.length + join.runIn.length, poses.toArray(new double[0][]));
        finish.goal = join.runIn.goal;
        return finish;
    }

    /**
     * Returns the state the vehicle gets to driving on from the earlier state along an arc of the curvature and length,
     * or null if it doesn't fit all the way or may not go that way.  It drives forward along the arc, or backs along it
     * if reverse is set.  Each point's ground is found from the one before it, the way driving it climbs and drops.
     */
    private Node extend(Node earlier, double curvature, boolean reverse, double length) {
        boolean cusp = !earlier.isStopped() && reverse != earlier.reverse;
        if (cusp && (earlier.cusps >= settings.maxCusps || earlier.segmentLength < settings.minGearLength - 1e-6)) {
            return null;
        }
        int samples = Math.max(1, (int) Math.ceil(length / SAMPLE_SPACING));
        double[][] poses = new double[samples][];
        int earlierY = earlier.y;
        int steps = 0;
        int tightSamples = 0;
        double direction = reverse ? -1 : 1;
        for (int i = 1; i <= samples; ++i) {
            double[] pose = arc(earlier.x, earlier.z, earlier.yaw, curvature, direction * length * i / samples);
            int y = checker.terrain.findSurface((int) Math.floor(pose[0]), (int) Math.floor(pose[1]), earlierY, settings.maxUp, settings.maxDown, headroom);
            int fit = y == TerrainMap.NO_SURFACE ? NO_FIT : fit(pose[0], y, pose[1], pose[2]);
            if (fit == NO_FIT) {
                return null;
            }
            tightSamples += fit == TIGHT ? 1 : 0;
            if (y != earlierY) {
                ++steps;
            }
            earlierY = y;
            poses[i - 1] = new double[] { pose[0], pose[1], pose[2], y, curvature, reverse ? 1 : 0 };
        }
        double[] latest = poses[samples - 1];
        double cost = getMoveCost(earlier, curvature, reverse, cusp, length, steps, latest) + TIGHT_COST * length * tightSamples / samples;
        Node later = new Node(latest[0], (int) latest[3], latest[1], latest[2], curvature, reverse, earlier, earlier.g + cost, earlier.length + length, poses);
        later.cusps = earlier.cusps + (cusp ? 1 : 0);
        later.segmentLength = (cusp || earlier.isStopped() ? 0 : earlier.segmentLength) + length;
        return later;
    }

    /**
     * How the vehicle fits with its origin over x,z, its wheels at height y, facing the yaw: {@link #CLEAR} of things
     * by the margin, {@link #TIGHT} only just clear of them, or {@link #NO_FIT} not at all.
     */
    private int fit(double x, int y, double z, double yaw) {
        if (checker.canDrive(x, y, z, yaw, settings.margin)) {
            return CLEAR;
        }
        return settings.margin > 0 && checker.canDrive(x, y, z, yaw) ? TIGHT : NO_FIT;
    }

    /**The cost of driving a stretch of the curvature: its length, more for turns too tight to take at cruise speed and for backing up slowly.**/
    private double getDriveCost(double curvature, double length, boolean reverse) {
        double slowdown;
        if (kinematics != null) {
            double speed = Math.min(settings.cruiseSpeed, kinematics.getMaxSpeed(curvature));
            slowdown = settings.cruiseSpeed / Math.max(0.5, speed);
        } else {
            slowdown = 1 + Math.abs(curvature) / maxCurvature;
        }
        return length * (reverse ? Math.max(slowdown, settings.reverseFactor) : slowdown);
    }

    /**
     * The cost of driving the move, and extra for changing steering from the move before it, for S-bends, for changing
     * direction, for steps, and for little room either side.
     */
    private double getMoveCost(Node earlier, double curvature, boolean reverse, boolean cusp, double length, int steps, double[] latest) {
        double cost = getDriveCost(curvature, length, reverse) + steps * STEP_COST;
        double earlierCurvature = earlier.isStopped() ? 0 : earlier.curvature;
        if (maxCurvature > 0 && !Double.isInfinite(maxCurvature)) {
            cost += CURVATURE_CHANGE_COST * Math.abs(curvature - earlierCurvature) / maxCurvature;
        }
        if (cusp) {
            //Turning the wheel the other way while stopped to change direction is no S-bend.
            cost += settings.cuspCost;
        } else if (curvature * earlierCurvature < 0) {
            cost += SWITCH_COST;
        }
        if (guide != null) {
            double room = guide.getClearance((int) Math.floor(latest[0]), (int) latest[3], (int) Math.floor(latest[1])) - (checker.shape.side - 0.5);
            cost += CLEARANCE_COST * Math.max(0, 1 - room) * length;
        }
        return cost;
    }

    private void add(Node node) {
        if (node == null) {
            return;
        }
        Node existing = nodes.get(node.key);
        if (existing == null || (!existing.closed && node.g < existing.g)) {
            nodes.put(node.key, node);
            node.h = estimate(node);
            open.add(new Entry(node, node.g + HEURISTIC_WEIGHT * node.h, false));
        }
    }

    /**
     * The estimated cost from the state to the end of a route: the flow field's distance to the goals.  Near a goal
     * that says nothing of which way the state faces, though it can matter more than anything there, so it's at least
     * the shortest way to drive onto the run-in of a goal nearby turning at full lock, if that's further.  Goals
     * further off are at least {@link #HEADING_RANGE} away, so that's the most this can add.
     */
    private double estimate(Node node) {
        double distance = guide != null ? guide.getCost((int) Math.floor(node.x), node.y, (int) Math.floor(node.z), 1) : Double.POSITIVE_INFINITY;
        if (Double.isInfinite(distance)) {
            distance = Math.hypot(node.x - center.x, node.z - center.z) + UNREACHED_ESTIMATE;
        }
        if (Double.isInfinite(settings.turnRadius)) {
            return distance;
        }
        double nearest = HEADING_RANGE;
        for (RunIn runIn : runIns) {
            double[] target = runIn.poses[0];
            if (Math.hypot(node.x - target[0], node.z - target[1]) < nearest && Math.abs(node.y - target[3]) <= 3) {
                double length = settings.allowReverse ? AnalyticPaths.reedsSheppLength(node.x, node.z, node.yaw, target[0], target[1], target[2], settings.turnRadius) : AnalyticPaths.dubinsLength(node.x, node.z, node.yaw, target[0], target[1], target[2], settings.turnRadius);
                nearest = Math.min(nearest, length + runIn.length + runIn.goal.cost);
            }
        }
        return Math.max(distance, nearest);
    }

    public boolean succeeded() {
        return found != null;
    }

    /**The route found, or null if there isn't one.**/
    public Route getRoute() {
        if (found == null) {
            return null;
        }
        List<Node> chain = new ArrayList<>();
        for (Node node = found; node != null; node = node.earlier) {
            chain.add(node);
        }
        Collections.reverse(chain);
        //Each state knows the poses along the move to it from the state before.  The first pose says which way it sets off.
        Node root = chain.get(0);
        double[] firstMove = chain.size() > 1 ? chain.get(1).poses[0] : null;
        List<Route.Pose> poses = new ArrayList<>();
        poses.add(new Route.Pose(root.x, root.y, root.z, root.yaw, firstMove != null ? firstMove[4] : 0, firstMove != null && firstMove[5] != 0));
        for (int i = 1; i < chain.size(); ++i) {
            for (double[] pose : chain.get(i).poses) {
                poses.add(new Route.Pose(pose[0], (int) pose[3], pose[1], pose[2], pose[4], pose[5] != 0, pose.length > 6 && pose[6] != 0));
            }
        }
        return new Route(Collections.unmodifiableList(poses), placed, found.goal);
    }

    /**
     * Where a vehicle at x,z facing the yaw ends up driving distance blocks along an arc of the curvature, as
     * {x, z, yaw}.  Negative distances go backwards.  Yaw rises as it turns left, and it heads along (sin, cos) of it.
     */
    public static double[] arc(double x, double z, double yaw, double curvature, double distance) {
        double startYaw = Math.toRadians(yaw);
        if (Math.abs(curvature) < 1e-9) {
            return new double[] { x + Math.sin(startYaw) * distance, z + Math.cos(startYaw) * distance, yaw };
        }
        double endYaw = startYaw + curvature * distance;
        return new double[] { x + (Math.cos(startYaw) - Math.cos(endYaw)) / curvature, z + (Math.sin(endYaw) - Math.sin(startYaw)) / curvature, RotationMatrix.clampAngle(Math.toDegrees(endYaw)) };
    }

    /**
     * The state the pose falls in.  Which way it's going counts too, as a state backing up and one driving forward
     * differ in what it costs to carry on from them, and so does having just turned on the spot, which is checked
     * later than other moves and mustn't push out a state that's been checked.
     */
    private long getKey(double x, int y, double z, double yaw, boolean reverse, boolean pivot) {
        long cellX = ((long) Math.floor(x) - originX) & 0xFFFFF;
        long cellZ = ((long) Math.floor(z) - originZ) & 0xFFFFF;
        long yawBin = Math.floorMod((long) Math.floor(yaw / YAW_BIN + 0.5), (long) Math.round(360 / YAW_BIN));
        return (pivot ? 1L << 61 : 0) | (reverse ? 1L << 60 : 0) | (cellX << 40) | (cellZ << 20) | ((y & 0xFFFL) << 8) | yawBin;
    }

    /**A goal's straight run-in: the poses along it, from where it starts to the goal, as {x, z, yaw, y, curvature, reverse}.**/
    private static final class RunIn {
        public final RouteGoal goal;
        public final double length;
        public final double[][] poses;

        public RunIn(RouteGoal goal, double length, double[][] poses) {
            this.goal = goal;
            this.length = length;
            this.poses = poses;
        }
    }

    /**A way to join a run-in exactly from a state, and what the route would cost in all.**/
    private static final class Join {
        public final AnalyticPaths.Path path;
        public final RunIn runIn;
        public final double g;
        /**The yaw it gets to the start of the run-in at, to turn on the spot from to line up, or NaN if it gets there lined up.**/
        public final double pivotYaw;

        public Join(AnalyticPaths.Path path, RunIn runIn, double g, double pivotYaw) {
            this.path = path;
            this.runIn = runIn;
            this.g = g;
            this.pivotYaw = pivotYaw;
        }
    }

    /**
     * A pose on a route.  Each knows the pose before it on the route, the move between them, and the poses along
     * that move, from just after the earlier one to this one.
     */
    private final class Node {
        public final long key;
        public final double x;
        public final int y;
        public final double z;
        public final double yaw;
        /**Curvature of the move from the earlier state to here.**/
        public final double curvature;
        /**True if the move from the earlier state to here backs up.**/
        public final boolean reverse;
        public final Node earlier;
        public final double g;
        /**How far it is from where the route sets off to here, in blocks.**/
        public final double length;
        /**
         * The poses along the move from the earlier state, as {x, z, yaw, y, curvature, reverse}, and 1 after that for
         * a pose turned on the spot to.  The last is this one.
         */
        public final double[][] poses;
        /**How many times the route has changed direction up to here, and how far it's gone since the last, or since it set off.**/
        public int cusps;
        public double segmentLength;
        /**True for where the route sets off, which hasn't gone either way yet.**/
        public boolean root;
        /**True if the move here was a turn on the spot, and if it's not been checked there's room for it yet.**/
        public final boolean pivot;
        public boolean unchecked;
        /**The goal, for the state at the end of a route.**/
        public RouteGoal goal;
        /**The estimate of the cost from here to the end of a route.**/
        public double h;
        public boolean closed;

        /**True if the vehicle's stopped here, not yet going either way: where it sets off, or just after turning on the spot.**/
        public boolean isStopped() {
            return root || pivot;
        }

        public Node(double x, int y, double z, double yaw, double curvature, boolean reverse, Node earlier, double g, double length, double[][] poses) {
            this(x, y, z, yaw, curvature, reverse, earlier, g, length, poses, false);
        }

        public Node(double x, int y, double z, double yaw, double curvature, boolean reverse, Node earlier, double g, double length, double[][] poses, boolean pivot) {
            this.key = getKey(x, y, z, yaw, reverse, pivot);
            this.pivot = pivot;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.curvature = curvature;
            this.reverse = reverse;
            this.earlier = earlier;
            this.g = g;
            this.length = length;
            this.poses = poses;
        }
    }

    /**A state queued to expand, or a route to finish with.**/
    private static final class Entry {
        public final Node node;
        public final double f;
        public final boolean finish;

        public Entry(Node node, double f, boolean finish) {
            this.node = node;
            this.f = f;
            this.finish = finish;
        }
    }
}
