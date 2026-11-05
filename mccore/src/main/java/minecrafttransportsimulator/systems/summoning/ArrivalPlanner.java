package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartSeat;
import minecrafttransportsimulator.systems.summoning.VehicleShape.ShapeBox;

/**
 * Finds where a vehicle pulls up by the player: its goal poses.  Each is exact, not snapped to blocks or headings.
 * <p>
 * The pack says which side of the vehicle faces the player (its arrival angle) and how big a gap to leave.  The
 * vehicle lines up so its driver's seat is opposite the player, if it has one, and stops with its nearest part the
 * gap away.  That's tried at bearings round the player, straight ahead of where they're looking first, then out to
 * the sides, then behind, and at heights near theirs.
 * <p>
 * A goal also needs a clear straight run-in behind it, which the vehicle drives along to pull up, so it settles
 * straight rather than stopping mid-turn.  If the run-in is shorter than it should be, the vehicle has to turn onto
 * it close by, so there must be room for that turn too: a quarter turn either way at the vehicle's turn radius,
 * sweeping its whole shape, without touching the player.  A vehicle that can turn on the spot needn't turn in if it
 * has room to turn on the spot where the run-in starts, as it can line up there.  If a goal hasn't the room, it's
 * moved further out from the player until it has, as a vehicle can't pull up lined up with the player if it can't
 * turn in.  It only moves out as far as the player can still walk to it, so it never ends up on the far side of a
 * wall from them.
 * <p>
 * Goals are ranked by how far round from straight ahead they are, how far above or below the player, how much
 * further out than the gap they had to go, and how short or bumpy their run-in is.
 */
final class ArrivalPlanner {
    /**How far apart the bearings tried round the player are, in degrees.**/
    static final double BEARING_STEP = 15;
    /**How far round from straight ahead bearings count as in front, to the side, and behind, in degrees.**/
    private static final double FRONT_LIMIT = 45;
    private static final double SIDE_LIMIT = 90;
    /**Costs of goals, in blocks of route: for each bearing step round, and extra for being to the side or behind.**/
    private static final double BEARING_COST = 1;
    private static final double SIDE_COST = 4;
    private static final double BEHIND_COST = 12;
    /**
     * Costs of goals for each block above or below the player, for each block further out than the gap, for each
     * block their run-in is short, and for each step in it.
     */
    private static final double HEIGHT_COST = 1;
    private static final double EXTRA_GAP_COST = 1;
    private static final double RUN_IN_SHORT_COST = 1;
    private static final double RUN_IN_STEP_COST = 0.5;
    /**How far apart the run-in is checked, and how finely the gap is found, in blocks.**/
    private static final double RUN_IN_SPACING = 0.5;
    private static final double GAP_SEARCH_STEP = 0.05;
    /**How far apart the turn in is checked, in blocks along it.**/
    private static final double TURN_IN_SPACING = 0.5;
    /**
     * The gap to try again with if the vehicle fits nowhere at the one it'd like, in blocks.  A wide vehicle by a player
     * in the middle of a narrow street can't leave much of a gap, and pulling up closer beats not pulling up at all.
     */
    static final double TIGHT_GAP = 0.5;
    /**How far apart the distances further out than the gap a goal is tried at are, in blocks.**/
    private static final double EXTRA_GAP_STEP = 0.5;
    /**Half the width of the player, which the vehicle must keep out of the block columns of.**/
    private static final double PLAYER_RADIUS = 0.3;
    /**How tall the player is, which they need clear to walk to the vehicle, and how far apart that's checked, in blocks.**/
    private static final double PLAYER_HEIGHT = 1.8;
    private static final double WALK_SPACING = 0.25;
    /**Most goals to keep, best first.**/
    static final int MAX_GOALS = 8;

    /**What the vehicle wants from its goal.**/
    static final class Settings {
        /**Which side faces the player, in degrees round from its left towards its front.**/
        double arrivalAngle;
        /**Gap between the player and the nearest part of the vehicle, in blocks.**/
        double gap = 1.2;
        /**The point on the vehicle to line up opposite the player, in its local space, usually the driver's seat.**/
        double alignX;
        double alignZ;
        /**
         * Shortest straight run-in a goal must have, and the length it should have, in blocks.  The shortest is
         * raised to the vehicle's length, and the length it should have to twice that, so long vehicles have the
         * room to straighten up.
         */
        double minRunIn = 3;
        double preferredRunIn = 8;
        /**Radius of the turns the vehicle makes coming in to the run-in, in blocks, with any margin for steering error.**/
        double turnRadius = 4;
        /**
         * True if the vehicle drives in, so goals need a run-in and room to turn in to it.  False if it's only put there,
         * like an aircraft placed on the ground or a vehicle that can't drive in, so it only needs room to park.
         */
        boolean drivesIn = true;
        /**How much further out than the gap a goal may go, to have room to turn in or to fit at all, in blocks.**/
        double maxExtraGap = 4;
        /**How far above and below the player a goal may be, in blocks.**/
        int heightUp = 2;
        int heightDown = 3;
        /**Most the vehicle can climb and drop between neighbouring blocks.**/
        int maxUp = 1;
        int maxDown = 2;
        /**True if the vehicle can turn on the spot, as vehicles that skid steer can.**/
        boolean canPivot;
    }

    /**
     * How far the player must be outside the circle a vehicle sweeps turning on the spot, in blocks: their own width
     * and the gap it'd leave them pulling up.
     */
    static double getPivotPlayerClearance(Settings settings) {
        return PLAYER_RADIUS + settings.gap;
    }

    private final PoseChecker checker;
    private final VehicleShape shape;
    private final Settings settings;
    private final Point3D player;
    private final double lookYaw;
    private final double headroom;
    private final double minRunIn;
    private final double preferredRunIn;
    /**Bearings to try, as offsets from straight ahead, nearest straight ahead first, and the next to try.**/
    private final List<Double> offsets = new ArrayList<>();
    private int offsetIndex;
    /**The gap being tried, which is the one in the settings until nothing fits at that.**/
    private double gap;
    private final List<Goal> goals = new ArrayList<>();

    ArrivalPlanner(PoseChecker checker, Settings settings, Point3D player, double lookYaw) {
        this.checker = checker;
        this.shape = checker.shape;
        this.settings = settings;
        this.player = player.copy();
        this.lookYaw = lookYaw;
        this.headroom = shape.top - shape.bottom;
        this.gap = settings.gap;
        double length = shape.bodyLength;
        this.minRunIn = Math.max(settings.minRunIn, length);
        this.preferredRunIn = Math.max(settings.preferredRunIn, 2 * length);
        //Nothing may drive through the player, coming in or pulling up.
        checker.keepClear(player, PLAYER_RADIUS);
        offsets.add(0D);
        for (double offset = BEARING_STEP; offset < 180; offset += BEARING_STEP) {
            offsets.add(offset);
            offsets.add(-offset);
        }
        offsets.add(180D);
    }

    /**Tries bearings until they're all done or maxNanos pass.  Returns true once they're all done.**/
    boolean step(long maxNanos) {
        long startTime = System.nanoTime();
        while (offsetIndex < offsets.size() && System.nanoTime() - startTime < maxNanos) {
            tryBearing(offsets.get(offsetIndex++));
            if (offsetIndex >= offsets.size() && goals.isEmpty() && gap > TIGHT_GAP) {
                //It fits nowhere.  Try every bearing again, pulling up closer.
                gap = TIGHT_GAP;
                offsetIndex = 0;
            }
        }
        return offsetIndex >= offsets.size();
    }

    /**Tries every bearing.  For tests and anything else that can wait.**/
    void finish() {
        while (!step(Long.MAX_VALUE)) {
        }
    }

    /**The gap being aimed for: the one in the settings, or {@link #TIGHT_GAP} if nothing fitted at that.**/
    double getGap() {
        return gap;
    }

    /**The goals found, cheapest first, at most {@link #MAX_GOALS}.  Empty if the vehicle fits nowhere by the player.**/
    List<Goal> getGoals() {
        return goals;
    }

    /**The bearing cost of the offset from straight ahead, in degrees.**/
    static double getBearingCost(double offset) {
        double angle = Math.abs(offset);
        double cost = BEARING_COST * angle / BEARING_STEP;
        if (angle > FRONT_LIMIT + 0.01) {
            cost += SIDE_COST;
        }
        if (angle > SIDE_LIMIT + 0.01) {
            cost += BEHIND_COST;
        }
        return cost;
    }

    /**
     * The yaw a vehicle at the bearing from the player faces to have the side at the arrival angle towards them.
     * Its left is 90 degrees anticlockwise of its nose, so with its left to the player its nose points 90 degrees
     * round from the bearing, and each degree the angle goes round towards its front turns it a degree further.
     */
    static double getArrivalYaw(double bearing, double arrivalAngle) {
        return VehicleSummonFlight.clampAngle(bearing + 90 + arrivalAngle);
    }

    /**
     * Where the vehicle's origin goes at the bearing from the player, as {x, z, yaw}: lined up so the align point is
     * opposite the player, and as near as it can be with every part of it at least the gap away from them.
     */
    double[] getOrigin(double bearing) {
        return getOrigin(bearing, gap);
    }

    /**Like {@link #getOrigin(double)}, but with this gap rather than the one in the settings.**/
    double[] getOrigin(double bearing, double gap) {
        double yaw = getArrivalYaw(bearing, settings.arrivalAngle);
        double sin = Math.sin(Math.toRadians(yaw));
        double cos = Math.cos(Math.toRadians(yaw));
        //The way to the player in the vehicle's local space, and the align point's offset across that way.
        double towardsX = Math.cos(Math.toRadians(settings.arrivalAngle));
        double towardsZ = Math.sin(Math.toRadians(settings.arrivalAngle));
        double along = settings.alignX * towardsX + settings.alignZ * towardsZ;
        double acrossX = settings.alignX - along * towardsX;
        double acrossZ = settings.alignZ - along * towardsZ;
        double limit = shape.radius() + gap + 1;
        double originX = player.x;
        double originZ = player.z;
        for (double distance = 0; distance <= limit; distance += GAP_SEARCH_STEP) {
            //The player is at the align point's offset across, this far along the way to them, in local space.
            double localX = acrossX + towardsX * distance;
            double localZ = acrossZ + towardsZ * distance;
            originX = player.x - (localX * cos + localZ * sin);
            originZ = player.z - (-localX * sin + localZ * cos);
            if (getNearestDistance(originX, originZ, yaw) >= gap) {
                break;
            }
        }
        return new double[] { originX, originZ, yaw };
    }

    /**How near the player any part of the vehicle comes, across the ground, with its origin here facing the yaw.**/
    double getNearestDistance(double originX, double originZ, double yaw) {
        double sin = Math.sin(Math.toRadians(yaw));
        double cos = Math.cos(Math.toRadians(yaw));
        double nearest = Double.MAX_VALUE;
        for (ShapeBox box : shape.boxes) {
            double centerX = originX + box.x * cos + box.z * sin;
            double centerZ = originZ - box.x * sin + box.z * cos;
            double dx = Math.max(0, Math.abs(player.x - centerX) - box.radius);
            double dz = Math.max(0, Math.abs(player.z - centerZ) - box.radius);
            nearest = Math.min(nearest, Math.hypot(dx, dz));
        }
        return nearest;
    }

    /**
     * Finds the goal at the bearing offset from straight ahead, if there is one: as near the player as it can be with
     * room to turn in, and at the height nearest the player's it fits at there.
     */
    private void tryBearing(double offset) {
        for (double extraGap = 0; extraGap <= settings.maxExtraGap + 1e-9; extraGap += EXTRA_GAP_STEP) {
            if (!canWalk(lookYaw + offset, gap + extraGap)) {
                //Something's in the way, so anywhere further out is past it, where the player can't get to the vehicle.
                return;
            }
            double[] origin = getOrigin(lookYaw + offset, gap + extraGap);
            Goal goal = tryOrigin(origin[0], origin[1], origin[2], getBearingCost(offset) + extraGap * EXTRA_GAP_COST);
            if (goal != null) {
                addGoal(goal);
                return;
            }
        }
    }

    /**
     * Returns true if the player can walk the distance out from where they stand at the bearing, to get to the vehicle
     * pulled up there: there's ground and room for them all the way, on their level, a step up, or a step down.
     */
    boolean canWalk(double bearing, double distance) {
        double alongX = Math.sin(Math.toRadians(bearing));
        double alongZ = Math.cos(Math.toRadians(bearing));
        int playerY = (int) Math.floor(player.y);
        for (double along = WALK_SPACING; along <= distance + 1e-9; along += WALK_SPACING) {
            int x = (int) Math.floor(player.x + alongX * along);
            int z = (int) Math.floor(player.z + alongZ * along);
            TerrainMap terrain = checker.terrain;
            boolean onLevel = terrain.isClear(x, playerY, z, TerrainMap.STEP_CLEARANCE, PLAYER_HEIGHT) && (terrain.isSolid(x, playerY - 1, z) || terrain.isSolid(x, playerY - 2, z));
            boolean stepUp = terrain.isSolid(x, playerY, z) && terrain.isClear(x, playerY + 1, z, 0, PLAYER_HEIGHT);
            if (!onLevel && !stepUp) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns the goal with the vehicle's origin over x,z facing the yaw, at the height nearest the player's it can
     * park, and if it drives in, has a long enough run-in and can turn in at, or null if there isn't one.
     */
    private Goal tryOrigin(double x, double z, double yaw, double cost) {
        int playerY = (int) Math.floor(player.y);
        for (int dy = 0; dy <= Math.max(settings.heightUp, settings.heightDown); ++dy) {
            for (int y : dy == 0 ? new int[] { playerY } : new int[] { playerY + dy, playerY - dy }) {
                if ((y > playerY && y - playerY > settings.heightUp) || (y < playerY && playerY - y > settings.heightDown) || !checker.canPark(x, y, z, yaw)) {
                    continue;
                }
                if (!settings.drivesIn) {
                    return new Goal(x, y, z, yaw, 0, cost + Math.abs(y - playerY) * HEIGHT_COST);
                }
                double[] runIn = measureRunIn(x, y, z, yaw);
                if (runIn[0] < minRunIn) {
                    continue;
                }
                if (runIn[0] < preferredRunIn) {
                    //It turns onto the run-in close by, so it needs room to.
                    double forwardX = Math.sin(Math.toRadians(yaw));
                    double forwardZ = Math.cos(Math.toRadians(yaw));
                    double startX = x - forwardX * runIn[0];
                    double startZ = z - forwardZ * runIn[0];
                    if (!canTurnIn(startX, (int) runIn[2], startZ, yaw) && !(settings.canPivot && canPivotAt(startX, (int) runIn[2], startZ))) {
                        continue;
                    }
                }
                return new Goal(x, y, z, yaw, runIn[0], cost + Math.abs(y - playerY) * HEIGHT_COST + (preferredRunIn - runIn[0]) * RUN_IN_SHORT_COST + runIn[1] * RUN_IN_STEP_COST);
            }
        }
        return null;
    }

    /**
     * Returns true if the vehicle can turn onto a straight that starts with its origin at x,z on the ground at y,
     * facing the yaw: a quarter turn at the turn radius, either way, has room for the vehicle all along it.
     */
    boolean canTurnIn(double x, int y, double z, double yaw) {
        return canTurnIn(x, y, z, yaw, 1) || canTurnIn(x, y, z, yaw, -1);
    }

    /**
     * Returns true if the vehicle could turn on the spot, about its origin at x,z on the ground at y, to face any way:
     * its shape fits at every yaw round, close enough that no corner moves more than {@link #TURN_IN_SPACING}
     * between them, and the player is well outside the circle its furthest point sweeps.
     */
    boolean canPivotAt(double x, int y, double z) {
        if (Math.hypot(player.x - x, player.z - z) < shape.radius() + getPivotPlayerClearance(settings)) {
            return false;
        }
        int steps = (int) Math.ceil(2 * Math.PI * Math.max(0.5, shape.radius()) / TURN_IN_SPACING);
        for (int i = 0; i < steps; ++i) {
            if (!checker.canDrive(x, y, z, 360D * i / steps)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true if the quarter turn onto the straight has room, turning left onto it if side is 1, right if -1.
     * Turning left onto it, the vehicle came in from its left heading across to its right, round a circle centered
     * on its left.  It's walked back from the straight, finding the ground at each point from the one after it, the
     * way driving it climbs and drops.
     */
    private boolean canTurnIn(double x, int y, double z, double yaw, int side) {
        double radius = settings.turnRadius;
        if (Double.isInfinite(radius) || Double.isNaN(radius)) {
            return false;
        }
        double forwardX = Math.sin(Math.toRadians(yaw));
        double forwardZ = Math.cos(Math.toRadians(yaw));
        //The vehicle's left, in the world.
        double leftX = forwardZ;
        double leftZ = -forwardX;
        int samples = Math.max(1, (int) Math.ceil(radius * Math.PI / 2 / TURN_IN_SPACING));
        int laterY = y;
        for (int i = 1; i <= samples; ++i) {
            double angle = Math.PI / 2 * i / samples;
            double across = side * radius * (1 - Math.cos(angle));
            double back = radius * Math.sin(angle);
            double pointX = x + leftX * across - forwardX * back;
            double pointZ = z + leftZ * across - forwardZ * back;
            int pointY = checker.terrain.findSurface((int) Math.floor(pointX), (int) Math.floor(pointZ), laterY, settings.maxDown, settings.maxUp, headroom);
            if (pointY == TerrainMap.NO_SURFACE || !checker.canDrive(pointX, pointY, pointZ, yaw - side * Math.toDegrees(angle))) {
                return false;
            }
            laterY = pointY;
        }
        return true;
    }

    /**
     * How far back from the goal the vehicle can drive straight in from, up to the preferred run-in, how many
     * steps up or down there are along it, and the ground height where it starts, as {length, steps, height}.  Walking back, each point's ground is found from the
     * one after it, the way driving in climbs and drops.
     */
    double[] measureRunIn(double x, int y, double z, double yaw) {
        double forwardX = Math.sin(Math.toRadians(yaw));
        double forwardZ = Math.cos(Math.toRadians(yaw));
        int laterY = y;
        int startY = y;
        double length = 0;
        int steps = 0;
        for (double back = RUN_IN_SPACING; back <= preferredRunIn + 1e-9; back += RUN_IN_SPACING) {
            double pointX = x - forwardX * back;
            double pointZ = z - forwardZ * back;
            //Driving on from here climbs to the later point if it's below it, and drops if above.
            int pointY = checker.terrain.findSurface((int) Math.floor(pointX), (int) Math.floor(pointZ), laterY, settings.maxDown, settings.maxUp, headroom);
            if (pointY == TerrainMap.NO_SURFACE || !checker.canDrive(pointX, pointY, pointZ, yaw)) {
                break;
            }
            if (pointY != laterY) {
                ++steps;
            }
            laterY = pointY;
            startY = pointY;
            length = back;
        }
        return new double[] { length, steps, startY };
    }

    private void addGoal(Goal goal) {
        int index = 0;
        while (index < goals.size() && goals.get(index).cost <= goal.cost) {
            ++index;
        }
        if (index < MAX_GOALS) {
            goals.add(index, goal);
            if (goals.size() > MAX_GOALS) {
                goals.remove(goals.size() - 1);
            }
        }
    }

    /**
     * The point to line up opposite the player, in the vehicle's local space as {x, z}: the controller seat nearest
     * the side that faces them, or the origin if it has no controller seat.
     */
    static double[] getAlignPoint(EntityVehicleF_Physics vehicle, double arrivalAngle) {
        double towardsX = Math.cos(Math.toRadians(arrivalAngle));
        double towardsZ = Math.sin(Math.toRadians(arrivalAngle));
        double[] best = { 0, 0 };
        double bestAlong = -Double.MAX_VALUE;
        for (APart part : vehicle.allParts) {
            if (part instanceof PartSeat && part.placementDefinition.isController) {
                Point3D local = part.position.copy().subtract(vehicle.position).reOrigin(vehicle.orientation);
                double along = local.x * towardsX + local.z * towardsZ;
                if (along > bestAlong) {
                    bestAlong = along;
                    best = new double[] { local.x, local.z };
                }
            }
        }
        return best;
    }

    /**Where the vehicle pulls up: its origin over x,z, its wheels on the ground at y, facing the yaw.**/
    static final class Goal {
        final double x;
        final int y;
        final double z;
        final double yaw;
        /**How long the clear straight run-in behind it is, in blocks.**/
        final double runIn;
        /**Extra cost of ending a route here, in blocks of route.**/
        final double cost;

        Goal(double x, int y, double z, double yaw, double runIn, double cost) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.runIn = runIn;
            this.cost = cost;
        }

        /**The ground point under the vehicle origin.**/
        Point3D getGround() {
            return new Point3D(x, y, z);
        }
    }
}
