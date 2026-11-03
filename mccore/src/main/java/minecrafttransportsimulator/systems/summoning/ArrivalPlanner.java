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
 * straight rather than stopping mid-turn.  Goals are ranked by how far round from straight ahead they are, how far
 * above or below the player, and how short or bumpy their run-in is.
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
    /**Costs of goals for each block above or below the player, for each block their run-in is short, and for each step in it.**/
    private static final double HEIGHT_COST = 1;
    private static final double RUN_IN_SHORT_COST = 1;
    private static final double RUN_IN_STEP_COST = 0.5;
    /**How far apart the run-in is checked, and how finely the gap is found, in blocks.**/
    private static final double RUN_IN_SPACING = 0.5;
    private static final double GAP_SEARCH_STEP = 0.05;
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
        /**Shortest straight run-in a goal must have, and the length it should have, in blocks.**/
        double minRunIn = 3;
        double preferredRunIn = 8;
        /**How far above and below the player a goal may be, in blocks.**/
        int heightUp = 2;
        int heightDown = 3;
        /**Most the vehicle can climb and drop between neighbouring blocks.**/
        int maxUp = 1;
        int maxDown = 2;
    }

    private final PoseChecker checker;
    private final VehicleShape shape;
    private final Settings settings;
    private final Point3D player;
    private final double lookYaw;
    private final double headroom;
    /**Bearings to try, as offsets from straight ahead, nearest straight ahead first, and the next to try.**/
    private final List<Double> offsets = new ArrayList<>();
    private int offsetIndex;
    private final List<Goal> goals = new ArrayList<>();

    ArrivalPlanner(PoseChecker checker, Settings settings, Point3D player, double lookYaw) {
        this.checker = checker;
        this.shape = checker.shape;
        this.settings = settings;
        this.player = player.copy();
        this.lookYaw = lookYaw;
        this.headroom = shape.top - shape.bottom;
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
        }
        return offsetIndex >= offsets.size();
    }

    /**Tries every bearing.  For tests and anything else that can wait.**/
    void finish() {
        while (!step(Long.MAX_VALUE)) {
        }
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
        double yaw = getArrivalYaw(bearing, settings.arrivalAngle);
        double sin = Math.sin(Math.toRadians(yaw));
        double cos = Math.cos(Math.toRadians(yaw));
        //The way to the player in the vehicle's local space, and the align point's offset across that way.
        double towardsX = Math.cos(Math.toRadians(settings.arrivalAngle));
        double towardsZ = Math.sin(Math.toRadians(settings.arrivalAngle));
        double along = settings.alignX * towardsX + settings.alignZ * towardsZ;
        double acrossX = settings.alignX - along * towardsX;
        double acrossZ = settings.alignZ - along * towardsZ;
        double limit = shape.radius() + settings.gap + 1;
        double originX = player.x;
        double originZ = player.z;
        for (double distance = 0; distance <= limit; distance += GAP_SEARCH_STEP) {
            //The player is at the align point's offset across, this far along the way to them, in local space.
            double localX = acrossX + towardsX * distance;
            double localZ = acrossZ + towardsZ * distance;
            originX = player.x - (localX * cos + localZ * sin);
            originZ = player.z - (-localX * sin + localZ * cos);
            if (getNearestDistance(originX, originZ, yaw) >= settings.gap) {
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

    /**Finds the goal at the bearing offset from straight ahead, if there is one, at the height nearest the player's it fits at.**/
    private void tryBearing(double offset) {
        double[] origin = getOrigin(lookYaw + offset);
        double x = origin[0];
        double z = origin[1];
        double yaw = origin[2];
        int playerY = (int) Math.floor(player.y);
        for (int dy = 0; dy <= Math.max(settings.heightUp, settings.heightDown); ++dy) {
            for (int y : dy == 0 ? new int[] { playerY } : new int[] { playerY + dy, playerY - dy }) {
                if ((y > playerY && y - playerY > settings.heightUp) || (y < playerY && playerY - y > settings.heightDown)) {
                    continue;
                }
                if (checker.canPark(x, y, z, yaw)) {
                    double[] runIn = measureRunIn(x, y, z, yaw);
                    if (runIn[0] >= settings.minRunIn) {
                        double cost = getBearingCost(offset) + Math.abs(y - playerY) * HEIGHT_COST + (settings.preferredRunIn - runIn[0]) * RUN_IN_SHORT_COST + runIn[1] * RUN_IN_STEP_COST;
                        addGoal(new Goal(x, y, z, yaw, runIn[0], cost));
                        return;
                    }
                }
            }
        }
    }

    /**
     * How far back from the goal the vehicle can drive straight in from, up to the preferred run-in, and how many
     * steps up or down there are along it, as {length, steps}.  Walking back, each point's ground is found from the
     * one after it, the way driving in climbs and drops.
     */
    double[] measureRunIn(double x, int y, double z, double yaw) {
        double forwardX = Math.sin(Math.toRadians(yaw));
        double forwardZ = Math.cos(Math.toRadians(yaw));
        int laterY = y;
        double length = 0;
        int steps = 0;
        for (double back = RUN_IN_SPACING; back <= settings.preferredRunIn + 1e-9; back += RUN_IN_SPACING) {
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
            length = back;
        }
        return new double[] { length, steps };
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
