package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;

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
 * These are only where the search aims.  It can start the vehicle anywhere that's good enough on the way.
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
        int maxTargets = 6;
    }

    private final PoseChecker checker;
    private final Settings settings;
    private final Point3D player;
    private final double lookYaw;
    private final List<FlowField.Cell> cells;
    private int cellIndex;
    private final List<Target> scored = new ArrayList<>();
    private List<Target> targets;

    SpawnSelector(PoseChecker checker, FlowField goalField, Settings settings, Point3D player, double lookYaw) {
        this.checker = checker;
        this.settings = settings;
        this.player = player.copy();
        this.lookYaw = lookYaw;
        this.cells = goalField.getCells();
    }

    /**Looks over cells until they're all done or maxNanos pass.  Returns true once they're all done.**/
    boolean step(long maxNanos) {
        long startTime = System.nanoTime();
        while (cellIndex < cells.size() && System.nanoTime() - startTime < maxNanos) {
            score(cells.get(cellIndex++));
        }
        if (cellIndex >= cells.size() && targets == null) {
            pick();
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

    private void score(FlowField.Cell cell) {
        if (Math.floorMod(cell.x, settings.stride) != 0 || Math.floorMod(cell.z, settings.stride) != 0 || cell.distance < settings.minLength || cell.distance > settings.maxLength) {
            return;
        }
        double x = cell.x + 0.5;
        double z = cell.z + 0.5;
        double distance = Math.hypot(x - player.x, z - player.z);
        double yaw = cell.getYawToNext();
        if (distance < settings.minDistance || Double.isNaN(yaw) || !checker.canPark(x, cell.y, z, yaw)) {
            return;
        }
        boolean visible = isVisible(checker, player, x, cell.y, z);
        double offFront = Math.abs(VehicleSummonFlight.clampAngle(Math.toDegrees(Math.atan2(x - player.x, z - player.z)) - lookYaw));
        double cost = Math.abs(cell.distance - settings.targetLength) * LENGTH_COST + (visible ? VISIBLE_COST : 0) + offFront / 180 * BEHIND_COST;
        scored.add(new Target(x, cell.y, z, yaw, cell.distance, cost, visible));
    }

    /**Takes the best, skipping any too near a better one already taken.**/
    private void pick() {
        scored.sort((a, b) -> Double.compare(a.cost, b.cost));
        targets = new ArrayList<>();
        for (Target target : scored) {
            if (targets.size() >= settings.maxTargets) {
                break;
            }
            boolean tooNear = false;
            for (Target taken : targets) {
                if (Math.hypot(target.x - taken.x, target.z - taken.z) < MIN_SPACING) {
                    tooNear = true;
                    break;
                }
            }
            if (!tooNear) {
                targets.add(target);
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
