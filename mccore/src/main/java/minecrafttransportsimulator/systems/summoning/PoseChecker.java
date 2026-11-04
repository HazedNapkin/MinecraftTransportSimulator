package minecrafttransportsimulator.systems.summoning;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.systems.summoning.VehicleShape.Column;

/**
 * Checks whether a vehicle fits at a pose: its origin at any point, its wheels on the ground at a block height,
 * and facing any yaw.  It checks every block column the vehicle's shape covers there, so it accounts for the
 * vehicle's length, width and overhangs whichever way it faces.
 * <p>
 * Parking is strict: everything clear and every wheel on level ground, as the vehicle is put there.  Driving
 * lets the body pass over low bumps, pitch up onto steps, and wheels roll through dips, as it does when driving.
 */
final class PoseChecker {
    /**Height the vehicle is placed above the ground when teleported, so nothing lower needs to be clear.**/
    static final double PARKING_CLEARANCE = 0.1;
    /**
     * Yaws to try landing an aircraft at, as offsets from the one it'd like: as it'd like, side-on both ways,
     * reversed, then the diagonals.
     */
    private static final double[] LANDING_YAW_ORDER = { 0, 90, -90, 180, 45, -45, 135, -135 };
    /**How far apart the yaws an aircraft turns through on its way down are checked, in degrees.**/
    private static final double LANDING_TURN_STEP = 15;

    final TerrainMap terrain;
    final VehicleShape shape;
    /**Block columns no part of the vehicle may be in, keyed by {@link VehicleShape#columnKey(int, int)}.**/
    private final Set<Long> keptClear = new HashSet<>();
    /**
     * Whether a column can be driven through at a height, by {@link TerrainMap#blockKey} and then the height range
     * needed clear in eighths, as {@link TerrainMap#isClear} rounds it.  Driving checks the same columns over and
     * over, and this answers each in one lookup rather than the several the check takes.
     */
    private final Map<Long, Map<Integer, Boolean>> drivableColumns = new HashMap<>();

    PoseChecker(TerrainMap terrain, VehicleShape shape) {
        this.terrain = terrain;
        this.shape = shape;
    }

    /**
     * Keeps the vehicle out of every block column within radius blocks of the position, square on.  Used to keep
     * it off the player it's coming to, as a long vehicle swinging round could sweep right through them.
     */
    void keepClear(Point3D position, double radius) {
        for (int x = (int) Math.floor(position.x - radius); x <= (int) Math.floor(position.x + radius); ++x) {
            for (int z = (int) Math.floor(position.z - radius); z <= (int) Math.floor(position.z + radius); ++z) {
                keptClear.add(VehicleShape.columnKey(x, z));
            }
        }
        drivableColumns.clear();
    }

    /**Returns true if the vehicle can be placed with its origin over x,z, its wheels on the ground at height y, facing the yaw.**/
    boolean canPark(double x, int y, double z, double yaw) {
        if (!terrain.isSolid((int) Math.floor(x), y - 1, (int) Math.floor(z))) {
            return false;
        }
        for (Column column : shape.getFootprint(x, z, yaw)) {
            if (isBlocked(column, y) || !terrain.isClear(column.x, y, column.z, Math.max(column.low, PARKING_CLEARANCE), column.high)) {
                return false;
            }
        }
        for (Column support : shape.getSupports(x, z, yaw)) {
            if (!terrain.isSolid(support.x, y - 1, support.z)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true if the vehicle can drive with its origin over x,z, its wheels at height y, facing the yaw.
     * Parts of it may hang over terrain up to {@link TerrainMap#STEP_CLEARANCE} high, or over a step it can
     * pitch up onto, and each wheel may be on the ground, on a step, or in a dip it rolls through.
     */
    boolean canDrive(double x, int y, double z, double yaw) {
        if (!terrain.isSolid((int) Math.floor(x), y - 1, (int) Math.floor(z))) {
            return false;
        }
        for (Column column : shape.getFootprint(x, z, yaw)) {
            if (!canDriveThrough(column, y)) {
                return false;
            }
        }
        List<Column> supports = shape.getSupports(x, z, yaw);
        for (Column support : supports) {
            if (!terrain.isSolid(support.x, y - 1, support.z) && !terrain.isSolid(support.x, y, support.z) && !terrain.isSolid(support.x, y - 2, support.z)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true if an aircraft facing the yaw can come straight down from its origin being at fromY to land with its
     * origin over x,z and its wheels on the ground at height y.  Every column it covers must be clear all the way down,
     * and each wheel must come down within a block of that ground, so it doesn't land half off a ledge.
     */
    boolean canLand(double x, int y, double z, double yaw, double fromY) {
        if (!isClearDown(x, y, z, yaw, fromY)) {
            return false;
        }
        for (Column support : shape.getSupports(x, z, yaw)) {
            if (!terrain.isSolid(support.x, y - 1, support.z) && !terrain.isSolid(support.x, y - 2, support.z)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Picks the yaw for an aircraft facing currentYaw to land at over x,z on the ground at y, coming down from its
     * origin being at fromY (see {@link #canLand}).  Yaws are tried round from the one it'd like.
     * <p>
     * It prefers a yaw it can turn to smoothly on the way down, which needs every yaw it turns through to be clear all
     * the way down too.  Returns {yaw, 1} for one of those.  Failing that, it returns {yaw, 0} for the first it can land
     * at once it's finished turning, as it can turn up at cruise height where there's nothing in the way.  Returns null
     * if it can't land here at all.
     */
    double[] findLandingYaw(double x, int y, double z, double preferredYaw, double currentYaw, double fromY) {
        double[] turnFirst = null;
        for (double offset : LANDING_YAW_ORDER) {
            double yaw = VehicleSummonFlight.clampAngle(preferredYaw + offset);
            if (canLand(x, y, z, yaw, fromY)) {
                if (canTurnOnWayDown(x, y, z, currentYaw, yaw, fromY)) {
                    return new double[] { yaw, 1 };
                }
                if (turnFirst == null) {
                    turnFirst = new double[] { yaw, 0 };
                }
            }
        }
        return turnFirst;
    }

    /**
     * Returns true if every yaw an aircraft turns through from fromYaw to toYaw, the short way round as the flight
     * turns, has room all the way down from its origin being at fromY to the ground at height y.
     */
    private boolean canTurnOnWayDown(double x, int y, double z, double fromYaw, double toYaw, double fromY) {
        double turn = VehicleSummonFlight.clampAngle(toYaw - fromYaw);
        int steps = (int) Math.ceil(Math.abs(turn) / LANDING_TURN_STEP);
        for (int i = 0; i < steps; ++i) {
            if (!isClearDown(x, y, z, fromYaw + turn * i / steps, fromY)) {
                return false;
            }
        }
        return true;
    }

    /**Returns true if every column the aircraft covers facing the yaw is clear from the ground at y up to it, from its origin being at fromY.**/
    private boolean isClearDown(double x, int y, double z, double yaw, double fromY) {
        double descent = fromY + shape.bottom - y;
        for (Column column : shape.getFootprint(x, z, yaw)) {
            if (terrain.isLiquid(column.x, y, column.z) || !terrain.isClear(column.x, y, column.z, Math.max(column.low, PARKING_CLEARANCE), descent + column.high)) {
                return false;
            }
        }
        return true;
    }

    /**
     * True if the vehicle can drive through the column at height y: it's not blocked, and the part of the vehicle over
     * it clears the terrain there, or would on a step it can pitch up onto.
     */
    private boolean canDriveThrough(Column column, int y) {
        double low = Math.max(column.low, TerrainMap.STEP_CLEARANCE);
        int rangeKey = ((int) Math.floor(low * 8) << 16) | ((int) Math.ceil(column.high * 8) & 0xFFFF);
        Map<Integer, Boolean> ranges = drivableColumns.computeIfAbsent(TerrainMap.blockKey(column.x, y, column.z), k -> new HashMap<>());
        Boolean drivable = ranges.get(rangeKey);
        if (drivable == null) {
            drivable = !isBlocked(column, y) && (terrain.isClear(column.x, y, column.z, low, column.high) || (terrain.isSolid(column.x, y, column.z) && terrain.isClear(column.x, y + 1, column.z, low, column.high)));
            ranges.put(rangeKey, drivable);
        }
        return drivable;
    }

    /**True if the vehicle can't be in the column at all at height y: it's kept clear, unloaded, or liquid.**/
    private boolean isBlocked(Column column, int y) {
        return keptClear.contains(VehicleShape.columnKey(column.x, column.z)) || !terrain.isLoaded(column.x, column.z) || terrain.isLiquid(column.x, y, column.z);
    }
}
