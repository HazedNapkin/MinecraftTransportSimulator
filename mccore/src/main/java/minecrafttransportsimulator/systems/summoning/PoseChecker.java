package minecrafttransportsimulator.systems.summoning;

import java.util.HashSet;
import java.util.List;
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

    final TerrainMap terrain;
    final VehicleShape shape;
    /**Block columns no part of the vehicle may be in, keyed by {@link VehicleShape#columnKey(int, int)}.**/
    private final Set<Long> keptClear = new HashSet<>();

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
    }

    /**Returns true if the vehicle can be placed with its origin over x,z, its wheels on the ground at height y, facing the yaw.**/
    boolean canPark(double x, int y, double z, double yaw) {
        if (!terrain.isSolid((int) Math.floor(x), y - 1, (int) Math.floor(z))) {
            return false;
        }
        for (Column column : shape.getFootprint(x, z, yaw).values()) {
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
        for (Column column : shape.getFootprint(x, z, yaw).values()) {
            if (isBlocked(column, y)) {
                return false;
            }
            double low = Math.max(column.low, TerrainMap.STEP_CLEARANCE);
            if (!terrain.isClear(column.x, y, column.z, low, column.high) && !(terrain.isSolid(column.x, y, column.z) && terrain.isClear(column.x, y + 1, column.z, low, column.high))) {
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

    /**True if the vehicle can't be in the column at all at height y: it's kept clear, unloaded, or liquid.**/
    private boolean isBlocked(Column column, int y) {
        return keptClear.contains(VehicleShape.columnKey(column.x, column.z)) || !terrain.isLoaded(column.x, column.z) || terrain.isLiquid(column.x, y, column.z);
    }
}
