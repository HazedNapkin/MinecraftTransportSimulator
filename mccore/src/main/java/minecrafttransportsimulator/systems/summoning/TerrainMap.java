package minecrafttransportsimulator.systems.summoning;

import java.util.HashMap;
import java.util.Map;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * What summon planning knows about the blocks round the player, cached, as planning asks about the same blocks
 * again and again.  One is made per summon, so the cache never goes stale by much.
 * <p>
 * A surface is somewhere a wheel can rest: a block with a solid top, and room above it.  Its height is the y of the
 * block the wheel sits in, so a surface at y is the top of the solid block at y - 1.  A column can have several, like
 * a road with a bridge over it, so heights are always looked up near a height we already have.
 */
final class TerrainMap {
    static final int NO_SURFACE = Integer.MIN_VALUE;
    /**
     * Height above a surface that things may stick up without blocking it, in blocks.  Wheels roll over slabs,
     * carpets and snow lower than this, and the body passes over them.
     */
    static final double STEP_CLEARANCE = 0.6;

    private final TerrainQuery query;
    /**What each block is, as {@link TerrainQuery#getBlock} flags.  Solid and liquid are asked together, as they're one lookup.**/
    private final Map<Long, Integer> blockCache = new HashMap<>();
    private final Map<Long, Map<Integer, Boolean>> clearCache = new HashMap<>();
    /**Whether each chunk is loaded, keyed by {@link VehicleShape#columnKey(int, int)} of the chunk.**/
    private final Map<Long, Boolean> loadedCache = new HashMap<>();

    TerrainMap(TerrainQuery query) {
        this.query = query;
    }

    /**Returns true if the block has a solid top to stand on.**/
    boolean isSolid(int x, int y, int z) {
        return (getBlock(x, y, z) & TerrainQuery.SOLID) != 0;
    }

    boolean isLiquid(int x, int y, int z) {
        return (getBlock(x, y, z) & TerrainQuery.LIQUID) != 0;
    }

    private int getBlock(int x, int y, int z) {
        return blockCache.computeIfAbsent(blockKey(x, y, z), k -> query.getBlock(x, y, z));
    }

    /**Returns true if the column's chunk is loaded.**/
    boolean isLoaded(int x, int z) {
        return loadedCache.computeIfAbsent(VehicleShape.columnKey(x >> 4, z >> 4), k -> query.isLoaded(x, z));
    }

    /**
     * Returns true if nothing in the block column at x,z collides between low and high above height y.
     * Heights are rounded outwards to 1/8 of a block so nearby checks share cached results.
     */
    boolean isClear(int x, int y, int z, double low, double high) {
        int lowEighths = (int) Math.floor(low * 8);
        int highEighths = (int) Math.ceil(high * 8);
        if (highEighths <= lowEighths) {
            return true;
        }
        Map<Integer, Boolean> results = clearCache.computeIfAbsent(blockKey(x, y, z), k -> new HashMap<>());
        int rangeKey = (lowEighths << 16) | (highEighths & 0xFFFF);
        Boolean clear = results.get(rangeKey);
        if (clear == null) {
            clear = query.isClear(x, z, y + lowEighths / 8D, y + highEighths / 8D);
            results.put(rangeKey, clear);
        }
        return clear;
    }

    /**
     * Returns true if a wheel can rest at height y in the column, with headroom blocks clear above it from
     * {@link #STEP_CLEARANCE} up.  Liquid and unloaded columns never count.
     */
    boolean isSurface(int x, int y, int z, double headroom) {
        return isLoaded(x, z) && isSolid(x, y - 1, z) && !isLiquid(x, y, z) && isClear(x, y, z, STEP_CLEARANCE, headroom);
    }

    /**
     * Finds the surface in the column from refY - down to refY + up, nearest refY first, and the higher
     * first of two as near.  Returns {@link #NO_SURFACE} if there isn't one.
     */
    int findSurface(int x, int z, int refY, int up, int down, double headroom) {
        for (int offset = 0; offset <= Math.max(up, down); ++offset) {
            if (offset <= up && isSurface(x, refY + offset, z, headroom)) {
                return refY + offset;
            }
            if (offset > 0 && offset <= down && isSurface(x, refY - offset, z, headroom)) {
                return refY - offset;
            }
        }
        return NO_SURFACE;
    }

    /**
     * Returns true if a vehicle at height y could have a wheel in the column: there's a surface no more than
     * up above y or down below it.  Columns that aren't open are in the way.
     */
    boolean isOpen(int x, int y, int z, int up, int down, double headroom) {
        return findSurface(x, z, y, up, down, headroom) != NO_SURFACE;
    }

    /**Returns true if no block is in the way on the straight line between the points.  Not cached, as lines are rarely asked twice.**/
    boolean isLineClear(Point3D from, Point3D to) {
        return query.isLineClear(from.x, from.y, from.z, to.x, to.y, to.z);
    }

    static long blockKey(int x, int y, int z) {
        return ((long) (x & 0x3FFFFFF) << 38) | ((long) (z & 0x3FFFFFF) << 12) | (y & 0xFFF);
    }
}
