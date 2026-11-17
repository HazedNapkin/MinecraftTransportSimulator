package minecrafttransportsimulator.systems.navigation;

import java.util.BitSet;

/**
 * A made-up world of full blocks for planning tests.  Everything is loaded, and there's no liquid unless added.
 * <p>
 * Blocks are kept as bits rather than in a set, so asking about one allocates nothing, and the memory a test sees
 * planning allocate is planning's own.  It reaches {@link #REACH} blocks out from the origin, and from {@link #MIN_Y}
 * up to {@link #MAX_Y}: there's nothing beyond, and nothing can be put there.
 *
 * @author HazedNapkin
 */
public final class FakeTerrain implements ITerrainQuery {
    public static final int REACH = 256;
    public static final int MIN_Y = -16;
    public static final int MAX_Y = 47;
    private final BitSet solid = new BitSet();
    private final BitSet liquid = new BitSet();

    /**The bit for the block, or -1 if it's further out than the world reaches.**/
    private static int index(int x, int y, int z) {
        if (x < -REACH || x >= REACH || z < -REACH || z >= REACH || y < MIN_Y || y > MAX_Y) {
            return -1;
        }
        return ((x + REACH) << 15) | ((z + REACH) << 6) | (y - MIN_Y);
    }

    private static int indexToSet(int x, int y, int z) {
        int index = index(x, y, z);
        if (index < 0) {
            throw new IllegalArgumentException("Block " + x + "," + y + "," + z + " is further out than a FakeTerrain reaches");
        }
        return index;
    }

    /**Makes flat ground with its top at y = 0, so surfaces are at y = 0, over the square from -size to size.**/
    public static FakeTerrain flat(int size) {
        FakeTerrain terrain = new FakeTerrain();
        terrain.fill(-size, -1, -size, size, -1, size);
        return terrain;
    }

    /**Fills the box of blocks, corners included.**/
    public FakeTerrain fill(int x1, int y1, int z1, int x2, int y2, int z2) {
        for (int x = Math.min(x1, x2); x <= Math.max(x1, x2); ++x) {
            for (int y = Math.min(y1, y2); y <= Math.max(y1, y2); ++y) {
                for (int z = Math.min(z1, z2); z <= Math.max(z1, z2); ++z) {
                    solid.set(indexToSet(x, y, z));
                }
            }
        }
        return this;
    }

    public FakeTerrain addLiquid(int x, int y, int z) {
        liquid.set(indexToSet(x, y, z));
        return this;
    }

    @Override
    public int getBlock(int x, int y, int z) {
        int index = index(x, y, z);
        return index < 0 ? 0 : (solid.get(index) ? SOLID : 0) | (liquid.get(index) ? LIQUID : 0);
    }

    public boolean isSolid(int x, int y, int z) {
        int index = index(x, y, z);
        return index >= 0 && solid.get(index);
    }

    @Override
    public boolean isClear(int x, int z, double bottomY, double topY) {
        for (int y = (int) Math.floor(bottomY); y < topY; ++y) {
            if (y + 1 > bottomY && isSolid(x, y, z)) {
                return false;
            }
        }
        return true;
    }

    @Override
    public boolean isLoaded(int x, int z) {
        return true;
    }

    @Override
    public boolean isLineClear(double x1, double y1, double z1, double x2, double y2, double z2) {
        double length = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
        int samples = Math.max(1, (int) Math.ceil(length / 0.05));
        for (int i = 0; i <= samples; ++i) {
            double fraction = (double) i / samples;
            if (isSolid((int) Math.floor(x1 + (x2 - x1) * fraction), (int) Math.floor(y1 + (y2 - y1) * fraction), (int) Math.floor(z1 + (z2 - z1) * fraction))) {
                return false;
            }
        }
        return true;
    }
}
