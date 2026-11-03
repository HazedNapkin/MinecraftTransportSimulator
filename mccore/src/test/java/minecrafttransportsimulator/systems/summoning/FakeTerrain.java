package minecrafttransportsimulator.systems.summoning;

import java.util.HashSet;
import java.util.Set;

/**
 * A made-up world of full blocks for planning tests.  Everything is loaded, and there's no liquid unless added.
 */
final class FakeTerrain implements TerrainQuery {
    private final Set<Long> solid = new HashSet<>();
    private final Set<Long> liquid = new HashSet<>();

    /**Makes flat ground with its top at y = 0, so surfaces are at y = 0, over the square from -size to size.**/
    static FakeTerrain flat(int size) {
        FakeTerrain terrain = new FakeTerrain();
        terrain.fill(-size, -1, -size, size, -1, size);
        return terrain;
    }

    /**Fills the box of blocks, corners included.**/
    FakeTerrain fill(int x1, int y1, int z1, int x2, int y2, int z2) {
        for (int x = Math.min(x1, x2); x <= Math.max(x1, x2); ++x) {
            for (int y = Math.min(y1, y2); y <= Math.max(y1, y2); ++y) {
                for (int z = Math.min(z1, z2); z <= Math.max(z1, z2); ++z) {
                    solid.add(TerrainMap.blockKey(x, y, z));
                }
            }
        }
        return this;
    }

    FakeTerrain addLiquid(int x, int y, int z) {
        liquid.add(TerrainMap.blockKey(x, y, z));
        return this;
    }

    @Override
    public boolean isSolid(int x, int y, int z) {
        return solid.contains(TerrainMap.blockKey(x, y, z));
    }

    @Override
    public boolean isLiquid(int x, int y, int z) {
        return liquid.contains(TerrainMap.blockKey(x, y, z));
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
}
