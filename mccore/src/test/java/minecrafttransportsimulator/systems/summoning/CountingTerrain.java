package minecrafttransportsimulator.systems.summoning;

/**
 * Wraps a world and counts the questions planning asks it, for measuring.  In game each is a real world lookup, so
 * how many a summon makes matters more than how long a test takes.
 */
final class CountingTerrain implements TerrainQuery {
    private final TerrainQuery world;
    int blocks;
    int clear;
    int loaded;
    int lines;

    CountingTerrain(TerrainQuery world) {
        this.world = world;
    }

    /**Every block lookup made: what blocks are, collision checks and chunk checks.  Raycasts are counted apart.**/
    int getWorldCalls() {
        return blocks + clear + loaded;
    }

    @Override
    public int getBlock(int x, int y, int z) {
        ++blocks;
        return world.getBlock(x, y, z);
    }

    @Override
    public boolean isClear(int x, int z, double bottomY, double topY) {
        ++clear;
        return world.isClear(x, z, bottomY, topY);
    }

    @Override
    public boolean isLoaded(int x, int z) {
        ++loaded;
        return world.isLoaded(x, z);
    }

    @Override
    public boolean isLineClear(double x1, double y1, double z1, double x2, double y2, double z2) {
        ++lines;
        return world.isLineClear(x1, y1, z1, x2, y2, z2);
    }
}
