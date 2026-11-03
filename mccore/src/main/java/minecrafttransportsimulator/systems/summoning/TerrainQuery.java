package minecrafttransportsimulator.systems.summoning;

import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.blocks.components.ABlockBase.Axis;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;

/**
 * The block questions summon planning asks of the world.  Planning only goes through this, so it can be run
 * against a made-up world in tests.  Answers aren't cached here: callers cache what they need.
 */
interface TerrainQuery {
    /**Returns true if the block has a solid top to stand on.**/
    boolean isSolid(int x, int y, int z);

    /**Returns true if the block is a liquid.**/
    boolean isLiquid(int x, int y, int z);

    /**Returns true if nothing in the block column at x,z collides between the heights, slightly inset from the column's sides.**/
    boolean isClear(int x, int z, double bottomY, double topY);

    /**Returns true if the chunk holding the block column is loaded.  Nothing is known about unloaded columns, so they're treated as blocked.**/
    boolean isLoaded(int x, int z);

    /**Returns true if no block is in the way on the straight line between the points.**/
    boolean isLineClear(double x1, double y1, double z1, double x2, double y2, double z2);

    /**Asks the world itself.**/
    static TerrainQuery of(AWrapperWorld world) {
        return new TerrainQuery() {
            @Override
            public boolean isSolid(int x, int y, int z) {
                return world.isBlockSolid(new Point3D(x + 0.5, y + 0.5, z + 0.5), Axis.UP);
            }

            @Override
            public boolean isLiquid(int x, int y, int z) {
                return world.isBlockLiquid(new Point3D(x + 0.5, y + 0.5, z + 0.5));
            }

            @Override
            public boolean isClear(int x, int z, double bottomY, double topY) {
                BoundingBox box = new BoundingBox(new Point3D(x + 0.5, (bottomY + topY) / 2D, z + 0.5), 0.49, (topY - bottomY) / 2D, 0.49);
                return !world.checkForCollisions(box, new Point3D(), false, false);
            }

            @Override
            public boolean isLoaded(int x, int z) {
                return world.chunkLoaded(new Point3D(x + 0.5, 0, z + 0.5));
            }

            @Override
            public boolean isLineClear(double x1, double y1, double z1, double x2, double y2, double z2) {
                return world.getBlockHit(new Point3D(x1, y1, z1), new Point3D(x2 - x1, y2 - y1, z2 - z1)) == null;
            }
        };
    }
}
