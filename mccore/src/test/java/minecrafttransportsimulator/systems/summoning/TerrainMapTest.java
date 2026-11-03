package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TerrainMapTest {
    @Test
    public void flatGroundIsASurface() {
        TerrainMap map = new TerrainMap(FakeTerrain.flat(4));
        assertTrue(map.isSurface(0, 0, 0, 2));
        assertFalse(map.isSurface(0, 1, 0, 2));
        assertFalse(map.isSurface(0, -1, 0, 2));
    }

    @Test
    public void needsHeadroomAndNoLiquid() {
        FakeTerrain terrain = FakeTerrain.flat(4).fill(1, 2, 0, 1, 2, 0).addLiquid(2, 0, 0);
        TerrainMap map = new TerrainMap(terrain);
        //A block 2 up leaves room for something 2 high but not 3.
        assertTrue(map.isSurface(1, 0, 0, 2));
        assertFalse(map.isSurface(1, 0, 0, 3));
        assertFalse(map.isSurface(2, 0, 0, 2));
    }

    @Test
    public void bridgeGivesTwoSurfaces() {
        //A bridge deck at y = 4, with its top making a surface at 5 over the road at 0.
        TerrainMap map = new TerrainMap(FakeTerrain.flat(4).fill(-4, 4, 0, 4, 4, 0));
        assertTrue(map.isSurface(0, 0, 0, 3));
        assertTrue(map.isSurface(0, 5, 0, 3));
        assertEquals(0, map.findSurface(0, 0, 1, 1, 2, 3));
        assertEquals(5, map.findSurface(0, 0, 4, 1, 2, 3));
    }

    @Test
    public void findSurfacePrefersNearestThenHigher() {
        //A step up at x = 1.
        TerrainMap map = new TerrainMap(FakeTerrain.flat(4).fill(1, 0, -4, 4, 0, 4));
        assertEquals(1, map.findSurface(1, 0, 0, 1, 2, 2));
        assertEquals(TerrainMap.NO_SURFACE, map.findSurface(1, 0, 0, 0, 2, 2));
        assertEquals(0, map.findSurface(0, 0, 1, 1, 2, 2));
    }
}
