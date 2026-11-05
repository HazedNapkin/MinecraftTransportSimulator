package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FlowFieldTest {
    private static FlowField.Settings settings(double clearance) {
        FlowField.Settings settings = new FlowField.Settings();
        settings.clearance = clearance;
        settings.radius = 30;
        return settings;
    }

    private static FlowField flood(FakeTerrain terrain, FlowField.Settings settings, int sourceX, int sourceY, int sourceZ) {
        FlowField field = new FlowField(new TerrainMap(terrain), settings, sourceX, sourceZ);
        assertTrue(field.addSource(sourceX, sourceY, sourceZ, 0));
        field.finish();
        return field;
    }

    @Test
    public void flatDistancesAreOctile() {
        FlowField field = flood(FakeTerrain.flat(20), settings(0), 0, 0, 0);
        assertEquals(0, field.getDistance(0, 0, 0, 0), 1e-9);
        assertEquals(10, field.getDistance(10, 0, 0, 0), 1e-9);
        assertEquals(10 * Math.sqrt(2), field.getDistance(10, 0, 10, 0), 1e-9);
        assertEquals(5 + 5 * Math.sqrt(2), field.getDistance(10, 0, 5, 0), 1e-9);
    }

    @Test
    public void goesRoundWalls() {
        //A wall along x = 5 from z = -10 to 10, so getting past it means going round an end.
        FakeTerrain terrain = FakeTerrain.flat(20).fill(5, 0, -10, 5, 2, 10);
        FlowField field = flood(terrain, settings(0), 0, 0, 0);
        assertTrue(field.getDistance(10, 0, 0, 0) > 20);
        //Following the next cells from behind the wall gets back to the source.
        FlowField.Cell cell = field.getCell(10, 0, 0);
        int steps = 0;
        while (cell.next != null && steps++ < 1000) {
            cell = cell.next;
        }
        assertEquals(0, cell.x);
        assertEquals(0, cell.z);
    }

    @Test
    public void wideVehiclesNeedWideGaps() {
        //A wall along x = 5 with a 1 block gap at z = 0, and no way round within range.
        FakeTerrain terrain = FakeTerrain.flat(20).fill(5, 0, -20, 5, 2, -1).fill(5, 0, 1, 5, 2, 20);
        FlowField narrow = flood(terrain, settings(0.5), 0, 0, 0);
        FlowField wide = flood(terrain, settings(1), 0, 0, 0);
        assertEquals(10, narrow.getDistance(10, 0, 0, 0), 1e-9);
        assertEquals(Double.POSITIVE_INFINITY, wide.getDistance(10, 0, 0, 0), 0);
    }

    @Test
    public void climbsHillsOneStepAtATime() {
        //A hill at x >= 5 stepping up a block every 2 blocks, 3 steps high, with the goal on top.
        FakeTerrain terrain = FakeTerrain.flat(20);
        for (int step = 0; step < 3; ++step) {
            terrain.fill(5 + 2 * step, 0, -20, 20, step, 20);
        }
        FlowField field = flood(terrain, settings(0), 12, 3, 0);
        //The bottom of the hill can drive up to the top.
        FlowField.Cell bottom = field.getCell(0, 0, 0);
        assertNotNull(bottom);
        assertEquals(12, bottom.distance, 1e-9);
        assertEquals(12, field.getDistance(0, 1, 0, 1), 1e-9);
    }

    @Test
    public void cliffsAreOneWay() {
        //A 2 block cliff at x >= 5.  Vehicles can drop 2 but only climb 1.
        FakeTerrain terrain = FakeTerrain.flat(20).fill(5, 0, -20, 20, 1, 20);
        FlowField.Settings toward = settings(0);
        //Driving to a goal on top of the cliff, nothing below can get there.
        FlowField up = flood(terrain, toward, 10, 2, 0);
        assertNull(up.getCell(0, 0, 0));
        //Driving to a goal below it, the top can drive off the edge.
        FlowField down = flood(terrain, toward, 0, 0, 0);
        assertNotNull(down.getCell(10, 2, 0));
    }

    @Test
    public void clearanceMeasuresToNearestBlockedColumn() {
        FakeTerrain terrain = FakeTerrain.flat(20).fill(3, 0, 0, 3, 2, 0);
        FlowField field = new FlowField(new TerrainMap(terrain), settings(0), 0, 0);
        assertEquals(2.5, field.getClearance(0, 0, 0), 1e-9);
        assertEquals(FlowField.CLEARANCE_RANGE, field.getClearance(-10, 0, 0), 1e-9);
    }
}
