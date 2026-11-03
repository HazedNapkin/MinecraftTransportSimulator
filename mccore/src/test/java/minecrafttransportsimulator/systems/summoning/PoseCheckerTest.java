package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PoseCheckerTest {
    /**Height the aircraft comes down from, with its origin this high above the ground at y = 0.**/
    private static final double FROM_Y = 20;

    private static PoseChecker checker(FakeTerrain terrain) {
        return new PoseChecker(new TerrainMap(terrain), ArrivalPlannerTest.car());
    }

    @Test
    public void landsAsItWouldLikeOnOpenGround() {
        double[] yaw = checker(FakeTerrain.flat(30)).findLandingYaw(0.5, 0, 0.5, 30, 0, FROM_Y);
        assertEquals(30, yaw[0], 1e-9);
        assertEquals(1, yaw[1], 0);
    }

    @Test
    public void turnsToFitBetweenThings() {
        //Pillars either side of the spot along x, so the 4 long car only fits lengthways along z.
        FakeTerrain terrain = FakeTerrain.flat(30).fill(-2, 0, 0, -2, 8, 0).fill(3, 0, 0, 3, 8, 0);
        PoseChecker checker = checker(terrain);
        assertFalse(checker.canLand(0.5, 0, 0.5, 90, FROM_Y));
        assertTrue(checker.canLand(0.5, 0, 0.5, 0, FROM_Y));
        //It'd like to face +X, but can only land facing along z.  Turning there from +X on the way down would hit
        //the pillars, so it has to finish turning first.
        double[] yaw = checker.findLandingYaw(0.5, 0, 0.5, 90, 90, FROM_Y);
        assertEquals(0, Math.abs(VehicleSummonFlight.clampAngle(yaw[0])) % 180, 1e-9);
        assertEquals(0, yaw[1], 0);
        //Already facing along z, it can come straight down.
        assertEquals(1, checker.findLandingYaw(0.5, 0, 0.5, 0, 0, FROM_Y)[1], 0);
    }

    @Test
    public void wontLandHalfOffALedge() {
        //Ground only for z < 1, so a car facing +Z centered at z = 0.5 has its front wheels over a drop.
        FakeTerrain terrain = new FakeTerrain().fill(-30, -1, -30, 30, -1, 0);
        PoseChecker checker = checker(terrain);
        assertFalse(checker.canLand(0.5, 0, 0.5, 0, FROM_Y));
        //Whichever way it faces, some wheel is over the drop.
        assertNull(checker.findLandingYaw(0.5, 0, 0.5, 0, 0, FROM_Y));
    }

    @Test
    public void wontLandUnderARoof() {
        FakeTerrain terrain = FakeTerrain.flat(30).fill(-5, 6, -5, 5, 6, 5);
        assertNull(checker(terrain).findLandingYaw(0.5, 0, 0.5, 0, 0, FROM_Y));
    }
}
