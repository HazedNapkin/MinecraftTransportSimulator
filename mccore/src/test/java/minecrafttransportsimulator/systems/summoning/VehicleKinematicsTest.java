package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class VehicleKinematicsTest {
    private static final double SPEED_FACTOR = 0.35;
    /**30 km/h in m/s.**/
    private static final double CRUISE = 30 / 3.6;

    private static VehicleKinematics car(double steeringForceFactor) {
        return new VehicleKinematics(3, SPEED_FACTOR, steeringForceFactor, false);
    }

    @Test
    public void slowTurnRadiusFollowsWheelbase() {
        //45 degrees of steering over 0.7 * wheelbase gives a radius of 0.7 * wheelbase / 45 degrees in radians.
        double expected = 0.7 * 3 / Math.toRadians(45);
        assertEquals(expected, car(0).getTurnRadius(1), 1e-9);
        assertEquals(expected, car(0).getTurnRadius(0), 1e-9);
    }

    @Test
    public void turnsWidenWithSpeed() {
        VehicleKinematics kinematics = car(0);
        double slow = kinematics.getTurnRadius(1);
        double cruise = kinematics.getTurnRadius(CRUISE);
        double fast = kinematics.getTurnRadius(50 / 3.6);
        assertTrue(cruise > slow * 2);
        assertTrue(fast > cruise * 2);
        //About 7.3 blocks at 30 km/h for a 3 block wheelbase.
        assertEquals(7.3, cruise, 0.2);
    }

    @Test
    public void maxSpeedInvertsMaxCurvature() {
        VehicleKinematics kinematics = car(0.2);
        for (double speed = 5; speed <= 25; speed += 2.5) {
            assertEquals(speed, kinematics.getMaxSpeed(kinematics.getMaxCurvature(speed)), 1e-6);
        }
    }

    @Test
    public void maxSpeedLimits() {
        VehicleKinematics kinematics = car(0);
        double fullLock = kinematics.getMaxCurvature(0);
        assertEquals(Double.POSITIVE_INFINITY, kinematics.getMaxSpeed(0), 0);
        assertEquals(0, kinematics.getMaxSpeed(fullLock * 1.01), 0);
        //Anything up to full lock can be driven at least up to where steering starts to fade.
        double fadeSpeed = 0.35 * 20 * SPEED_FACTOR;
        assertTrue(kinematics.getMaxSpeed(fullLock) >= fadeSpeed - 1e-9);
        assertTrue(kinematics.getMaxSpeed(fullLock * 0.5) > kinematics.getMaxSpeed(fullLock));
        //Either way round.
        assertEquals(kinematics.getMaxSpeed(fullLock * 0.5), kinematics.getMaxSpeed(-fullLock * 0.5), 0);
    }

    @Test
    public void ignoringSpeedScalesByFactor() {
        VehicleKinematics kinematics = new VehicleKinematics(3, SPEED_FACTOR, 0.5, true);
        assertEquals(kinematics.getTurnRadius(1), kinematics.getTurnRadius(40), 1e-9);
        assertEquals(2 * car(0).getTurnRadius(1), kinematics.getTurnRadius(1), 1e-9);
        assertEquals(Double.POSITIVE_INFINITY, kinematics.getMaxSpeed(kinematics.getMaxCurvature(0)), 0);
        assertEquals(0, kinematics.getMaxSpeed(kinematics.getMaxCurvature(0) * 1.01), 0);
        assertFalse(new VehicleKinematics(3, SPEED_FACTOR, 0, true).canSteer());
    }

    @Test
    public void noWheelbaseCantSteer() {
        VehicleKinematics kinematics = new VehicleKinematics(0, SPEED_FACTOR, 0, false);
        assertFalse(kinematics.canSteer());
        assertEquals(Double.POSITIVE_INFINITY, kinematics.getTurnRadius(1), 0);
        assertEquals(0, kinematics.getMaxSpeed(0.1), 0);
        assertEquals(0, kinematics.getRudderInput(0.1, 1), 0);
    }

    @Test
    public void rudderSteersOppositeToCurvature() {
        VehicleKinematics kinematics = car(0);
        double halfLock = kinematics.getMaxCurvature(CRUISE) / 2;
        assertEquals(-22.5, kinematics.getRudderInput(halfLock, CRUISE), 1e-9);
        assertEquals(22.5, kinematics.getRudderInput(-halfLock, CRUISE), 1e-9);
        //Held to full lock.
        assertEquals(-45, kinematics.getRudderInput(halfLock * 10, CRUISE), 1e-9);
    }
}
