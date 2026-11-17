package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import minecrafttransportsimulator.systems.navigation.DriveControls;
import minecrafttransportsimulator.systems.navigation.PathTracker;
import minecrafttransportsimulator.systems.navigation.VehicleKinematics;

/**
 * What the driver does with the controls: see {@link DriveControls}.  These hold for any sensible way of driving, so
 * they should still pass when how it keeps to its speed or changes gear is reworked.  How well it drives is scored by
 * the {@link SummonScorecardTest}.
 *
 * @author HazedNapkin
 */
public class DriveControlsTest {
    private static PathTracker.Command command(double targetSpeed, double rudder, boolean reverse) {
        PathTracker.Command command = new PathTracker.Command();
        command.targetSpeed = targetSpeed;
        command.targetSpeedAhead = targetSpeed;
        command.rudder = rudder;
        command.reverse = reverse;
        command.remaining = 20;
        command.legRemaining = 20;
        return command;
    }

    private static DriveControls newDriver() {
        return new DriveControls(VehicleSummonJob.BRAKING);
    }

    @Test
    public void neverThrottlesAndBrakesAtOnce() {
        for (double target = 0; target <= 10; target += 0.25) {
            for (double speed = 0; speed <= 12; speed += 0.25) {
                for (double left : new double[] { 0.5, 3, 20 }) {
                    PathTracker.Command command = command(target, 0, false);
                    command.legRemaining = command.remaining = left;
                    DriveControls.Controls controls = newDriver().update(command, speed, false, false);
                    assertTrue("throttle out of range", controls.throttle >= 0 && controls.throttle <= 1);
                    assertTrue("brake out of range", controls.brake >= 0 && controls.brake <= 1);
                    assertTrue("both pedals at " + speed + " for " + target, controls.throttle == 0 || controls.brake == 0);
                }
            }
        }
    }

    @Test
    public void slowsMoreTheFasterItsGoing() {
        for (double target = 0.5; target <= 10; target += 0.5) {
            double lastThrottle = 1;
            double lastBrake = 0;
            for (double speed = 0; speed <= 14; speed += 0.1) {
                DriveControls.Controls controls = newDriver().update(command(target, 0, false), speed, false, false);
                assertTrue("throttle rose as it went faster", controls.throttle <= lastThrottle + 1e-9);
                assertTrue("brake fell as it went faster", controls.brake >= lastBrake - 1e-9);
                lastThrottle = controls.throttle;
                lastBrake = controls.brake;
            }
        }
    }

    @Test
    public void givesNoThrottleWhenItShouldBeStopped() {
        for (double speed = 0; speed <= 10; speed += 0.1) {
            DriveControls.Controls controls = newDriver().update(command(0, 0, false), speed, false, false);
            assertEquals(0, controls.throttle, 0);
        }
        DriveControls.Controls held = newDriver().update(command(0, 0, false), 0, false, false);
        assertEquals("holds itself still", 1, held.brake, 0);
    }

    @Test
    public void pullsAwayFromAStandstill() {
        DriveControls.Controls controls = newDriver().update(command(VehicleSummonJob.MIN_SPEED, 0, false), 0, false, false);
        assertTrue(controls.throttle > 0);
        assertEquals(0, controls.brake, 0);
    }

    @Test
    public void givesMoreThrottleTheLongerItsShort() {
        DriveControls driver = newDriver();
        double first = driver.update(command(3, 0, false), 1, false, false).throttle;
        double later = first;
        for (int i = 0; i < 40; ++i) {
            later = driver.update(command(3, 0, false), 1, false, false).throttle;
        }
        assertTrue("a vehicle that won't get going gets more throttle", later > first);
        assertTrue(later <= DriveControls.MAX_THROTTLE);
    }

    @Test
    public void brakesToStopRightAtTheEnd() {
        //Going too fast for the plan's braking, it brakes as hard as it takes to stop at the end, not later.
        PathTracker.Command command = command(Math.sqrt(2 * VehicleSummonJob.BRAKING * 4), 0, false);
        command.remaining = command.legRemaining = 4;
        DriveControls driver = newDriver();
        assertEquals(-6 * 6 / (2 * 4D), driver.getWantedAcceleration(command, 6), 1e-9);
        DriveControls.Controls controls = driver.update(command, 6, false, false);
        assertEquals(0, controls.throttle, 0);
        double brake = (6 * 6 / (2 * 4D) - driver.getCoasting()) / driver.getBraking();
        assertTrue(controls.brake >= brake - 1e-9 && controls.brake < brake + DriveControls.PEDAL_STEP);
        //Well short of stopping, it keeps to the speed asked for.
        assertEquals((command.targetSpeed - 2) / DriveControls.SPEED_RESPONSE_TIME, newDriver().getWantedAcceleration(command, 2), 1e-9);
        //Once it's stopping, it doesn't throttle back up if it's slowed more than it had to: it lets itself roll in.
        DriveControls.Controls slowed = driver.update(command, 3, false, false);
        assertEquals(0, slowed.throttle, 0);
    }

    @Test
    public void learnsHowHardItsBrakesStopIt() {
        //A vehicle with brakes three times as strong as guessed, that coasts down at 0.3 m/s per second.
        double strength = 3 * DriveControls.INITIAL_BRAKING;
        DriveControls driver = newDriver();
        PathTracker.Command command = command(2, 0, false);
        command.remaining = command.legRemaining = 100;
        double speed = 30;
        for (int i = 0; i < 60 && speed > 1; ++i) {
            DriveControls.Controls controls = driver.update(command, speed, false, false);
            speed = Math.max(0, speed - (controls.brake * strength + 0.3) * DriveControls.TICK);
        }
        assertEquals(strength, driver.getBraking(), strength * 0.15);
    }

    @Test
    public void parksOnceFullBrakesWouldStopItAtTheEnd() {
        PathTracker.Command command = command(1, 0, false);
        command.remaining = command.legRemaining = 2;
        assertFalse("still a way to go", newDriver().update(command, 1, false, false).park);
        DriveControls driver = newDriver();
        command.remaining = command.legRemaining = driver.getStoppingDistance(1);
        DriveControls.Controls controls = driver.update(command, 1, false, false);
        assertTrue(controls.park);
        assertEquals("arrived", controls.parkReason);
        command.remaining = command.legRemaining = -0.1;
        assertTrue("past the end", newDriver().update(command, 0.5, false, false).park);
    }

    @Test
    public void neverParksShortOfWhereItChangesDirection() {
        //Stopped right at where it backs up from, with more route after: that's no end.
        PathTracker.Command command = command(0, 0, false);
        command.legRemaining = 0;
        command.remaining = 10;
        DriveControls driver = newDriver();
        for (int i = 0; i < 60; ++i) {
            assertFalse(driver.update(command, 0, false, false).park);
        }
    }

    @Test
    public void countsStoppingJustShortAsArrivingOnceItCantGetGoing() {
        PathTracker.Command command = command(0.5, 0, false);
        command.remaining = command.legRemaining = 0.5;
        DriveControls driver = newDriver();
        int ticks = 0;
        while (!driver.update(command, 0, false, false).park) {
            assertTrue("never gave up", ++ticks < 100);
        }
        assertTrue("gave up too soon", ticks >= DriveControls.STALLED_TICKS);
        command.remaining = command.legRemaining = 5;
        DriveControls far = newDriver();
        for (int i = 0; i < 100; ++i) {
            assertFalse("nowhere near", far.update(command, 0, false, false).park);
        }
    }

    @Test
    public void shiftsIntoGearWithoutStoppingFromNeutral() {
        assertEquals(DriveControls.Shift.UP, DriveControls.getShift(0, false, 5));
        assertEquals(DriveControls.Shift.DOWN, DriveControls.getShift(0, true, 5));
        assertFalse("going forward from neutral holds nothing up", DriveControls.isShifting(0, false));
        assertTrue("on the way to reverse it's held", DriveControls.isShifting(0, true));
    }

    @Test
    public void stopsBeforeShiftingBetweenForwardAndReverse() {
        assertEquals(DriveControls.Shift.NONE, DriveControls.getShift(1, true, 2));
        assertEquals(DriveControls.Shift.NONE, DriveControls.getShift(-1, false, 2));
        assertEquals(DriveControls.Shift.DOWN, DriveControls.getShift(1, true, 0));
        assertEquals(DriveControls.Shift.UP, DriveControls.getShift(-1, false, 0));
        assertTrue(DriveControls.isShifting(1, true));
        assertTrue(DriveControls.isShifting(-1, false));
    }

    @Test
    public void leavesTheRightGearAlone() {
        for (double speed : new double[] { 0, 1, 8 }) {
            assertEquals(DriveControls.Shift.NONE, DriveControls.getShift(1, false, speed));
            assertEquals(DriveControls.Shift.NONE, DriveControls.getShift(3, false, speed));
            assertEquals(DriveControls.Shift.NONE, DriveControls.getShift(-1, true, speed));
        }
        assertFalse(DriveControls.isShifting(2, false));
        assertFalse(DriveControls.isShifting(-1, true));
    }

    @Test
    public void holdsStillAndStraightWhileShifting() {
        DriveControls.Controls controls = newDriver().update(command(3, 20, true), 2, false, true);
        assertEquals(0, controls.throttle, 0);
        assertEquals(1, controls.brake, 0);
        assertEquals(0, controls.rudder, 0);
        assertTrue(controls.straightened);
        assertFalse(controls.neutral);
        assertFalse(controls.park);
    }

    @Test
    public void onlySteersOnceItsGoingTheWayItsMeantTo() {
        //Meant to be backing up, but still rolling forward: steering now would turn it the wrong way.
        DriveControls.Controls controls = newDriver().update(command(2, 20, true), 2, false, false);
        assertEquals(0, controls.rudder, 0);
        assertTrue(controls.straightened);
        //All but stopped, it may steer, so it's ready as it sets off.
        controls = newDriver().update(command(2, 20, true), 0.1, false, false);
        assertEquals(20, controls.rudder, 0);
        assertFalse(controls.straightened);
        //Going the way it's meant to.
        assertEquals(20, newDriver().update(command(2, 20, true), 2, true, false).rudder, 0);
        assertEquals(-15, newDriver().update(command(5, -15, false), 5, false, false).rudder, 0);
    }

    @Test
    public void stopsBeforeTurningOnTheSpot() {
        PathTracker.Command command = command(0, 30, false);
        command.pivot = true;
        DriveControls.Controls controls = newDriver().update(command, 3, false, false);
        assertEquals(0, controls.throttle, 0);
        assertEquals(1, controls.brake, 0);
        assertEquals(0, controls.rudder, 0);
        assertFalse("still in gear while it stops", controls.neutral);
        controls = newDriver().update(command, 0, false, false);
        assertEquals(0, controls.throttle, 0);
        assertEquals("brakes off, or MTS won't turn it", 0, controls.brake, 0);
        assertEquals(30, controls.rudder, 0);
        assertTrue(controls.neutral);
    }

    @Test
    public void simulatedVehicleDoesWhatItsControlsSay() {
        //The simulator the scorecard relies on: it speeds up, coasts down, brakes harder than it coasts, and holds on a slope when parked.
        VehicleKinematics kinematics = SummonScenarios.vehicle("car").kinematics;
        SimVehicle vehicle = new SimVehicle(new SimVehicle.Spec(), kinematics, 0, 0, 0, 0);
        vehicle.step(0, 1, 0);
        assertEquals("in neutral the engine does nothing", 0, vehicle.velocity, 0);
        vehicle.shift(DriveControls.Shift.UP);
        for (int i = 0; i < 40; ++i) {
            vehicle.step(0, 1, 0);
        }
        double driven = vehicle.velocity;
        assertTrue(driven > 3 && vehicle.z > 0 && Math.abs(vehicle.x) < 1e-9);
        for (int i = 0; i < 12; ++i) {
            vehicle.step(0, 0, 0);
        }
        double coasted = driven - vehicle.velocity;
        double before = vehicle.velocity;
        for (int i = 0; i < 10; ++i) {
            vehicle.step(0, 0, 1);
        }
        assertTrue("brakes slow it more than coasting", before - vehicle.velocity > coasted);

        SimVehicle parked = new SimVehicle(new SimVehicle.Spec().grade(2), kinematics, 0, 0, 0, 0);
        parked.parkingBrake = true;
        for (int i = 0; i < 20; ++i) {
            parked.step(0, 0, 0);
        }
        assertEquals("parked on a slope it stays put", 0, parked.z, 0);

        //Steering left turns its yaw up going forward, and down backing up once it counts as going in reverse.
        SimVehicle turning = new SimVehicle(new SimVehicle.Spec(), kinematics, 0, 0, 0, 2);
        turning.gear = 1;
        turning.step(-20, 0.3, 0);
        assertTrue(turning.yaw > 0);
        SimVehicle backing = new SimVehicle(new SimVehicle.Spec(), kinematics, 0, 0, 0, 0);
        backing.gear = -1;
        for (int i = 0; i < 20; ++i) {
            backing.step(0, 0.5, 0);
        }
        assertTrue(backing.goingInReverse && backing.z < 0);
        backing.step(-20, 0.5, 0);
        assertTrue(backing.yaw < 0);
    }
}
