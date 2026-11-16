package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * What the driver does with the controls: see {@link DriveControls}.  These hold for any sensible way of driving, so
 * they should still pass when how it keeps to its speed or changes gear is reworked.  How well it drives is scored by
 * the {@link SummonScorecardTest}.
 */
public class DriveControlsTest {
    private static PathTracker.Command command(double targetSpeed, double rudder, boolean reverse) {
        PathTracker.Command command = new PathTracker.Command();
        command.targetSpeed = targetSpeed;
        command.rudder = rudder;
        command.reverse = reverse;
        command.remaining = 20;
        command.legRemaining = 20;
        return command;
    }

    @Test
    public void neverThrottlesAndBrakesAtOnce() {
        DriveControls.Controls controls = new DriveControls.Controls();
        for (double target = 0; target <= 10; target += 0.25) {
            for (double speed = 0; speed <= 12; speed += 0.25) {
                DriveControls.setPedals(speed, target, controls);
                assertTrue("throttle out of range", controls.throttle >= 0 && controls.throttle <= 1);
                assertTrue("brake out of range", controls.brake >= 0 && controls.brake <= 1);
                assertTrue("both pedals at " + speed + " for " + target, controls.throttle == 0 || controls.brake == 0);
            }
        }
    }

    @Test
    public void pedalsFollowHowFarOffTheSpeedItIs() {
        DriveControls.Controls controls = new DriveControls.Controls();
        for (double target = 0; target <= 10; target += 0.5) {
            //The faster it's going for the same target, the less throttle and the more brake.
            double lastThrottle = 1;
            double lastBrake = 0;
            for (double speed = 0; speed <= 14; speed += 0.1) {
                DriveControls.setPedals(speed, target, controls);
                assertTrue("throttle rose as it went faster", controls.throttle <= lastThrottle + 1e-9);
                assertTrue("brake fell as it went faster", controls.brake >= lastBrake - 1e-9);
                lastThrottle = controls.throttle;
                lastBrake = controls.brake;
            }
        }
    }

    @Test
    public void givesNoThrottleWhenItShouldBeStopped() {
        DriveControls.Controls controls = new DriveControls.Controls();
        for (double speed = 0; speed <= 10; speed += 0.1) {
            DriveControls.setPedals(speed, 0, controls);
            assertEquals(0, controls.throttle, 0);
        }
    }

    @Test
    public void pullsAwayFromAStandstill() {
        DriveControls.Controls controls = new DriveControls.Controls();
        DriveControls.setPedals(0, VehicleSummonJob.MIN_SPEED, controls);
        assertTrue(controls.throttle > 0);
        assertEquals(0, controls.brake, 0);
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
        DriveControls.Controls controls = new DriveControls.Controls();
        DriveControls.drive(command(3, 20, true), 2, false, true, controls);
        assertEquals(0, controls.throttle, 0);
        assertEquals(1, controls.brake, 0);
        assertEquals(0, controls.rudder, 0);
        assertTrue(controls.straightened);
        assertFalse(controls.neutral);
    }

    @Test
    public void onlySteersOnceItsGoingTheWayItsMeantTo() {
        DriveControls.Controls controls = new DriveControls.Controls();
        //Meant to be backing up, but still rolling forward: steering now would turn it the wrong way.
        DriveControls.drive(command(2, 20, true), 2, false, false, controls);
        assertEquals(0, controls.rudder, 0);
        assertTrue(controls.straightened);
        //All but stopped, it may steer, so it's ready as it sets off.
        DriveControls.drive(command(2, 20, true), 0.1, false, false, controls);
        assertEquals(20, controls.rudder, 0);
        assertFalse(controls.straightened);
        //Going the way it's meant to.
        DriveControls.drive(command(2, 20, true), 2, true, false, controls);
        assertEquals(20, controls.rudder, 0);
        DriveControls.drive(command(5, -15, false), 5, false, false, controls);
        assertEquals(-15, controls.rudder, 0);
    }

    @Test
    public void stopsBeforeTurningOnTheSpot() {
        DriveControls.Controls controls = new DriveControls.Controls();
        PathTracker.Command command = command(0, 30, false);
        command.pivot = true;
        DriveControls.pivot(command, 3, controls);
        assertEquals(0, controls.throttle, 0);
        assertEquals(1, controls.brake, 0);
        assertEquals(0, controls.rudder, 0);
        assertFalse("still in gear while it stops", controls.neutral);
        DriveControls.pivot(command, 0, controls);
        assertEquals(0, controls.throttle, 0);
        assertEquals("brakes off, or MTS won't turn it", 0, controls.brake, 0);
        assertEquals(30, controls.rudder, 0);
        assertTrue(controls.neutral);
    }

    @Test
    public void countsStoppingJustShortAsArriving() {
        PathTracker.Command command = command(0, 0, false);
        command.remaining = 0.5;
        assertTrue(DriveControls.isStalledAtEnd(command, 0, 200));
        assertFalse("still moving", DriveControls.isStalledAtEnd(command, 1, 200));
        assertFalse("only just set off", DriveControls.isStalledAtEnd(command, 0, 5));
        command.remaining = 5;
        assertFalse("nowhere near", DriveControls.isStalledAtEnd(command, 0, 200));
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
        vehicle.step(0, 0, 0);
        vehicle.step(0, 0, 0);
        for (int i = 0; i < 10; ++i) {
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
