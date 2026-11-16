package minecrafttransportsimulator.systems.summoning;

/**
 * Turns what the {@link PathTracker} says to do each tick into what the driver does with the controls: steering,
 * throttle, brake and gears.  It knows nothing of the vehicle but what it's told, so it can be run against a made-up
 * vehicle in tests as well as by the {@link VehicleSummonJob}, which is what does it to a real one.
 * <p>
 * Driving, it keeps to the speed the tracker asks for with the throttle and brake.  Changing between forward and
 * reverse, it stops first, held straight, and only steers again once the vehicle's going the new way.  Turning on the
 * spot, it stops, held straight, then lets go of the brakes in neutral and steers, which is what turns a vehicle that
 * skid steers.
 */
final class DriveControls {
    /**
     * How slow the vehicle must be to shift between forward and reverse, and how slow it must be going the new way
     * before it's steered, in m/s.  MTS only turns a vehicle the way it's going once it's going that way, and a
     * vehicle that can skid steer spins on the spot in neutral, so it's held straight until then.
     */
    static final double SHIFT_SPEED = 0.3;
    static final double STEER_SETTLE_SPEED = 0.5;
    /**
     * How near the end a vehicle that's all but stopped counts as there, in blocks, how slow is all but stopped,
     * in m/s, and how many ticks into the drive that's first looked for.  Braking can bring it to rest just short, and
     * it shouldn't sit there until it's called stuck.
     */
    static final double STALLED_ARRIVE_DISTANCE = 1;
    static final double STALLED_SPEED = 0.3;
    static final int STALLED_AFTER_TICKS = 20;
    /**The most throttle used, the least used when short of the speed, and how much more for each m/s short.**/
    static final double MAX_THROTTLE = 0.8;
    static final double BASE_THROTTLE = 0.25;
    static final double THROTTLE_GAIN = 0.15;
    /**How far over the speed it may be before it brakes, in m/s, and how much brake for each m/s over.**/
    static final double BRAKE_DEADBAND = 1;
    static final double BRAKE_GAIN = 0.25;
    /**The throttle held when it's about at the speed, and the slowest speed asked for that it's held at, in m/s.**/
    static final double HOLD_THROTTLE = 0.15;
    static final double HOLD_MIN_SPEED = 0.5;

    /**A change of gear to make.**/
    enum Shift {
        NONE,
        UP,
        DOWN
    }

    /**What to do with the controls this tick.**/
    static final class Controls {
        /**The rudder input to steer with.**/
        double rudder;
        double throttle;
        double brake;
        /**True if the steering was let go of, so the tracker should carry on from straight ahead rather than where it had it.**/
        boolean straightened;
        /**True if it's turning on the spot, so the parking brake must be off and engines in neutral.**/
        boolean neutral;
    }

    private DriveControls() {
    }

    /**
     * The shift to make to an engine in the gear, negative for reverse and 0 for neutral, to be going forward, or
     * backing up if reverse is set, at the speed in m/s.  From neutral it shifts without stopping, as when engines have
     * just started.  In gear the other way, it only shifts once it's stopped.
     */
    static Shift getShift(double gear, boolean reverse, double speed) {
        if (isInGear(gear, reverse) || (gear != 0 && speed >= SHIFT_SPEED)) {
            return Shift.NONE;
        }
        return reverse ? Shift.DOWN : Shift.UP;
    }

    /**
     * True while an engine in the gear is still changing between forward and reverse, so the vehicle's held still: it's
     * in gear the wrong way, or in neutral on the way to reverse, as it only shifts a gear a tick.
     */
    static boolean isShifting(double gear, boolean reverse) {
        return !isInGear(gear, reverse) && (gear != 0 || reverse);
    }

    private static boolean isInGear(double gear, boolean reverse) {
        return reverse ? gear < 0 : gear > 0;
    }

    /**
     * True if the vehicle has all but stopped just short of the end, ticks into the drive, which counts as arriving.
     */
    static boolean isStalledAtEnd(PathTracker.Command command, double speed, int ticks) {
        return command.remaining < STALLED_ARRIVE_DISTANCE && speed < STALLED_SPEED && ticks > STALLED_AFTER_TICKS;
    }

    /**
     * Sets the controls for driving as the command says, at the speed in m/s.  If any engine's still shifting between
     * forward and reverse, it's held stopped and straight.  It only steers once MTS sees it going the way it's meant
     * to, which goingInReverse says, or it'd turn the wrong way.
     */
    static void drive(PathTracker.Command command, double speed, boolean goingInReverse, boolean shifting, Controls controls) {
        controls.neutral = false;
        if (shifting) {
            controls.straightened = true;
            controls.rudder = 0;
            controls.throttle = 0;
            controls.brake = 1;
            return;
        }
        if (goingInReverse != command.reverse && speed >= STEER_SETTLE_SPEED) {
            controls.straightened = true;
            controls.rudder = 0;
        } else {
            controls.straightened = false;
            controls.rudder = command.rudder;
        }
        setPedals(speed, command.targetSpeed, controls);
    }

    /**
     * Sets the throttle and brake to keep to the target speed, which slows for turns and steps, and brakes to a stop
     * at the end.  Both speeds are in m/s.
     */
    static void setPedals(double speed, double targetSpeed, Controls controls) {
        if (speed < targetSpeed) {
            controls.throttle = Math.max(0, Math.min(MAX_THROTTLE, BASE_THROTTLE + (targetSpeed - speed) * THROTTLE_GAIN));
            controls.brake = 0;
        } else if (speed > targetSpeed + BRAKE_DEADBAND) {
            controls.throttle = 0;
            controls.brake = Math.min(1, (speed - targetSpeed) * BRAKE_GAIN);
        } else {
            controls.throttle = targetSpeed > HOLD_MIN_SPEED ? HOLD_THROTTLE : 0;
            controls.brake = 0;
        }
    }

    /**
     * Sets the controls for turning on the spot as the command says, at the speed in m/s.  MTS only skid steers a
     * vehicle that's all but stopped, with an engine running in neutral and the brakes off, so it stops first, held
     * straight, then goes to neutral and lets go of the brakes, and the steering turns it.
     */
    static void pivot(PathTracker.Command command, double speed, Controls controls) {
        controls.throttle = 0;
        controls.straightened = false;
        if (speed > SHIFT_SPEED) {
            controls.neutral = false;
            controls.rudder = 0;
            controls.brake = 1;
            return;
        }
        controls.neutral = true;
        controls.brake = 0;
        controls.rudder = command.rudder;
    }
}
