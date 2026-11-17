package minecrafttransportsimulator.systems.navigation;

/**
 * Turns what the {@link PathTracker} says to do each tick into what the driver does with the controls: steering,
 * throttle, brake and gears, and when to put the parking brake on.  It knows nothing of the vehicle but what it's
 * told, so it can be run against a made-up vehicle in tests as well as by whatever drives a real
 * one, like a vehicle summon.  Make one for each drive: it learns how the vehicle it's driving responds.
 * <p>
 * It works out how hard it wants to speed up or slow down: towards the speed the tracker asks for, or, once it's on
 * the way to stopping where it changes direction or pulls up, exactly hard enough to stop there.  Wanting to slow less
 * than the vehicle does on its own, it uses the throttle, set by how far off the speed it is and how far off it's been,
 * so it finds the throttle a heavy vehicle or a hill needs.  Wanting to slow more, it brakes, as hard as it's learnt the
 * vehicle's brakes need to be pressed for that.  Vehicles' brakes differ a lot, so it learns how hard they stop it
 * from how fast it slows each time it brakes, and how fast it slows on its own each time it lets go.
 * <p>
 * It parks once pressing the brakes fully would stop it right where it pulls up, so it stops there rather than short
 * of it or after rolling past.  Changing between forward and reverse, it stops first, held straight, and only steers
 * again once the vehicle's going the new way.  Turning on the spot, it stops, held straight, then lets go of the
 * brakes in neutral and steers, which is what turns a vehicle that skid steers.
 *
 * @author HazedNapkin
 */
public final class DriveControls {
    /**The length of a tick, in seconds.**/
    public static final double TICK = 0.05;
    /**
     * How slow the vehicle must be to shift between forward and reverse, and how slow it must be going the new way
     * before it's steered, in m/s.  MTS only turns a vehicle the way it's going once it's going that way, and a
     * vehicle that can skid steer spins on the spot in neutral, so it's held straight until then.
     */
    public static final double SHIFT_SPEED = 0.3;
    public static final double STEER_SETTLE_SPEED = 0.5;
    /**
     * How near the end a vehicle that's all but stopped counts as there, in blocks, how slow is all but stopped, in
     * m/s, and how many ticks in a row it must be like that.  If it's come to rest just short and can't get going
     * again, it shouldn't sit there until it's called stuck.
     */
    public static final double STALLED_ARRIVE_DISTANCE = 1;
    public static final double STALLED_SPEED = 0.3;
    public static final int STALLED_TICKS = 20;
    /**How long it takes to make up a difference from the speed it should be doing, in seconds.**/
    public static final double SPEED_RESPONSE_TIME = 0.5;
    /**
     * How near to slowing as hard as the drive plans to brake it must be, as a fraction, before it brakes to stop
     * exactly where it stops, rather than just keeping to the speed the tracker asks for.
     */
    public static final double STOPPING_FRACTION = 0.97;
    /**The most throttle used, and what it sets off with before it's learnt what the vehicle needs.**/
    public static final double MAX_THROTTLE = 0.8;
    public static final double INITIAL_THROTTLE = 0.3;
    /**How much throttle for each m/s short of the speed, and how much more for each m/s it's been short each second.**/
    public static final double THROTTLE_GAIN = 0.2;
    public static final double THROTTLE_INTEGRAL_GAIN = 0.3;
    /**
     * What it guesses the vehicle slows by at full brake, and on its own with neither pedal down, in m/s per second,
     * before it's learnt them, and the most and least they may be learnt as.
     */
    public static final double INITIAL_BRAKING = 5;
    public static final double MIN_BRAKING = 0.5;
    public static final double MAX_BRAKING = 40;
    public static final double INITIAL_COASTING = 0.3;
    public static final double MAX_COASTING = 5;
    /**
     * How much of each tick's measurement of coasting goes into what it's learnt, and how much of what it's measured
     * of braking is kept each tick, as it's weighed by how hard it braked: braking lightly says little of how hard
     * braking fully stops it.  What it guesses at first counts as this many ticks at full brake.
     */
    public static final double LEARNING_RATE = 0.2;
    public static final double BRAKING_MEMORY = 0.97;
    public static final double BRAKING_PRIOR = 0.5;
    /**
     * How many ticks after letting go of the throttle before it measures, as the engine takes a moment to stop
     * pushing, how fast it must be going to measure, in m/s, and the least brake it measures from.
     */
    public static final int SETTLE_TICKS = 10;
    public static final double LEARN_SPEED = 0.5;
    public static final double LEARN_BRAKE = 0.05;
    /**
     * How much of the braking it's learnt to count on: when deciding when to park, so it doesn't park too late, and
     * when deciding when to start stopping, so a vehicle with weaker brakes than planned starts earlier.
     */
    public static final double PARK_BRAKING_FRACTION = 1;
    public static final double STOPPING_BRAKING_FRACTION = 0.8;
    /**
     * How far short of the end it may park, in blocks, as it's never exactly there on a tick, and the furthest short of
     * it it ever parks, however fast it's going, so a wrong idea of its brakes can't have it park far off.
     */
    public static final double PARK_MARGIN = 0.05;
    public static final double MAX_PARK_DISTANCE = 1;
    /**
     * How much harder than it slows on its own it must want to slow before it brakes, in m/s per second, so it doesn't
     * flick between the pedals.
     */
    public static final double BRAKE_THRESHOLD = 1.0;
    /**The steps the pedals are set in, so they're not changed, and sent to clients, for every tiny difference.**/
    public static final double PEDAL_STEP = 0.05;
    /**The steps the steering is set in, in degrees of rudder input, for the same reason.  It makes no difference to where it goes that matters.**/
    public static final double RUDDER_STEP = 0.5;
    /**Slower than this, in m/s, with nowhere to go, it holds itself still with the brake.**/
    public static final double HOLD_SPEED = 0.3;

    /**A change of gear to make.**/
    public enum Shift {
        NONE,
        UP,
        DOWN
    }

    /**What to do with the controls this tick.**/
    public static final class Controls {
        /**The rudder input to steer with.**/
        public double rudder;
        public double throttle;
        public double brake;
        /**True if the steering was let go of, so the tracker should carry on from straight ahead rather than where it had it.**/
        public boolean straightened;
        /**True if it's turning on the spot, so the parking brake must be off and engines in neutral.**/
        public boolean neutral;
        /**True once it's there: put the parking brake on and finish.  Why, for debugging.**/
        public boolean park;
        public String parkReason;
    }

    private final double plannedBraking;
    private final Controls controls = new Controls();
    /**What it's learnt the vehicle slows by at full brake, and with neither pedal down, in m/s per second.**/
    private double braking = INITIAL_BRAKING;
    /**What it's measured of braking, each weighed by how hard it braked, and the weights, for finding braking from.**/
    private double brakingSum = INITIAL_BRAKING * BRAKING_PRIOR;
    private double brakingWeight = BRAKING_PRIOR;
    private double coasting = INITIAL_COASTING;
    /**The throttle it's found it needs to keep to the speed, without the part for how far off it is right now.**/
    private double throttleTrim = INITIAL_THROTTLE;
    /**What it did last tick, and how fast it was going then, to learn from.**/
    private double lastSpeed = -1;
    private double lastThrottle;
    private double lastBrake;
    private int ticksOffThrottle = SETTLE_TICKS;
    private int stalledTicks;
    /**
     * True once it's on its way to stopping where the way it's going ends, until it goes on to the next way.  It
     * stays stopping even if it's slowed more than it needed, rather than speeding up again for a moment, unless it's
     * all but stopped short.  How far it had left to go last tick says when it's gone on to the next way.
     */
    private boolean stopping;
    private double lastLegRemaining = Double.MAX_VALUE;

    /**Makes the controls for one drive, planned to brake no harder than plannedBraking, in m/s per second.**/
    public DriveControls(double plannedBraking) {
        this.plannedBraking = plannedBraking;
    }

    /**What it's learnt the vehicle slows by at full brake, in m/s per second.**/
    public double getBraking() {
        return braking;
    }

    /**What it's learnt the vehicle slows by on its own, with neither pedal down, in m/s per second.**/
    public double getCoasting() {
        return coasting;
    }

    /**
     * Works out the controls for the command at the speed in m/s.  goingInReverse is whether MTS counts it as backing
     * up, and shifting whether any engine's still changing between forward and reverse: see {@link #isShifting}.
     */
    public Controls update(PathTracker.Command command, double speed, boolean goingInReverse, boolean shifting) {
        learn(speed);
        controls.park = false;
        controls.parkReason = null;
        if (command.pivot) {
            pivot(command, speed);
        } else if (shouldPark(command, speed)) {
            controls.park = true;
            controls.straightened = false;
            controls.neutral = false;
            controls.rudder = 0;
            controls.throttle = 0;
            controls.brake = 1;
        } else if (shifting) {
            controls.neutral = false;
            controls.straightened = true;
            controls.rudder = 0;
            controls.throttle = 0;
            controls.brake = 1;
        } else {
            controls.neutral = false;
            //Only steer once MTS sees it going the way it's meant to, or it'd turn the wrong way.
            if (goingInReverse != command.reverse && speed >= STEER_SETTLE_SPEED) {
                controls.straightened = true;
                controls.rudder = 0;
            } else {
                controls.straightened = false;
                controls.rudder = Math.rint(command.rudder / RUDDER_STEP) * RUDDER_STEP;
            }
            setPedals(command, speed);
        }
        lastSpeed = speed;
        lastThrottle = controls.throttle;
        lastBrake = controls.brake;
        ticksOffThrottle = controls.throttle > 0 ? 0 : ticksOffThrottle + 1;
        return controls;
    }

    /**
     * Learns how hard the vehicle's brakes stop it, and how fast it slows on its own, from how its speed changed since
     * last tick with what was done then.  Only while it's going steadily one way, and once the engine's stopped pushing.
     */
    private void learn(double speed) {
        if (lastSpeed < LEARN_SPEED || speed < STALLED_SPEED || ticksOffThrottle < SETTLE_TICKS || lastThrottle > 0) {
            return;
        }
        double slowing = (lastSpeed - speed) / TICK;
        if (lastBrake >= LEARN_BRAKE) {
            //The braking that best fits slowing = coasting + braking * brake over what it's measured, recent first.
            brakingSum = brakingSum * BRAKING_MEMORY + lastBrake * (slowing - coasting);
            brakingWeight = brakingWeight * BRAKING_MEMORY + lastBrake * lastBrake;
            braking = Math.max(MIN_BRAKING, Math.min(MAX_BRAKING, brakingSum / brakingWeight));
        } else if (lastBrake == 0) {
            coasting = Math.max(-MAX_COASTING, Math.min(MAX_COASTING, coasting + LEARNING_RATE * (slowing - coasting)));
        }
    }

    /**How far the vehicle would go, in blocks, before full brakes stopped it from the speed in m/s.**/
    public double getStoppingDistance(double speed) {
        double stopping = Math.max(MIN_BRAKING, (braking + coasting) * PARK_BRAKING_FRACTION);
        return speed * speed / (2 * stopping);
    }

    /**
     * True once it's there: on the last way it goes, full brakes would stop it at the end, or it's past it, or it's
     * been all but stopped just short of it for a while.
     */
    private boolean shouldPark(PathTracker.Command command, double speed) {
        boolean lastLeg = command.remaining <= command.legRemaining + 1e-6;
        if (lastLeg && command.remaining <= Math.min(MAX_PARK_DISTANCE, getStoppingDistance(speed) + PARK_MARGIN)) {
            controls.parkReason = "arrived";
            return true;
        }
        stalledTicks = command.remaining < STALLED_ARRIVE_DISTANCE && speed < STALLED_SPEED ? stalledTicks + 1 : 0;
        if (lastLeg && stalledTicks > STALLED_TICKS) {
            controls.parkReason = "stalled short";
            return true;
        }
        return false;
    }

    /**
     * How hard it wants to speed up, in m/s per second, negative to slow down: towards the speed asked for, or once
     * it's on its way to stopping, just hard enough to stop where this way it's going ends.  It's on its way to
     * stopping once slowing as hard as planned from here, or as hard as its brakes can if they're weaker, would only
     * just stop it there.
     */
    public double getWantedAcceleration(PathTracker.Command command, double speed) {
        double left = command.legRemaining;
        if (left <= 0) {
            //Past where it stops.
            return -MAX_BRAKING;
        }
        if (!isStopping(command, speed)) {
            //Towards the speed it should be doing a moment ahead, so it's slowed for a turn by the time it's there.
            return (command.targetSpeedAhead - speed) / SPEED_RESPONSE_TIME;
        }
        //Just hard enough to stop there, but less if it's slowed more than braking as hard as it should from here
        //would have, so it gets back to that rather than creeping in.  Never harder than keeping to the speed it
        //should be doing now: stopping looks ahead for itself.
        double toSpeed = (command.targetSpeed - speed) / SPEED_RESPONSE_TIME;
        double shortfall = Math.max(0, STOPPING_FRACTION * getStoppingSpeed(left) - speed);
        return Math.min(toSpeed, -speed * speed / (2 * left) + shortfall / SPEED_RESPONSE_TIME);
    }

    /**The speed from which slowing as hard as planned, or as hard as its brakes can if they're weaker, stops it in the distance, in m/s.**/
    private double getStoppingSpeed(double distance) {
        return Math.sqrt(2 * Math.min(plannedBraking, STOPPING_BRAKING_FRACTION * (braking + coasting)) * distance);
    }

    /**True if it's on its way to stopping where the way it's going ends: see {@link #getWantedAcceleration}.**/
    private boolean isStopping(PathTracker.Command command, double speed) {
        if (speed < STALLED_SPEED) {
            //All but stopped: if it's short, it has to get going again.
            return false;
        }
        return stopping || speed >= STOPPING_FRACTION * getStoppingSpeed(command.legRemaining);
    }

    /**Sets the throttle and brake to speed up or slow down as it wants to.**/
    private void setPedals(PathTracker.Command command, double speed) {
        if (command.legRemaining > lastLegRemaining + MAX_PARK_DISTANCE) {
            //Gone on to the next way.
            stopping = false;
        }
        lastLegRemaining = command.legRemaining;
        stopping = isStopping(command, speed);
        double wanted = getWantedAcceleration(command, speed);
        if (command.targetSpeed <= 0 && speed < HOLD_SPEED) {
            //Nowhere to go: hold still.
            controls.throttle = 0;
            controls.brake = 1;
        } else if (wanted > -coasting && !stopping) {
            //Slowing on its own would slow it too much, if at all: the throttle, set by how far off the speed it is.
            double error = command.targetSpeedAhead - speed;
            throttleTrim = Math.max(0, Math.min(MAX_THROTTLE, throttleTrim + THROTTLE_INTEGRAL_GAIN * error * TICK));
            controls.throttle = command.targetSpeed > 0 ? step(Math.min(MAX_THROTTLE, throttleTrim + THROTTLE_GAIN * error)) : 0;
            controls.brake = 0;
        } else if (wanted > -coasting - BRAKE_THRESHOLD) {
            //About what it slows by on its own: let it.
            controls.throttle = 0;
            controls.brake = 0;
        } else {
            //Brake as much more than it slows on its own as it wants to.
            controls.throttle = 0;
            controls.brake = Math.min(1, step((-wanted - coasting) / braking));
        }
    }

    /**The pedal rounded up to a {@link #PEDAL_STEP}, as any pressing at all should press it.**/
    private static double step(double pedal) {
        return pedal <= 0 ? 0 : Math.ceil(pedal / PEDAL_STEP - 1e-9) * PEDAL_STEP;
    }

    /**
     * Sets the controls for turning on the spot as the command says, at the speed in m/s.  MTS only skid steers a
     * vehicle that's all but stopped, with an engine running in neutral and the brakes off, so it stops first, held
     * straight, then goes to neutral and lets go of the brakes, and the steering turns it.
     */
    private void pivot(PathTracker.Command command, double speed) {
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

    /**
     * The steering to set off along the route with at the speed in m/s, already turned for the curve it starts on.  A
     * vehicle placed mid-turn would otherwise run wide while the wheel turned.  None if it starts by turning on the spot.
     */
    public static double getStartRudder(Route route, VehicleKinematics kinematics, double speed) {
        if (route.poses.size() < 2 || route.poses.get(1).pivot) {
            return 0;
        }
        return kinematics.getRudderInput(route.getStart().curvature, Math.max(speed, 1));
    }

    /**
     * The shift to make to an engine in the gear, negative for reverse and 0 for neutral, to be going forward, or
     * backing up if reverse is set, at the speed in m/s.  From neutral it shifts without stopping, as when engines have
     * just started.  In gear the other way, it only shifts once it's stopped.
     */
    public static Shift getShift(double gear, boolean reverse, double speed) {
        if (isInGear(gear, reverse) || (gear != 0 && speed >= SHIFT_SPEED)) {
            return Shift.NONE;
        }
        return reverse ? Shift.DOWN : Shift.UP;
    }

    /**
     * True while an engine in the gear is still changing between forward and reverse, so the vehicle's held still: it's
     * in gear the wrong way, or in neutral on the way to reverse, as it only shifts a gear a tick.
     */
    public static boolean isShifting(double gear, boolean reverse) {
        return !isInGear(gear, reverse) && (gear != 0 || reverse);
    }

    private static boolean isInGear(double gear, boolean reverse) {
        return reverse ? gear < 0 : gear > 0;
    }
}
