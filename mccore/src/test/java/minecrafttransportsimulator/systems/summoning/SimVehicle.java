package minecrafttransportsimulator.systems.summoning;

/**
 * A made-up ground vehicle for testing how well the summon drive controls a real one.  Unlike {@link DriveSimulator},
 * which goes at whatever speed it's told, this one has to be driven with the throttle, brake and gears, and doesn't do
 * as it's told at once: its engine takes a moment to respond and only pushes so hard, its brakes only stop it so fast,
 * it coasts, it may creep in gear, and it may be on a slope.  How it steers follows MTS: by its rudder input and speed,
 * the other way backing up, but only once MTS counts it as going in reverse, and on the spot if it skid steers and is
 * stopped in neutral with the brakes off.
 * <p>
 * The numbers in a {@link Spec} are stand-ins, not taken from MTS's physics.  What matters is that the controls cope
 * with the whole spread of them.  To tie one to a real vehicle, drive it in game with summon debugging on and set the
 * spec to match what {@link SummonDriveReport} says of it.
 */
final class SimVehicle {
    /**The length of a tick, in seconds.**/
    static final double TICK = 0.05;
    /**MTS's ground velocities, in blocks a tick before the speed factor: below the first a vehicle may skid steer, and below the second it changes which way it counts as going while steering.**/
    private static final double SKID_STEER_VELOCITY = 0.05;
    private static final double REVERSE_FLIP_VELOCITY = 0.1;

    /**How a vehicle speeds up and stops.  Accelerations are in m/s per second.**/
    static final class Spec {
        /**How hard the engine pushes at full throttle from a stop, falling to nothing at the top speed, in m/s.**/
        double power = 3;
        double topSpeed = 30;
        /**How hard the brakes stop it at full brake, or with the parking brake on.**/
        double brakes = 6;
        /**How long the engine takes to respond to the throttle, in seconds.**/
        double lag = 0.2;
        /**How fast it slows coasting.**/
        double drag = 0.3;
        /**How hard it pulls in gear with no throttle, as vehicles with automatic gearboxes do.**/
        double creep;
        /**How hard a slope pushes it forwards, negative for a slope pushing it back.**/
        double grade;
        /**True if it turns on the spot stopped in neutral.**/
        boolean skidSteer;

        Spec power(double power) {
            this.power = power;
            return this;
        }

        Spec brakes(double brakes) {
            this.brakes = brakes;
            return this;
        }

        Spec lag(double lag) {
            this.lag = lag;
            return this;
        }

        Spec drag(double drag) {
            this.drag = drag;
            return this;
        }

        Spec creep(double creep) {
            this.creep = creep;
            return this;
        }

        Spec grade(double grade) {
            this.grade = grade;
            return this;
        }

        Spec skidSteer(boolean skidSteer) {
            this.skidSteer = skidSteer;
            return this;
        }

        Spec copy() {
            Spec copy = new Spec().power(power).brakes(brakes).lag(lag).drag(drag).creep(creep).grade(grade).skidSteer(skidSteer);
            copy.topSpeed = topSpeed;
            return copy;
        }
    }

    private final Spec spec;
    private final VehicleKinematics kinematics;
    double x;
    double z;
    double yaw;
    /**How fast it's going along the way it faces, in m/s, negative backing up.**/
    double velocity;
    /**1 in a forward gear, -1 in reverse, 0 in neutral.**/
    int gear;
    boolean parkingBrake;
    /**True once MTS would count it as going in reverse, which is when it steers the other way.**/
    boolean goingInReverse;
    /**What the engine's giving of the throttle asked for, as it lags behind.**/
    private double engineThrottle;

    SimVehicle(Spec spec, VehicleKinematics kinematics, double x, double z, double yaw, double velocity) {
        this.spec = spec;
        this.kinematics = kinematics;
        this.x = x;
        this.z = z;
        this.yaw = yaw;
        this.velocity = velocity;
    }

    /**How fast it's going either way, in m/s, which is what MTS tells the summon job.**/
    double getSpeed() {
        return Math.abs(velocity);
    }

    /**Shifts a gear, as MTS does: one at a time, through neutral.**/
    void shift(DriveControls.Shift shift) {
        if (shift == DriveControls.Shift.UP) {
            gear = Math.min(1, gear + 1);
        } else if (shift == DriveControls.Shift.DOWN) {
            gear = Math.max(-1, gear - 1);
        }
    }

    /**Moves it on a tick with the controls held as given.**/
    void step(double rudder, double throttle, double brake) {
        engineThrottle += spec.lag > TICK ? (throttle - engineThrottle) * TICK / spec.lag : throttle - engineThrottle;
        double push = gear * (engineThrottle * spec.power * Math.max(0, 1 - Math.abs(velocity) / spec.topSpeed) + spec.creep) + spec.grade;
        double resistance = (parkingBrake ? 1 : Math.max(0, Math.min(1, brake))) * spec.brakes + spec.drag;
        double next = velocity + push * TICK;
        //Brakes and drag slow it whichever way it's going, and hold it once it's stopped, but never push it back.
        next = next > 0 ? Math.max(0, next - resistance * TICK) : Math.min(0, next + resistance * TICK);
        velocity = next;

        double groundVelocity = kinematics.toGroundVelocity(Math.abs(velocity));
        if (!goingInReverse && velocity < -1e-4 && (rudder == 0 || groundVelocity < REVERSE_FLIP_VELOCITY)) {
            goingInReverse = true;
        } else if (goingInReverse && velocity > 1e-4 && (rudder == 0 || groundVelocity < REVERSE_FLIP_VELOCITY)) {
            goingInReverse = false;
        }
        double moved = velocity * TICK;
        double turn;
        if (spec.skidSteer && gear == 0 && !parkingBrake && rudder != 0 && groundVelocity < SKID_STEER_VELOCITY) {
            //Positive rudder turns it right, which is yaw falling, a twentieth of the steering angle a tick.
            turn = -rudder / 20;
            goingInReverse = false;
        } else {
            turn = Math.toDegrees(kinematics.getCurvature(rudder, Math.abs(velocity)) * Math.abs(moved)) * (goingInReverse ? -1 : 1);
        }
        double midYaw = Math.toRadians(yaw + turn / 2);
        x += Math.sin(midYaw) * moved;
        z += Math.cos(midYaw) * moved;
        yaw = VehicleSummonFlight.clampAngle(yaw + turn);
    }
}
