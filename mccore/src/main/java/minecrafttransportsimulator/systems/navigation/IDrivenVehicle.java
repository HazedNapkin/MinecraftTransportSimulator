package minecrafttransportsimulator.systems.navigation;

import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;

/**
 * A ground vehicle as whatever drives it sees it: where it is, how fast it's going, and its controls.  Something that
 * drives a vehicle by itself, like an autopilot following a {@link Route}, goes through this rather than the vehicle,
 * so it can be tested with a made-up one.  {@link WrapperDrivenVehicle} is the real one.
 *
 * @author HazedNapkin
 */
public interface IDrivenVehicle {
    /**False once the vehicle has gone from the world.**/
    boolean isValid();

    /**Where its origin is.  This is the vehicle's own, so copy it to keep it.**/
    Point3D getPosition();

    /**Which way it's facing, pitch and roll included.  This is the vehicle's own, so copy it to keep it.**/
    RotationMatrix getOrientation();

    /**Which way it's facing, in degrees: see the conventions in {@link Route}.**/
    double getYaw();

    /**How fast it's going, whichever way, in m/s.**/
    double getSpeed();

    /**True if MTS counts it as going backwards, which is when its steering works the other way.**/
    boolean isGoingInReverse();

    /**The space it takes up.  Worked out afresh each call, so keep it.**/
    VehicleShape getShape();

    /**How it steers.  Worked out afresh each call, so keep it.**/
    VehicleKinematics getKinematics();

    /**Whoever is riding anywhere in it, or null if no one is.**/
    UUID getRiderID();

    /**Sets the steering, as MTS's rudder input in degrees: positive turns right.  Only sent to clients if it changed enough to matter.**/
    void setRudder(double rudder);

    /**Sets the throttle, 0 to 1.  Only sent to clients if it changed enough to matter.**/
    void setThrottle(double throttle);

    /**Sets the brake, 0 to 1.  Only sent to clients if it changed enough to matter.**/
    void setBrake(double brake);

    void setParkingBrake(boolean on);

    /**Starts every working engine that drives it along the ground that isn't running.**/
    void startEngines();

    /**
     * Shifts the engines towards the gear for going forwards, or backing up if reverse is set: see
     * {@link DriveControls#getShift}.  Returns true while it's still shifting between forwards and reverse, which it
     * has to stop for.
     */
    boolean shiftFor(boolean reverse, double speed);

    /**Puts every running engine that's in gear into neutral, as MTS needs to turn a skid steering vehicle on the spot.**/
    void shiftToNeutral();

    /**Lets go of the throttle, steering and brake, and takes any engine out of reverse, so whoever drives next isn't caught out.**/
    void releaseControls();

    /**True if it can back up the way a route plans: see {@link WrapperDrivenVehicle#canReverse()}.**/
    boolean canReverse();

    /**True if it can turn on the spot: see {@link WrapperDrivenVehicle#canPivot()}.**/
    boolean canPivot();
}
