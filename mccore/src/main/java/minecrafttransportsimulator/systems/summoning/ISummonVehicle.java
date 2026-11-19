package minecrafttransportsimulator.systems.summoning;

import java.util.List;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.systems.navigation.IDrivenVehicle;
import minecrafttransportsimulator.systems.navigation.VehicleShape;

/**
 * A vehicle as a {@link VehicleSummonJob} sees it: what it drives, plus what a call needs to know and do besides, like
 * whether it's fit to come, teleporting it and flying it.  {@link WrapperSummonVehicle} is the real one.
 *
 * @author HazedNapkin
 */
interface ISummonVehicle extends IDrivenVehicle {
    /**True once it's been in the world long enough for its collision boxes to be in place.**/
    boolean isSettled();

    /**True if a player is riding anywhere in it.**/
    boolean hasPlayerRider();

    /**The ID of the key that unlocks it, or null if it has none.**/
    UUID getKeyID();

    boolean isAircraft();

    /**True if it's an aircraft that flies itself over when called: see {@link VehiclePhone#canFlyIn}.**/
    boolean canFlyIn();

    /**True if it's broken.**/
    boolean isWrecked();

    /**True if it has an engine that isn't broken: one that drives it along the ground, or any if includeJets is set.**/
    boolean hasWorkingEngine(boolean includeJets);

    /**True if the working engines it would use, jets too if includeJets is set, need fuel, and the tank's empty.**/
    boolean isOutOfFuel(boolean includeJets);

    /**Starts every working engine that isn't running, jets too if includeJets is set.**/
    void startEngines(boolean includeJets);

    /**The pack's summon settings, each 0 if it doesn't set it: how far off to set off from in blocks, how fast to drive in in km/h, which side to pull up facing the player in degrees, and how far from them in blocks.**/
    double getSummonDistance();

    double getSummonSpeed();

    double getArrivalAngle();

    double getArrivalGap();

    /**The point to line up opposite the player, in its local space as {x, z}: see {@link ArrivalPlanner#getAlignPoint}.**/
    double[] getAlignPoint(double arrivalAngle);

    /**True if a call has marked it as being driven, and hasn't let go of it yet.**/
    boolean isMarkedDriving();

    /**
     * Readies it for a call to drive: marks it as driven, so it's parked if the call goes without letting go, remembers
     * its simple throttle to put back, and turns simple throttle on, so MTS changes gear as it goes.
     */
    void beginDriving();

    /**Lets go of the controls, and if a call was driving it, puts its simple throttle back and unmarks it.**/
    void stopControls();

    /**Stops the controls and holds it where it is with the parking brake.  Engines are left running.**/
    void park();

    /**
     * Saves it as it would be at the position, facing the yaw and moving forwards at the speed in m/s, then removes it
     * from the world, leaving it in neither the world nor anything else until what's returned is spawned.  Anything
     * it's towing or towed by is let go of first.
     */
    Saved remove(Point3D position, double yaw, double speed);

    //Flying.  See VehicleSummonFlight.

    /**Flies it to the target at up to the speed, in blocks a tick, overriding its own flight.**/
    void flyTo(Point3D target, double speed);

    /**Turns it smoothly to face the yaw as it flies, whichever way it's moving.**/
    void turnTo(double yaw);

    /**Hands it back to its own flight physics.**/
    void stopFlight();

    /**True while it's being flown by {@link #flyTo}.**/
    boolean isFlown();

    /**Holds it where it is in the air until a player gets in: see {@link VehiclePhone#holdInAir}.**/
    void holdInAir();

    /**Gives an aircraft held in the air to the player who's got in: see {@link VehiclePhone#releaseHold}.**/
    void releaseHold();

    /**Takes it off hold, as a new call takes over from whatever left it in the sky, without stopping the flight.**/
    void clearHold();

    /**Raises or lowers its landing gear.**/
    void setGearUp(boolean up);

    /**True if any of its wheels are on the ground.**/
    boolean hasWheelsDown();

    //Debugging.

    /**Chat lines for the player describing the shape planning thinks it has: see {@link SummonPlanReport#getShape}.**/
    List<String> describeShape(VehicleShape shape);

    /**Log lines describing every box of the shape: see {@link SummonPlanReport#getShapeBoxes}.**/
    List<String> describeShapeBoxes(VehicleShape shape);

    /**Shows the markers on it to every client.**/
    void sendDebugMarkers(List<VehicleSummonDebug.Marker> markers);

    /**A vehicle saved and removed to be put somewhere else.**/
    interface Saved {
        /**Spawns it where it was saved to be, and notes that's where it is.  Call once only.**/
        ISummonVehicle spawn();
    }
}
