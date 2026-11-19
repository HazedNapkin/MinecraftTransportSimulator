package minecrafttransportsimulator.systems.summoning;

import java.util.Collections;
import java.util.List;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.systems.navigation.DriveControls;
import minecrafttransportsimulator.systems.navigation.VehicleKinematics;
import minecrafttransportsimulator.systems.navigation.VehicleShape;

/**
 * A made-up vehicle for testing {@link VehicleSummonJob}.  It moves as a {@link SimVehicle} does, on flat ground at
 * the height it was put at, by whatever the job does with its controls, and keeps what a real one keeps between calls:
 * its controls, gear, parking brake and whether a call has marked it as driven.  It doesn't hit blocks: to make it
 * stuck, set {@link #jammed}.  Flying just notes what it was told.
 *
 * @author HazedNapkin
 */
final class FakeSummonVehicle implements ISummonVehicle {
    /**The least a control must change for the real vehicle to send it to clients.**/
    private static final double CONTROL_STEP = 0.005;

    final FakeSummonWorld world;
    final UUID phoneID;
    final SummonScenarios.Vehicle type;
    private final SimVehicle sim;
    private final double y;
    private final Point3D position = new Point3D();
    private final RotationMatrix orientation = new RotationMatrix();
    boolean valid = true;
    int ticksExisted;

    //What a real vehicle keeps.
    UUID riderID;
    boolean riderIsPlayer = true;
    UUID keyID;
    boolean aircraft;
    boolean flyIn;
    boolean wrecked;
    boolean hasEngine = true;
    boolean outOfFuel;
    boolean enginesRunning;
    boolean markedDriving;
    boolean simpleThrottle;
    boolean simpleThrottleBefore;
    double rudder;
    double throttle;
    double brake;
    double summonDistance;
    double summonSpeed;
    double arrivalAngle;
    double arrivalGap;
    /**Set to stop it moving at all, as if it had driven into something.**/
    boolean jammed;

    //Flying.
    boolean flown;
    boolean held;
    boolean gearUp;
    Point3D flyTarget;

    /**How many control changes it would have sent to clients.**/
    int controlsSent;

    FakeSummonVehicle(FakeSummonWorld world, UUID phoneID, SummonScenarios.Vehicle type, double x, double y, double z, double yaw, double speed) {
        this.world = world;
        this.phoneID = phoneID;
        this.type = type;
        this.y = y;
        this.sim = new SimVehicle(type.dynamics, type.kinematics, x, z, yaw, speed);
        syncPose();
    }

    /**Moves it on a tick, as MTS does after the summon job has had its tick.**/
    void step() {
        ++ticksExisted;
        if (jammed) {
            sim.velocity = 0;
        } else {
            sim.step(rudder, throttle, brake);
        }
        syncPose();
    }

    private void syncPose() {
        position.set(sim.x, y, sim.z);
        orientation.setToAngles(new Point3D(0, sim.yaw, 0));
    }

    boolean isParked() {
        return sim.parkingBrake && throttle == 0 && rudder == 0 && brake == 0 && !markedDriving;
    }

    int getGear() {
        return sim.gear;
    }

    boolean hasParkingBrake() {
        return sim.parkingBrake;
    }

    @Override
    public boolean isValid() {
        return valid;
    }

    @Override
    public Point3D getPosition() {
        return position;
    }

    @Override
    public RotationMatrix getOrientation() {
        return orientation;
    }

    @Override
    public double getYaw() {
        return sim.yaw;
    }

    @Override
    public double getSpeed() {
        return sim.getSpeed();
    }

    @Override
    public boolean isGoingInReverse() {
        return sim.goingInReverse;
    }

    @Override
    public VehicleShape getShape() {
        return type.shape;
    }

    @Override
    public VehicleKinematics getKinematics() {
        return type.kinematics;
    }

    @Override
    public UUID getRiderID() {
        return riderID;
    }

    @Override
    public void setRudder(double rudder) {
        this.rudder = send(this.rudder, rudder);
    }

    @Override
    public void setThrottle(double throttle) {
        this.throttle = send(this.throttle, throttle);
    }

    @Override
    public void setBrake(double brake) {
        this.brake = send(this.brake, brake);
    }

    private double send(double current, double value) {
        if (Math.abs(current - value) > CONTROL_STEP) {
            ++controlsSent;
            return value;
        }
        return current;
    }

    @Override
    public void setParkingBrake(boolean on) {
        sim.parkingBrake = on;
    }

    @Override
    public void startEngines() {
        startEngines(false);
    }

    @Override
    public void startEngines(boolean includeJets) {
        if (hasEngine) {
            enginesRunning = true;
        }
    }

    @Override
    public boolean shiftFor(boolean reverse, double speed) {
        if (!enginesRunning) {
            return false;
        }
        boolean shifting = DriveControls.isShifting(sim.gear, reverse);
        sim.shift(DriveControls.getShift(sim.gear, reverse, speed));
        return shifting;
    }

    @Override
    public void shiftToNeutral() {
        if (enginesRunning) {
            sim.gear = 0;
        }
    }

    @Override
    public void releaseControls() {
        setThrottle(0);
        setRudder(0);
        setBrake(0);
        if (sim.gear < 0) {
            sim.gear = 0;
        }
    }

    @Override
    public boolean canReverse() {
        return hasEngine && !type.dynamics.skidSteer;
    }

    @Override
    public boolean canPivot() {
        return hasEngine && type.dynamics.skidSteer;
    }

    @Override
    public boolean isSettled() {
        return ticksExisted >= 2;
    }

    @Override
    public boolean hasPlayerRider() {
        return riderID != null && riderIsPlayer;
    }

    @Override
    public UUID getKeyID() {
        return keyID;
    }

    @Override
    public boolean isAircraft() {
        return aircraft;
    }

    @Override
    public boolean canFlyIn() {
        return flyIn;
    }

    @Override
    public boolean isWrecked() {
        return wrecked;
    }

    @Override
    public boolean hasWorkingEngine(boolean includeJets) {
        return hasEngine;
    }

    @Override
    public boolean isOutOfFuel(boolean includeJets) {
        return outOfFuel;
    }

    @Override
    public double getSummonDistance() {
        return summonDistance;
    }

    @Override
    public double getSummonSpeed() {
        return summonSpeed;
    }

    @Override
    public double getArrivalAngle() {
        return arrivalAngle;
    }

    @Override
    public double getArrivalGap() {
        return arrivalGap;
    }

    @Override
    public double[] getAlignPoint(double arrivalAngle) {
        return new double[] { 0, 0 };
    }

    @Override
    public boolean isMarkedDriving() {
        return markedDriving;
    }

    @Override
    public void beginDriving() {
        if (!markedDriving) {
            simpleThrottleBefore = simpleThrottle;
            markedDriving = true;
        }
        simpleThrottle = true;
    }

    @Override
    public void stopControls() {
        if (markedDriving) {
            simpleThrottle = simpleThrottleBefore;
            markedDriving = false;
        }
        releaseControls();
    }

    @Override
    public void park() {
        stopControls();
        setParkingBrake(true);
    }

    @Override
    public Saved remove(Point3D position, double yaw, double speed) {
        valid = false;
        world.removed(this);
        return () -> {
            FakeSummonVehicle spawned = new FakeSummonVehicle(world, phoneID, type, position.x, position.y, position.z, yaw, speed);
            spawned.copyStateFrom(this);
            world.spawned(spawned);
            return spawned;
        };
    }

    /**Takes on everything a real vehicle saves, as it's spawned again from a save of the other.**/
    private void copyStateFrom(FakeSummonVehicle other) {
        riderID = other.riderID;
        riderIsPlayer = other.riderIsPlayer;
        keyID = other.keyID;
        aircraft = other.aircraft;
        flyIn = other.flyIn;
        wrecked = other.wrecked;
        hasEngine = other.hasEngine;
        outOfFuel = other.outOfFuel;
        enginesRunning = other.enginesRunning;
        markedDriving = other.markedDriving;
        simpleThrottle = other.simpleThrottle;
        simpleThrottleBefore = other.simpleThrottleBefore;
        rudder = other.rudder;
        throttle = other.throttle;
        brake = other.brake;
        summonDistance = other.summonDistance;
        summonSpeed = other.summonSpeed;
        arrivalAngle = other.arrivalAngle;
        arrivalGap = other.arrivalGap;
        flown = other.flown;
        held = other.held;
        gearUp = other.gearUp;
        flyTarget = other.flyTarget;
        sim.gear = other.sim.gear;
        sim.parkingBrake = other.sim.parkingBrake;
    }

    @Override
    public void flyTo(Point3D target, double speed) {
        flyTarget = target.copy();
        flown = true;
    }

    @Override
    public void turnTo(double yaw) {
    }

    @Override
    public void stopFlight() {
        flown = false;
    }

    @Override
    public boolean isFlown() {
        return flown;
    }

    @Override
    public void holdInAir() {
        flyTo(position, 0.2);
        held = true;
    }

    @Override
    public void releaseHold() {
        flown = false;
        held = false;
    }

    @Override
    public void clearHold() {
        held = false;
    }

    @Override
    public void setGearUp(boolean up) {
        gearUp = up;
    }

    @Override
    public boolean hasWheelsDown() {
        return !flown;
    }

    @Override
    public List<String> describeShape(VehicleShape shape) {
        return Collections.emptyList();
    }

    @Override
    public List<String> describeShapeBoxes(VehicleShape shape) {
        return Collections.emptyList();
    }

    @Override
    public void sendDebugMarkers(List<VehicleSummonDebug.Marker> markers) {
    }
}
