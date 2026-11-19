package minecrafttransportsimulator.systems.navigation;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.jsondefs.JSONPart.EngineType;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.packets.instances.PacketPartEngine;
import minecrafttransportsimulator.packets.instances.PacketPartEngine.Signal;

/**
 * A real MTS vehicle, driven through {@link IDrivenVehicle}.  Server only.
 *
 * @author HazedNapkin
 */
public class WrapperDrivenVehicle implements IDrivenVehicle {
    /**The least a control must change to be worth sending to every client.**/
    private static final double CONTROL_STEP = 0.005;

    public final EntityVehicleF_Physics vehicle;

    public WrapperDrivenVehicle(EntityVehicleF_Physics vehicle) {
        this.vehicle = vehicle;
    }

    @Override
    public boolean isValid() {
        return vehicle.isValid;
    }

    @Override
    public Point3D getPosition() {
        return vehicle.position;
    }

    @Override
    public RotationMatrix getOrientation() {
        return vehicle.orientation;
    }

    @Override
    public double getYaw() {
        return vehicle.orientation.angles.y;
    }

    @Override
    public double getSpeed() {
        //MTS's velocity is in blocks a tick before its speed factor.
        return vehicle.velocity * vehicle.speedFactor * 20;
    }

    @Override
    public boolean isGoingInReverse() {
        return vehicle.goingInReverse;
    }

    @Override
    public VehicleShape getShape() {
        return VehicleShape.of(vehicle);
    }

    @Override
    public VehicleKinematics getKinematics() {
        return VehicleKinematics.of(vehicle);
    }

    @Override
    public UUID getRiderID() {
        for (APart part : vehicle.allParts) {
            if (part.rider != null) {
                return part.rider.getID();
            }
        }
        return null;
    }

    @Override
    public void setRudder(double rudder) {
        set(vehicle.rudderInputVar, rudder);
    }

    @Override
    public void setThrottle(double throttle) {
        set(vehicle.throttleVar, throttle);
    }

    @Override
    public void setBrake(double brake) {
        set(vehicle.brakeVar, brake);
    }

    @Override
    public void setParkingBrake(boolean on) {
        set(vehicle.parkingBrakeVar, on ? 1 : 0);
    }

    @Override
    public void startEngines() {
        startEngines(false);
    }

    /**Starts every working engine that isn't running, jets too if includeJets is set.**/
    public void startEngines(boolean includeJets) {
        for (PartEngine engine : getWorkingEngines(includeJets)) {
            if (!engine.running) {
                if (engine.definition.engine.type == EngineType.MAGIC) {
                    //Magic engines start themselves once the magneto is on.  We can't use the
                    //auto-starter, as it refuses to start anything when the fuel tank is empty.
                    engine.magnetoVar.setTo(1, true);
                } else {
                    engine.autoStartEngine();
                    InterfaceManager.packetInterface.sendToAllClients(new PacketPartEngine(engine, Signal.AS_ON));
                }
            }
        }
    }

    @Override
    public boolean shiftFor(boolean reverse, double speed) {
        boolean shifting = false;
        for (PartEngine engine : getWorkingEngines(false)) {
            if (!engine.running) {
                continue;
            }
            double gear = engine.currentGearVar.currentValue;
            shifting |= DriveControls.isShifting(gear, reverse);
            DriveControls.Shift shift = DriveControls.getShift(gear, reverse, speed);
            if (shift == DriveControls.Shift.DOWN) {
                engine.shiftDown();
            } else if (shift == DriveControls.Shift.UP) {
                engine.shiftUp();
            }
        }
        return shifting;
    }

    @Override
    public void shiftToNeutral() {
        for (PartEngine engine : getWorkingEngines(false)) {
            if (engine.running && engine.currentGearVar.currentValue != 0) {
                engine.shiftNeutral();
            }
        }
    }

    @Override
    public void releaseControls() {
        set(vehicle.throttleVar, 0);
        set(vehicle.rudderInputVar, 0);
        set(vehicle.brakeVar, 0);
        for (PartEngine engine : vehicle.engines) {
            if (!engine.outOfHealth && engine.currentGearVar.currentValue < 0) {
                engine.shiftNeutral();
            }
        }
    }

    /**
     * True if the vehicle can back up the way a route plans: it has an engine to drive it, every engine has a
     * reverse gear, it steers with its wheels rather than always skid steering, and it isn't towing anything, as
     * backing a trailer is another thing altogether.  Vehicles that always skid steer turn on the spot instead.
     */
    @Override
    public boolean canReverse() {
        List<PartEngine> engines = getWorkingEngines(false);
        if (engines.isEmpty() || vehicle.definition.motorized.hasPermanentSkidSteer || !vehicle.towingConnections.isEmpty()) {
            return false;
        }
        for (PartEngine engine : engines) {
            if (engine.reverseGears == 0) {
                return false;
            }
        }
        return true;
    }

    /**
     * True if the vehicle can turn on the spot: it skid steers, as tracked vehicles do when stopped and mechs and
     * hovercraft always do, it has an engine to run in neutral, which MTS needs to skid steer, and it isn't towing anything.
     */
    @Override
    public boolean canPivot() {
        return (vehicle.definition.motorized.hasSkidSteer || vehicle.definition.motorized.hasPermanentSkidSteer) && !getWorkingEngines(false).isEmpty() && vehicle.towingConnections.isEmpty();
    }

    /**
     * The engines that aren't broken and that would move the vehicle.  Jets only count if includeJets is set, as they
     * don't drive a vehicle along the ground.
     */
    protected List<PartEngine> getWorkingEngines(boolean includeJets) {
        List<PartEngine> engines = new ArrayList<>();
        for (PartEngine engine : vehicle.engines) {
            if ((includeJets || !isJet(engine)) && !engine.outOfHealth) {
                engines.add(engine);
            }
        }
        return engines;
    }

    private static boolean isJet(PartEngine engine) {
        return engine.definition.generic.type.contains("jet");
    }

    /**Sets a variable and syncs it to clients, only if it changed enough to matter.**/
    protected static void set(ComputedVariable variable, double value) {
        if (Math.abs(variable.currentValue - value) > CONTROL_STEP) {
            variable.setTo(value, true);
        }
    }
}
