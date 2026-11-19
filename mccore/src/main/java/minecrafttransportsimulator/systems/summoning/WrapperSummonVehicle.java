package minecrafttransportsimulator.systems.summoning;

import java.util.List;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.items.instances.ItemVehicle;
import minecrafttransportsimulator.jsondefs.JSONPart.EngineType;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.systems.ConfigSystem;
import minecrafttransportsimulator.systems.navigation.VehicleShape;
import minecrafttransportsimulator.systems.navigation.WrapperDrivenVehicle;

/**
 * A real MTS vehicle, as a {@link VehicleSummonJob} sees it.  Server only.
 *
 * @author HazedNapkin
 */
final class WrapperSummonVehicle extends WrapperDrivenVehicle implements ISummonVehicle {

    WrapperSummonVehicle(EntityVehicleF_Physics vehicle) {
        super(vehicle);
    }

    @Override
    public boolean isSettled() {
        return vehicle.ticksExisted >= 2;
    }

    @Override
    public boolean hasPlayerRider() {
        return VehiclePhone.hasPlayerRider(vehicle);
    }

    @Override
    public UUID getKeyID() {
        return vehicle.keyUUID;
    }

    @Override
    public boolean isAircraft() {
        return vehicle.definition.motorized.isAircraft;
    }

    @Override
    public boolean canFlyIn() {
        return VehiclePhone.canFlyIn(vehicle);
    }

    @Override
    public boolean isWrecked() {
        return vehicle.outOfHealth;
    }

    @Override
    public boolean hasWorkingEngine(boolean includeJets) {
        return !getWorkingEngines(includeJets).isEmpty();
    }

    /**Magic engines don't burn fuel, so a vehicle powered only by them never runs out.**/
    @Override
    public boolean isOutOfFuel(boolean includeJets) {
        if (vehicle.isCreative || ConfigSystem.settings.general.fuelUsageFactor.value == 0 || vehicle.fuelTank.getFluidLevel() != 0) {
            return false;
        }
        for (PartEngine engine : getWorkingEngines(includeJets)) {
            if (engine.definition.engine.type != EngineType.MAGIC) {
                return true;
            }
        }
        return false;
    }

    @Override
    public double getSummonDistance() {
        return vehicle.summonDistanceVar.currentValue;
    }

    @Override
    public double getSummonSpeed() {
        return vehicle.summonSpeedVar.currentValue;
    }

    @Override
    public double getArrivalAngle() {
        return vehicle.summonArrivalAngleVar.currentValue;
    }

    @Override
    public double getArrivalGap() {
        return vehicle.summonArrivalGapVar.currentValue;
    }

    @Override
    public double[] getAlignPoint(double arrivalAngle) {
        return ArrivalPlanner.getAlignPoint(vehicle, arrivalAngle);
    }

    @Override
    public boolean isMarkedDriving() {
        return vehicle.getOrCreateVariable(VehiclePhone.VAR_DRIVING).isActive;
    }

    @Override
    public void beginDriving() {
        ComputedVariable driving = vehicle.getOrCreateVariable(VehiclePhone.VAR_DRIVING);
        if (!driving.isActive) {
            vehicle.getOrCreateVariable(VehiclePhone.VAR_SIMPLE_THROTTLE_BEFORE).setActive(vehicle.isSimpleThrottleVar.isActive, false);
            driving.setActive(true, false);
        }
        if (!vehicle.isSimpleThrottleVar.isActive) {
            vehicle.isSimpleThrottleVar.setActive(true, true);
        }
    }

    @Override
    public void stopControls() {
        ComputedVariable driving = vehicle.getOrCreateVariable(VehiclePhone.VAR_DRIVING);
        if (driving.isActive) {
            ComputedVariable simpleThrottleBefore = vehicle.getOrCreateVariable(VehiclePhone.VAR_SIMPLE_THROTTLE_BEFORE);
            if (vehicle.isSimpleThrottleVar.isActive != simpleThrottleBefore.isActive) {
                vehicle.isSimpleThrottleVar.setActive(simpleThrottleBefore.isActive, true);
            }
            simpleThrottleBefore.setActive(false, false);
            driving.setActive(false, false);
        }
        releaseControls();
    }

    @Override
    public void park() {
        stopControls();
        setParkingBrake(true);
    }

    /**
     * Clients only see vehicle movement through MTS's movement deltas, so just changing the position on the server
     * would leave them with the vehicle in the old place.  So it's saved and spawned again from the save instead.  It
     * keeps its phone ID, so the phone still knows it, and anything else it saves, like its engines running.
     */
    @Override
    public Saved remove(Point3D position, double yaw, double speed) {
        vehicle.disconnectAllConnections();
        IWrapperNBT data = vehicle.save(InterfaceManager.coreInterface.getNewNBTWrapper());
        data.setPoint3d("position", position);
        data.setPoint3d("angles", new Point3D(0, yaw, 0));
        //Motion is per tick, before the speed factor MTS scales it by.
        double motion = speed / 20D / vehicle.speedFactor;
        data.setPoint3d("motion", new Point3D(Math.sin(Math.toRadians(yaw)) * motion, 0, Math.cos(Math.toRadians(yaw)) * motion));
        data.setPoint3d("serverDeltaM", new Point3D());
        data.setPoint3d("serverDeltaR", new Point3D());
        data.setDouble("serverDeltaP", 0);
        AWrapperWorld world = vehicle.world;
        ItemVehicle item = (ItemVehicle) vehicle.cachedItem;
        vehicle.remove();
        return () -> {
            EntityVehicleF_Physics spawned = new EntityVehicleF_Physics(world, null, item, data);
            spawned.addPartsPostAddition(null, data);
            world.spawnEntity(spawned);
            VehicleLocationTracker.update(spawned);
            return new WrapperSummonVehicle(spawned);
        };
    }

    @Override
    public void flyTo(Point3D target, double speed) {
        VehicleSummonFlight.flyTo(vehicle, target, speed);
    }

    @Override
    public void turnTo(double yaw) {
        VehicleSummonFlight.turnTo(vehicle, yaw);
    }

    @Override
    public void stopFlight() {
        VehicleSummonFlight.stop(vehicle);
    }

    @Override
    public boolean isFlown() {
        return vehicle.getOrCreateVariable(VehicleSummonFlight.VAR_ACTIVE).isActive;
    }

    @Override
    public void holdInAir() {
        VehiclePhone.holdInAir(vehicle);
    }

    @Override
    public void releaseHold() {
        VehiclePhone.releaseHold(vehicle);
    }

    @Override
    public void clearHold() {
        set(vehicle.getOrCreateVariable(VehiclePhone.VAR_FLY_HELD), 0);
    }

    @Override
    public void setGearUp(boolean up) {
        set(vehicle.getOrCreateVariable(VehiclePhone.VAR_GEAR), up ? 1 : 0);
    }

    @Override
    public boolean hasWheelsDown() {
        //Note MTS's isAnythingOnGround() is true when nothing is on the ground, so check the grounded set directly.
        return !vehicle.groundDeviceCollective.groundedGroundDevices.isEmpty();
    }

    @Override
    public List<String> describeShape(VehicleShape shape) {
        return SummonPlanReport.getShape(vehicle, shape);
    }

    @Override
    public List<String> describeShapeBoxes(VehicleShape shape) {
        return SummonPlanReport.getShapeBoxes(vehicle, shape);
    }

    @Override
    public void sendDebugMarkers(List<VehicleSummonDebug.Marker> markers) {
        VehicleSummonDebug.send(vehicle, markers);
    }
}
