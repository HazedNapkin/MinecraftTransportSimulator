package minecrafttransportsimulator.systems.summoning;

import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.blocks.components.ABlockBase.Axis;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.systems.navigation.ITerrainQuery;

/**
 * A real world, as a {@link VehicleSummonJob} sees it.  Server only.
 *
 * @author HazedNapkin
 */
final class WrapperSummonWorld implements ISummonWorld {
    final AWrapperWorld world;
    /**The last player looked up, kept so looking them up every tick doesn't make a new wrapper each time.**/
    private WrapperSummonPlayer lastPlayer;

    WrapperSummonWorld(AWrapperWorld world) {
        this.world = world;
    }

    @Override
    public ISummonPlayer getPlayer(UUID playerID) {
        Object entity = world.getExternalEntity(playerID);
        if (!(entity instanceof IWrapperPlayer)) {
            return null;
        }
        if (lastPlayer == null || lastPlayer.player != entity) {
            lastPlayer = new WrapperSummonPlayer((IWrapperPlayer) entity);
        }
        return lastPlayer;
    }

    @Override
    public ISummonVehicle getLoadedVehicle(UUID phoneID) {
        EntityVehicleF_Physics vehicle = VehiclePhone.getLoadedVehicle(world, phoneID);
        return vehicle != null ? new WrapperSummonVehicle(vehicle) : null;
    }

    @Override
    public Point3D getLastPosition(UUID phoneID) {
        return VehicleLocationTracker.getLastPosition(world, phoneID);
    }

    @Override
    public void forgetPosition(UUID phoneID) {
        VehicleLocationTracker.forget(world, phoneID);
    }

    @Override
    public long getTime() {
        return world.getTime();
    }

    @Override
    public void keepChunkLoaded(Point3D position) {
        world.keepChunkLoaded(position);
    }

    @Override
    public ITerrainQuery getTerrain() {
        return ITerrainQuery.of(world);
    }

    @Override
    public boolean isAir(int x, int y, int z) {
        return world.isAir(new Point3D(x + 0.5, y + 0.5, z + 0.5));
    }

    @Override
    public boolean isLiquid(int x, int y, int z) {
        return world.isBlockLiquid(new Point3D(x + 0.5, y + 0.5, z + 0.5));
    }

    @Override
    public boolean isSolidTop(int x, int y, int z) {
        return world.isBlockSolid(new Point3D(x + 0.5, y + 0.5, z + 0.5), Axis.UP);
    }

    @Override
    public long getMaxHeight() {
        return world.getMaxHeight();
    }

    @Override
    public boolean isDebugEnabled() {
        return VehicleSummonDebug.isEnabled();
    }

    @Override
    public void logError(String message) {
        InterfaceManager.coreInterface.logError(message);
    }
}
