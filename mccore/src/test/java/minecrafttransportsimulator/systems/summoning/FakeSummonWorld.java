package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.systems.navigation.FakeTerrain;
import minecrafttransportsimulator.systems.navigation.ITerrainQuery;

/**
 * A made-up world for testing {@link VehicleSummonJob}, of {@link FakeTerrain} blocks, {@link FakeSummonPlayer}s and
 * {@link FakeSummonVehicle}s.  A vehicle can be put in unloaded, at a last known position: it loads once the job keeps
 * its chunk loaded.  {@link #tick()} runs a server tick after the job's: time moves on and every vehicle moves.
 * <p>
 * It counts what a test needs to check nothing was lost or duplicated: how many of each vehicle are in the world, and
 * how many times one was teleported.
 *
 * @author HazedNapkin
 */
final class FakeSummonWorld implements ISummonWorld {
    final FakeTerrain terrain;
    final Map<UUID, FakeSummonPlayer> players = new HashMap<>();
    /**Every vehicle in the world, loaded or not.**/
    final List<FakeSummonVehicle> vehicles = new ArrayList<>();
    private final Map<UUID, Point3D> lastPositions = new HashMap<>();
    /**Vehicles that aren't loaded, by where they are.**/
    private final Map<UUID, FakeSummonVehicle> unloaded = new HashMap<>();
    final List<String> errors = new ArrayList<>();
    long time;
    int teleports;
    /**What's spawned this tick, which only moves from next tick, as an entity spawned during the world tick does.**/
    private final List<FakeSummonVehicle> justSpawned = new ArrayList<>();

    FakeSummonWorld(FakeTerrain terrain) {
        this.terrain = terrain;
    }

    FakeSummonPlayer addPlayer(double x, double y, double z) {
        FakeSummonPlayer player = new FakeSummonPlayer(x, y, z);
        players.put(player.id, player);
        return player;
    }

    /**Puts a vehicle of the type on the ground at the position, loaded and settled in.**/
    FakeSummonVehicle addVehicle(SummonScenarios.Vehicle type, double x, double z, double yaw) {
        FakeSummonVehicle vehicle = new FakeSummonVehicle(this, UUID.randomUUID(), type, x, type.shape.groundOffset(), z, yaw, 0);
        vehicle.ticksExisted = 100;
        vehicle.setParkingBrake(true);
        vehicles.add(vehicle);
        lastPositions.put(vehicle.phoneID, vehicle.getPosition().copy());
        return vehicle;
    }

    /**Puts a vehicle of the type on the ground at the position, but not loaded, so the job has to load its chunk.**/
    FakeSummonVehicle addUnloadedVehicle(SummonScenarios.Vehicle type, double x, double z, double yaw) {
        FakeSummonVehicle vehicle = addVehicle(type, x, z, yaw);
        vehicle.ticksExisted = 0;
        unloaded.put(vehicle.phoneID, vehicle);
        return vehicle;
    }

    /**Runs the rest of a server tick: time moves on, and every loaded vehicle moves.**/
    void tick() {
        ++time;
        for (FakeSummonVehicle vehicle : vehicles) {
            if (vehicle.valid && !unloaded.containsKey(vehicle.phoneID) && !justSpawned.contains(vehicle)) {
                vehicle.step();
            }
        }
        justSpawned.clear();
    }

    /**How many of the vehicle with the phone ID are in the world, loaded or not.**/
    int count(UUID phoneID) {
        int count = 0;
        for (FakeSummonVehicle vehicle : vehicles) {
            if (vehicle.valid && vehicle.phoneID.equals(phoneID)) {
                ++count;
            }
        }
        return count;
    }

    /**The vehicle with the phone ID in the world, or null if there isn't one.**/
    FakeSummonVehicle get(UUID phoneID) {
        for (FakeSummonVehicle vehicle : vehicles) {
            if (vehicle.valid && vehicle.phoneID.equals(phoneID)) {
                return vehicle;
            }
        }
        return null;
    }

    void removed(FakeSummonVehicle vehicle) {
        ++teleports;
    }

    void spawned(FakeSummonVehicle vehicle) {
        vehicles.add(vehicle);
        justSpawned.add(vehicle);
        lastPositions.put(vehicle.phoneID, vehicle.getPosition().copy());
    }

    @Override
    public ISummonPlayer getPlayer(UUID playerID) {
        return players.get(playerID);
    }

    @Override
    public ISummonVehicle getLoadedVehicle(UUID phoneID) {
        if (unloaded.containsKey(phoneID)) {
            return null;
        }
        return get(phoneID);
    }

    @Override
    public Point3D getLastPosition(UUID phoneID) {
        return lastPositions.get(phoneID);
    }

    @Override
    public void forgetPosition(UUID phoneID) {
        lastPositions.remove(phoneID);
    }

    @Override
    public long getTime() {
        return time;
    }

    @Override
    public void keepChunkLoaded(Point3D position) {
        //Loads any vehicle in the same chunk.
        int chunkX = (int) Math.floor(position.x) >> 4;
        int chunkZ = (int) Math.floor(position.z) >> 4;
        unloaded.values().removeIf(vehicle -> ((int) Math.floor(vehicle.getPosition().x) >> 4) == chunkX && ((int) Math.floor(vehicle.getPosition().z) >> 4) == chunkZ);
    }

    @Override
    public ITerrainQuery getTerrain() {
        return terrain;
    }

    @Override
    public boolean isAir(int x, int y, int z) {
        return terrain.getBlock(x, y, z) == 0;
    }

    @Override
    public boolean isLiquid(int x, int y, int z) {
        return (terrain.getBlock(x, y, z) & ITerrainQuery.LIQUID) != 0;
    }

    @Override
    public boolean isSolidTop(int x, int y, int z) {
        return terrain.isSolid(x, y, z);
    }

    @Override
    public long getMaxHeight() {
        return FakeTerrain.MAX_Y + 1;
    }

    @Override
    public boolean isDebugEnabled() {
        return false;
    }

    @Override
    public void logError(String message) {
        errors.add(message);
    }
}
