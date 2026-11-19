package minecrafttransportsimulator.systems.summoning;

import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.systems.navigation.ITerrainQuery;

/**
 * The world as a {@link VehicleSummonJob} sees it: the players and vehicles in it, keeping chunks loaded, and the
 * blocks a flight needs to know about.  Planning asks about blocks through {@link #getTerrain()}.
 * {@link WrapperSummonWorld} is the real one.
 *
 * @author HazedNapkin
 */
interface ISummonWorld {
    /**The player with the ID if they're in this world, or null.**/
    ISummonPlayer getPlayer(UUID playerID);

    /**The loaded vehicle phones know by this ID, or null if it isn't loaded.**/
    ISummonVehicle getLoadedVehicle(UUID phoneID);

    /**Where the vehicle was last saved, or null if that's not known: see {@link VehicleLocationTracker}.**/
    Point3D getLastPosition(UUID phoneID);

    /**Forgets where the vehicle was, as it's not there any more.**/
    void forgetPosition(UUID phoneID);

    /**The world's time, in ticks.**/
    long getTime();

    /**Keeps the chunk with the position in loaded for a while.  Needs calling again every so often to keep it loaded.**/
    void keepChunkLoaded(Point3D position);

    /**Asks the world about blocks, for planning.  A new one each call.**/
    ITerrainQuery getTerrain();

    /**What the block is: air, liquid, and whether its top is solid enough to land on.**/
    boolean isAir(int x, int y, int z);

    boolean isLiquid(int x, int y, int z);

    boolean isSolidTop(int x, int y, int z);

    /**The height of the top of the world.**/
    long getMaxHeight();

    /**True if summon debugging is on, which tells the player more and shows them markers.**/
    boolean isDebugEnabled();

    void logError(String message);
}
