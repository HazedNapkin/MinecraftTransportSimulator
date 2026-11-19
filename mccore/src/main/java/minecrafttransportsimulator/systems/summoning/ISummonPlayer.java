package minecrafttransportsimulator.systems.summoning;

import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.systems.LanguageSystem.LanguageEntry;

/**
 * The player calling a vehicle, as a {@link VehicleSummonJob} sees them.  {@link WrapperSummonPlayer} is the real one.
 *
 * @author HazedNapkin
 */
interface ISummonPlayer {
    UUID getID();

    /**Where they're standing.  This is the player's own, so copy it to keep it.**/
    Point3D getPosition();

    /**The way they're looking, a block long.**/
    Point3D getLookDirection();

    /**True if they're carrying the key with the ID.**/
    boolean hasKey(UUID keyID);

    /**Tells them something in chat, about the vehicle with the name.**/
    void message(LanguageEntry language, String vehicleName);

    /**Tells them a line of summon debugging in chat.**/
    void debugMessage(String line);
}
