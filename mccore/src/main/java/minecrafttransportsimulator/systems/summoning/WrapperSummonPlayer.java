package minecrafttransportsimulator.systems.summoning;

import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import minecrafttransportsimulator.systems.LanguageSystem;
import minecrafttransportsimulator.systems.LanguageSystem.LanguageEntry;

/**
 * A real player, as a {@link VehicleSummonJob} sees them.  Server only.
 *
 * @author HazedNapkin
 */
final class WrapperSummonPlayer implements ISummonPlayer {
    final IWrapperPlayer player;

    WrapperSummonPlayer(IWrapperPlayer player) {
        this.player = player;
    }

    @Override
    public UUID getID() {
        return player.getID();
    }

    @Override
    public Point3D getPosition() {
        return player.getPosition();
    }

    @Override
    public Point3D getLookDirection() {
        return player.getLineOfSight(1);
    }

    @Override
    public boolean hasKey(UUID keyID) {
        return VehiclePhone.hasKey(player, keyID);
    }

    @Override
    public void message(LanguageEntry language, String vehicleName) {
        VehiclePhone.message(player, language, vehicleName);
    }

    @Override
    public void debugMessage(String line) {
        player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, line);
    }
}
