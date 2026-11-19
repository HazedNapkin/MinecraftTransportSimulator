package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.systems.LanguageSystem.LanguageEntry;

/**
 * A made-up player for testing {@link VehicleSummonJob}: stands where they're put, looks along +Z unless turned, and
 * keeps every message they're sent.
 *
 * @author HazedNapkin
 */
final class FakeSummonPlayer implements ISummonPlayer {
    final UUID id = UUID.randomUUID();
    final Point3D position;
    final Point3D look = new Point3D(0, 0, 1);
    final Set<UUID> keys = new HashSet<>();
    final List<LanguageEntry> messages = new ArrayList<>();
    final List<String> debugLines = new ArrayList<>();

    FakeSummonPlayer(double x, double y, double z) {
        this.position = new Point3D(x, y, z);
    }

    @Override
    public UUID getID() {
        return id;
    }

    @Override
    public Point3D getPosition() {
        return position;
    }

    @Override
    public Point3D getLookDirection() {
        return look;
    }

    @Override
    public boolean hasKey(UUID keyID) {
        return keys.contains(keyID);
    }

    @Override
    public void message(LanguageEntry language, String vehicleName) {
        messages.add(language);
    }

    @Override
    public void debugMessage(String line) {
        debugLines.add(line);
    }

    /**The last message they were sent, or null if none.**/
    LanguageEntry lastMessage() {
        return messages.isEmpty() ? null : messages.get(messages.size() - 1);
    }
}
