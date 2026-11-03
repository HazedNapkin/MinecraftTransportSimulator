package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.baseclasses.EntityInteractResult;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.items.components.AItemBase;
import minecrafttransportsimulator.items.instances.ItemItem;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.jsondefs.JSONItem.ItemComponentType;
import minecrafttransportsimulator.mcinterface.IWrapperItemStack;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.packets.instances.PacketPlayerChatMessage;
import minecrafttransportsimulator.systems.LanguageSystem;
import minecrafttransportsimulator.systems.LanguageSystem.LanguageEntry;

/**
 * Helpers for vehicle phone items.  Vehicles are registered on a phone by sneak-clicking them,
 * and are stored on the phone's stack, so whoever holds the phone can call them.
 */
public final class VehiclePhone {
    /**MTS's landing gear variable: 1 raises the gear, 0 lowers it.**/
    static final String VAR_GEAR = "gear_setpoint";
    /**
     * Set on an aircraft a call has left in the sky, which the summon flight holds in place until a player gets in.
     * Saved with the vehicle, so the hold carries over a restart.
     */
    static final String VAR_FLY_HELD = "phone_fly_held";
    static final int MAX_VEHICLES = 8;
    /**Top speed the summon flight uses to hold an aircraft in place, in blocks per tick.**/
    private static final double HOLD_SPEED = 0.2;
    /**Same reach as the clicks that {@link minecrafttransportsimulator.systems.ControlSystem} sends to entities.**/
    private static final double CLICK_DISTANCE = 3.5;
    private static final String DATA_VEHICLES = "phoneVehicles";
    /**Saved with the vehicle, and on its item when picked up, so placing it again keeps the ID phones know it by.**/
    public static final String PHONE_UUID_TAG = "phoneUUID";
    private static final String ENTRY_SEPARATOR = ";";

    private VehiclePhone() {
    }

    public static boolean isPhone(AItemBase item) {
        return item instanceof ItemItem && ((ItemItem) item).definition.item.type == ItemComponentType.PHONE;
    }

    /**Registers the vehicle on the phone the player is holding.  Server only.**/
    public static void registerVehicle(EntityVehicleF_Physics vehicle, IWrapperPlayer player) {
        if (!canControl(player, vehicle)) {
            message(player, LanguageSystem.INTERACT_PHONE_NEEDKEY);
            return;
        }
        IWrapperItemStack stack = player.getHeldStack();
        List<PhoneEntry> entries = getEntries(stack);
        String name = getName(vehicle);
        UUID phoneID = getPhoneID(vehicle);
        entries.removeIf(entry -> entry.vehicleID.equals(phoneID));
        if (entries.size() >= MAX_VEHICLES) {
            message(player, LanguageSystem.INTERACT_PHONE_FULL);
            return;
        }
        if (vehicle.phoneUUID == null) {
            //Keep the ID it's had so far, so phones it was registered on before still know it.
            //It only reaches clients when they next load the vehicle, but until then they fall back to the same ID.
            vehicle.phoneUUID = phoneID;
        }
        entries.add(new PhoneEntry(phoneID, name));
        setEntries(stack, entries);
        VehicleLocationTracker.update(vehicle);
        message(player, LanguageSystem.INTERACT_PHONE_REGISTERED, name);
    }

    /**
     * The ID phones know this vehicle by.  Vehicles registered before phone IDs were saved, or never
     * registered at all, go by their {@link EntityVehicleF_Physics#uniqueUUID} until they're registered.
     */
    public static UUID getPhoneID(EntityVehicleF_Physics vehicle) {
        return vehicle.phoneUUID != null ? vehicle.phoneUUID : vehicle.uniqueUUID;
    }

    /**The loaded vehicle phones know by this ID, or null if it isn't loaded.  Works on both sides.**/
    public static EntityVehicleF_Physics getLoadedVehicle(AWrapperWorld world, UUID phoneID) {
        Object entity = world.getEntity(phoneID);
        if (entity instanceof EntityVehicleF_Physics && getPhoneID((EntityVehicleF_Physics) entity).equals(phoneID)) {
            return (EntityVehicleF_Physics) entity;
        }
        //Vehicles that have been picked up and placed again have a new uniqueUUID, so look through them all.
        for (EntityVehicleF_Physics vehicle : world.getEntitiesOfType(EntityVehicleF_Physics.class)) {
            if (phoneID.equals(vehicle.phoneUUID) && vehicle.isValid) {
                return vehicle;
            }
        }
        return null;
    }

    /**
     * True if the player may register and call this vehicle: it has no key, or they're
     * carrying its key.  Operators get no exception, as isOP() is true for every player in
     * singleplayer and LAN worlds.
     */
    static boolean canControl(IWrapperPlayer player, EntityVehicleF_Physics vehicle) {
        if (vehicle.keyUUID == null) {
            return true;
        }
        for (int i = 0; i < player.getInventory().getSize(); ++i) {
            IWrapperNBT data = player.getInventory().getStack(i).getData();
            if (data != null && vehicle.keyUUID.equals(data.getUUID(ItemItem.KEY_UUID_TAG))) {
                return true;
            }
        }
        return false;
    }

    /**
     * True if the player is aiming at a vehicle or one of its parts.  Clicks on those go to
     * {@link ItemItem#doEntityInteraction}, so the phone screen shouldn't also open.  This is checked
     * server-side because 1.12.2 sends vanilla's item use to the server before it can be cancelled on the client.
     */
    public static boolean isAimingAtVehicle(IWrapperPlayer player) {
        Point3D startPosition = player.getEyePosition();
        Point3D endPosition = player.getLineOfSight(CLICK_DISTANCE).add(startPosition);
        EntityInteractResult result = player.getWorld().getMultipartEntityIntersect(startPosition, endPosition);
        return result != null && (result.entity instanceof EntityVehicleF_Physics || (result.entity instanceof APart && ((APart) result.entity).vehicleOn != null));
    }

    /**
     * True if this aircraft flies itself over to the player when called, rather than being placed on the ground:
     * its motorized summonFlyIn is set, or a variable modifier has turned it on.
     */
    static boolean canFlyIn(EntityVehicleF_Physics vehicle) {
        return vehicle.summonFlyInVar.isActive;
    }

    /**
     * Leaves the aircraft in the sky where it is until a player gets in.  The summon flight holds it in place, as
     * otherwise it'd be left to its own flight physics and could fall.  Server only.
     */
    static void holdInAir(EntityVehicleF_Physics vehicle) {
        VehicleSummonFlight.flyTo(vehicle, vehicle.position.copy(), HOLD_SPEED);
        setIfChanged(vehicle, VAR_FLY_HELD, 1);
    }

    /**
     * Gives an aircraft a call left in the sky to the player who's got in: stops the summon flight, so it's back to
     * its own flight physics, set up as it was before the call.  Server only.
     */
    static void releaseHold(EntityVehicleF_Physics vehicle) {
        VehicleSummonFlight.stop(vehicle);
        setIfChanged(vehicle, VAR_FLY_HELD, 0);
    }

    /**True if a player is riding anywhere in the vehicle.**/
    static boolean hasPlayerRider(EntityVehicleF_Physics vehicle) {
        for (APart part : vehicle.allParts) {
            if (part.rider instanceof IWrapperPlayer) {
                return true;
            }
        }
        return false;
    }

    /**Sets the variable and syncs it to clients, only if it's changed.**/
    private static void setIfChanged(EntityVehicleF_Physics vehicle, String variableName, double value) {
        ComputedVariable variable = vehicle.getOrCreateVariable(variableName);
        if (variable.currentValue != value) {
            variable.setTo(value, true);
        }
    }

    static String getName(EntityVehicleF_Physics vehicle) {
        return vehicle.subDefinition.name != null ? vehicle.subDefinition.name : vehicle.definition.general.name;
    }

    public static List<PhoneEntry> getEntries(IWrapperItemStack stack) {
        List<PhoneEntry> entries = new ArrayList<>();
        IWrapperNBT data = stack.getData();
        if (data != null) {
            for (String value : data.getStrings(DATA_VEHICLES)) {
                int split = value.indexOf(ENTRY_SEPARATOR);
                if (split > 0) {
                    try {
                        entries.add(new PhoneEntry(UUID.fromString(value.substring(0, split)), value.substring(split + 1)));
                    } catch (IllegalArgumentException e) {
                        //Corrupt entry, drop it.
                    }
                }
            }
        }
        return entries;
    }

    static void setEntries(IWrapperItemStack stack, List<PhoneEntry> entries) {
        IWrapperNBT data = stack.getData();
        if (data == null) {
            data = InterfaceManager.coreInterface.getNewNBTWrapper();
        }
        List<String> values = new ArrayList<>();
        entries.forEach(entry -> values.add(entry.vehicleID + ENTRY_SEPARATOR + entry.name));
        data.setStrings(DATA_VEHICLES, values);
        stack.setData(data);
    }

    static void message(IWrapperPlayer player, LanguageEntry language, Object... args) {
        if (player != null) {
            player.sendPacket(new PacketPlayerChatMessage(player, language, args));
        }
    }

    /**A vehicle registered on a phone.**/
    public static final class PhoneEntry {
        public final UUID vehicleID;
        public final String name;

        PhoneEntry(UUID vehicleID, String name) {
            this.vehicleID = vehicleID;
            this.name = name;
        }
    }
}
