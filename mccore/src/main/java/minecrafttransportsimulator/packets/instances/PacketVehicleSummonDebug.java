package minecrafttransportsimulator.packets.instances;

import java.util.ArrayList;
import java.util.List;

import io.netty.buffer.ByteBuf;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.packets.components.APacketEntity;
import minecrafttransportsimulator.systems.summoning.VehicleSummonDebug;
import minecrafttransportsimulator.systems.summoning.VehicleSummonDebug.Kind;
import minecrafttransportsimulator.systems.summoning.VehicleSummonDebug.Marker;

/**
 * Packet sent from the server to clients with the markers showing a vehicle's summon plan, when the
 * summonDebug config is on.  Positions are sent as floats, which is plenty for markers.
 */
public class PacketVehicleSummonDebug extends APacketEntity<EntityVehicleF_Physics> {
    private final List<Marker> markers;

    public PacketVehicleSummonDebug(EntityVehicleF_Physics vehicle, List<Marker> markers) {
        super(vehicle);
        this.markers = markers;
    }

    public PacketVehicleSummonDebug(ByteBuf buf) {
        super(buf);
        int count = Math.min(buf.readShort(), VehicleSummonDebug.MAX_MARKERS);
        Kind[] kinds = Kind.values();
        this.markers = new ArrayList<>(count);
        for (int i = 0; i < count; ++i) {
            Point3D position = new Point3D(buf.readFloat(), buf.readFloat(), buf.readFloat());
            markers.add(new Marker(position, kinds[buf.readByte() % kinds.length]));
        }
    }

    @Override
    public void writeToBuffer(ByteBuf buf) {
        super.writeToBuffer(buf);
        buf.writeShort(markers.size());
        for (Marker marker : markers) {
            buf.writeFloat((float) marker.position.x);
            buf.writeFloat((float) marker.position.y);
            buf.writeFloat((float) marker.position.z);
            buf.writeByte(marker.kind.ordinal());
        }
    }

    @Override
    public boolean handle(AWrapperWorld world, EntityVehicleF_Physics vehicle) {
        VehicleSummonDebug.show(vehicle, markers);
        return false;
    }
}
