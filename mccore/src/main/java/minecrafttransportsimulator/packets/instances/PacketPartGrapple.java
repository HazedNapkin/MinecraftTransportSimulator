package minecrafttransportsimulator.packets.instances;

import java.util.UUID;

import io.netty.buffer.ByteBuf;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.PartGrapple;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.packets.components.APacketEntity;

/**
 * Packet used to send a grapple's aim to the server while its controller looks through an animated camera.
 * The camera turns the controller to face along it, so the server can't tell where they're really aiming
 * from the way they face.  Sent to the server by the controller's client when the aim changes, then on to all
 * clients so they turn the launcher the same way.  Null angles mean the controller stopped using the camera.
 */
public class PacketPartGrapple extends APacketEntity<PartGrapple> {
    private final UUID controllerID;
    private final Point3D angles;

    public PacketPartGrapple(PartGrapple grapple, UUID controllerID, Point3D angles) {
        super(grapple);
        this.controllerID = controllerID;
        this.angles = angles != null ? angles.copy() : null;
    }

    public PacketPartGrapple(ByteBuf buf) {
        super(buf);
        boolean active = buf.readBoolean();
        this.controllerID = readUUIDFromBuffer(buf);
        this.angles = active ? new Point3D(buf.readDouble(), buf.readDouble(), 0) : null;
    }

    @Override
    public void writeToBuffer(ByteBuf buf) {
        super.writeToBuffer(buf);
        buf.writeBoolean(angles != null);
        writeUUIDToBuffer(controllerID, buf);
        if (angles != null) {
            buf.writeDouble(angles.x);
            buf.writeDouble(angles.y);
        }
    }

    @Override
    public boolean handle(AWrapperWorld world, PartGrapple grapple) {
        return grapple.setCameraAimAngles(controllerID, angles);
    }
}
