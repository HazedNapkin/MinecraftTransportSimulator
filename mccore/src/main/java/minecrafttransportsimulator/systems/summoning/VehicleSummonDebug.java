package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.ColorRGB;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.TransformationMatrix;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.packets.instances.PacketVehicleSummonDebug;
import minecrafttransportsimulator.systems.ConfigSystem;

/**
 * Shows what summon planning decided, for tuning it.  When the summonDebug config is on, the server sends each
 * vehicle's plan to clients as markers, and they draw them as small boxes along with the vehicle's hitboxes (F3+B).
 * A vehicle's markers stay until its next plan replaces them.
 */
public final class VehicleSummonDebug {
    /**Most markers sent at once, so a huge plan can't make a huge packet.**/
    public static final int MAX_MARKERS = 2048;
    private static final double MARKER_RADIUS = 0.15;

    /**What a marker shows, which sets its color.**/
    public enum Kind {
        /**A point on the route.**/
        PATH(ColorRGB.GREEN),
        /**Where the vehicle pulls up.**/
        GOAL(ColorRGB.BLUE),
        /**Where it was placed to drive in from.**/
        SPAWN(ColorRGB.YELLOW),
        /**A spot that was considered.**/
        CANDIDATE(ColorRGB.ORANGE),
        /**A spot that was ruled out.**/
        REJECTED(ColorRGB.RED);

        final ColorRGB color;

        Kind(ColorRGB color) {
            this.color = color;
        }
    }

    /**Markers each vehicle is showing on this client.**/
    private static final Map<EntityVehicleF_Physics, List<Marker>> shown = new WeakHashMap<>();

    private VehicleSummonDebug() {
    }

    /**True if plans should be sent to clients.**/
    static boolean isEnabled() {
        return ConfigSystem.settings.general.summonDebug.value;
    }

    /**Sends the markers to every client, replacing the vehicle's old ones, if debugging is on.  Server only.**/
    static void send(EntityVehicleF_Physics vehicle, List<Marker> markers) {
        if (isEnabled()) {
            InterfaceManager.packetInterface.sendToAllClients(new PacketVehicleSummonDebug(vehicle, markers.size() > MAX_MARKERS ? markers.subList(0, MAX_MARKERS) : markers));
        }
    }

    /**Makes markers of the kind at each of the points.**/
    static List<Marker> markers(List<Point3D> points, Kind kind) {
        List<Marker> markers = new ArrayList<>();
        for (Point3D point : points) {
            markers.add(new Marker(point, kind));
        }
        return markers;
    }

    /**Sets the markers the vehicle shows.  Client only.**/
    public static void show(EntityVehicleF_Physics vehicle, List<Marker> markers) {
        shown.put(vehicle, markers);
    }

    /**Draws the vehicle's markers, if it has any.  Called with the transform its hitboxes are drawn with.  Client only.**/
    public static void render(EntityVehicleF_Physics vehicle, TransformationMatrix transform) {
        for (Marker marker : shown.getOrDefault(vehicle, Collections.emptyList())) {
            marker.box.renderWireframe(vehicle, transform, null, marker.kind.color);
        }
    }

    /**A point to mark, and what it shows.**/
    public static final class Marker {
        public final Point3D position;
        public final Kind kind;
        private final BoundingBox box;

        public Marker(Point3D position, Kind kind) {
            this.position = position;
            this.kind = kind;
            this.box = new BoundingBox(position.copy().add(0, MARKER_RADIUS, 0), MARKER_RADIUS);
        }
    }
}
