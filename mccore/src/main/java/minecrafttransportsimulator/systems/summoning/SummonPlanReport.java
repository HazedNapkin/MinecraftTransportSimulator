package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;
import minecrafttransportsimulator.systems.summoning.VehicleShape.ShapeBox;

/**
 * Describes what summon planning worked from and what it chose, for tracking down plans that go wrong, when summon
 * debugging is on.  The shape planning thinks the vehicle has is the usual suspect: everything that keeps it off
 * things and off the player goes by it, so a shape missing part of the vehicle lets that part through.
 */
final class SummonPlanReport {
    /**How much smaller across than the vehicle's whole box its shape may be before that's worth a warning.**/
    private static final double SMALL_SHAPE_RATIO = 0.7;

    private SummonPlanReport() {
    }

    /**
     * Chat lines describing the vehicle's shape: what it was made from, what was left out and why, and how far it
     * reaches, against the box round the whole vehicle.  Warns if it looks too small.
     */
    static List<String> getShape(EntityVehicleF_Physics vehicle, VehicleShape shape) {
        int used = 0;
        int skipped = 0;
        Set<String> skippedTypes = new TreeSet<>();
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (box.collisionTypes != null && (box.collisionTypes.contains(CollisionType.BLOCK) || box.collisionTypes.contains(CollisionType.ENTITY) || box.collisionTypes.contains(CollisionType.VEHICLE))) {
                ++used;
            } else {
                ++skipped;
                if (box.collisionTypes == null || box.collisionTypes.isEmpty()) {
                    skippedTypes.add("none");
                } else {
                    for (CollisionType type : box.collisionTypes) {
                        skippedTypes.add(type.name().toLowerCase());
                    }
                }
            }
        }
        int wheels = 0;
        for (APart part : vehicle.allParts) {
            if (part instanceof PartGroundDevice) {
                ++wheels;
            }
        }
        List<String> lines = new ArrayList<>();
        lines.add(String.format("Summon shape: from %d block, entity or vehicle collision boxes, %d wheels and the space between them; left out %d other collision boxes (types %s)", used, wheels, skipped, skippedTypes.isEmpty() ? "-" : String.join(", ", skippedTypes)));
        lines.add(String.format("Summon shape: reaches %.2f forward, %.2f back, %.2f each side, %.2f high, all within %.2f of the origin, body %.2f long.  The whole vehicle's box reaches %.2f across x, %.2f across z, %.2f up", shape.front, shape.back, shape.side, shape.top - shape.bottom, shape.radius(), shape.bodyLength, vehicle.encompassingBox.widthRadius, vehicle.encompassingBox.depthRadius, vehicle.encompassingBox.heightRadius));
        double shapeReach = Math.max(shape.side, Math.max(shape.front, shape.back));
        double vehicleReach = Math.max(vehicle.encompassingBox.widthRadius, vehicle.encompassingBox.depthRadius);
        if (shapeReach < vehicleReach * SMALL_SHAPE_RATIO) {
            lines.add(String.format("Summon shape: WARNING the shape reaches %.2f but the vehicle %.2f.  Parts left out of it can pass through blocks and the player", shapeReach, vehicleReach));
        }
        if (!shape.boxes.isEmpty() && !coversOrigin(shape)) {
            lines.add("Summon shape: WARNING nothing in the shape covers the vehicle's origin, so the planner thinks the middle of the vehicle is empty");
        }
        return lines;
    }

    /**Lines for the log describing every box of the shape, in the vehicle's local space.**/
    static List<String> getShapeBoxes(EntityVehicleF_Physics vehicle, VehicleShape shape) {
        List<String> lines = new ArrayList<>();
        lines.add(String.format("[Summon debug] Shape of %s: %d boxes, %d wheels.  Boxes are x (left), z (forward), half width, and height above the ground", vehicle.definition.systemName, shape.boxes.size(), shape.supports.size()));
        for (ShapeBox box : shape.boxes) {
            lines.add(String.format("[Summon debug]   box at x %.2f z %.2f, half width %.2f, from %.2f to %.2f high", box.x, box.z, box.radius, box.low, box.high));
        }
        for (ShapeBox support : shape.supports) {
            lines.add(String.format("[Summon debug]   wheel at x %.2f z %.2f", support.x, support.z));
        }
        return lines;
    }

    /**True if any box of the shape covers the column the vehicle's origin is over.**/
    private static boolean coversOrigin(VehicleShape shape) {
        for (ShapeBox box : shape.boxes) {
            if (Math.abs(box.x) <= box.radius && Math.abs(box.z) <= box.radius) {
                return true;
            }
        }
        return false;
    }

    /**
     * A chat line describing where the vehicle is to pull up: how far its origin is from the player and which way,
     * how near any part of its shape comes to them against the gap that was aimed for, how it faces, and how many
     * places were found.  Warns if its origin is closer to the player than half its width, as then it's on them.
     */
    static List<String> getGoal(ArrivalPlanner.Goal goal, ArrivalPlanner arrival, VehicleShape shape, Point3D player, double lookYaw, int goalCount) {
        List<String> lines = new ArrayList<>();
        double distance = Math.hypot(goal.x - player.x, goal.z - player.z);
        double bearing = VehicleSummonFlight.clampAngle(Math.toDegrees(Math.atan2(goal.x - player.x, goal.z - player.z)) - lookYaw);
        lines.add(String.format("Summon goal: origin %.2f from the player at %.0f deg off where they look, %d blocks %s, facing %.0f; nearest part %.2f away (aimed for %.2f); run-in %.1f, cost %.1f; best of %d", distance, bearing, Math.abs(goal.y - (int) Math.floor(player.y)), goal.y >= (int) Math.floor(player.y) ? "up" : "down", goal.yaw, arrival.getNearestDistance(goal.x, goal.z, goal.yaw), arrival.getGap(), goal.runIn, goal.cost, goalCount));
        if (distance < shape.side) {
            lines.add(String.format("Summon goal: WARNING its origin is only %.2f from the player, closer than half its width (%.2f), so it'll be on top of them.  The shape is likely missing what's round its origin", distance, shape.side));
        }
        return lines;
    }

    /**
     * Debug markers outlining the shape with the vehicle at the goal: the middle and corners of each box, at its
     * middle height.  Shows what the planner thinks the vehicle covers there.
     */
    static List<VehicleSummonDebug.Marker> getShapeMarkers(VehicleShape shape, ArrivalPlanner.Goal goal) {
        List<VehicleSummonDebug.Marker> markers = new ArrayList<>();
        double sin = Math.sin(Math.toRadians(goal.yaw));
        double cos = Math.cos(Math.toRadians(goal.yaw));
        for (ShapeBox box : shape.boxes) {
            //Boxes stay square to the world as the vehicle turns; only their middles turn with it.
            double x = goal.x + box.x * cos + box.z * sin;
            double z = goal.z - box.x * sin + box.z * cos;
            double y = goal.y + (box.low + box.high) / 2;
            markers.add(new VehicleSummonDebug.Marker(new Point3D(x, y, z), VehicleSummonDebug.Kind.SHAPE));
            for (int corner = 0; corner < 4; ++corner) {
                double cornerX = x + (corner % 2 == 0 ? box.radius : -box.radius);
                double cornerZ = z + (corner < 2 ? box.radius : -box.radius);
                markers.add(new VehicleSummonDebug.Marker(new Point3D(cornerX, y, cornerZ), VehicleSummonDebug.Kind.SHAPE));
            }
        }
        return markers;
    }
}
