package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;

/**
 * The space a vehicle takes up, in its own local space, taken from the boxes it collides with
 * blocks, entities or vehicles with, from its wheels, and from the space between its wheels.  Used to check the vehicle fits where we path,
 * teleport or land it.
 * <p>
 * Heights are measured up from the bottom of the wheels, which is where the ground is when the
 * vehicle is parked.  Like the vehicle's own collision boxes, each box stays square and
 * axis-aligned in the world as the vehicle turns: only its center rotates with the vehicle.
 */
final class VehicleShape {
    /**Boxes that overlap a block by less than this are not counted as being in it.**/
    private static final double CELL_OVERLAP_TOLERANCE = 0.15;
    /**
     * Boxes starting this high above the ground or higher aren't counted in the length of the vehicle's body: they're
     * things up over it, like a gun barrel or a roof rack, that stick out past it.
     */
    static final double BODY_HEIGHT = 1.5;

    /**Everything that needs room: the collision boxes and the wheels.**/
    final List<ShapeBox> boxes;
    /**Where the wheels touch the ground.  These need ground under them.**/
    final List<ShapeBox> supports;
    /**Lowest point below the vehicle origin (negative), usually the bottom of the wheels.**/
    final double bottom;
    /**Highest point above the vehicle origin.**/
    final double top;
    /**Furthest point forwards of the vehicle origin.**/
    final double front;
    /**Furthest point behind the vehicle origin, as a distance behind it.**/
    final double back;
    /**Furthest point to either side of the vehicle origin.**/
    final double side;
    /**
     * Length of the vehicle's body, front to back, leaving out things up over it that stick out past it (see
     * {@link #BODY_HEIGHT}).  It's how far the vehicle drives straight to settle lined up, which a long gun barrel
     * shouldn't stretch.
     */
    final double bodyLength;
    /**Distance from the origin to the furthest point of the vehicle, so it fits in this circle at any yaw.**/
    private final double radius;

    private VehicleShape(List<ShapeBox> boxes, List<ShapeBox> supports, double bottom, double top, double front, double back, double side, double bodyLength, double radius) {
        this.boxes = boxes;
        this.supports = supports;
        this.bottom = bottom;
        this.top = top;
        this.front = front;
        this.back = back;
        this.side = side;
        this.bodyLength = bodyLength;
        this.radius = radius;
    }

    /**Radius of a circle around the vehicle origin the vehicle fits in at any yaw.**/
    double radius() {
        return radius;
    }

    /**How far above the ground the vehicle origin sits when parked.**/
    double groundOffset() {
        return -bottom + 0.1;
    }

    /**
     * Returns the block columns the vehicle covers with its origin at the passed-in point and
     * facing the passed-in yaw, with the height range each column needs clear above the ground.
     * <p>
     * This is the hot path of planning, run for every pose checked, so it gathers the columns in a flat
     * grid round the origin rather than a map.  Nothing reaches further than {@link #radius} from the origin.
     */
    List<Column> getFootprint(double originX, double originZ, double yaw) {
        double sin = Math.sin(Math.toRadians(yaw));
        double cos = Math.cos(Math.toRadians(yaw));
        int baseX = (int) Math.floor(originX - radius) - 1;
        int baseZ = (int) Math.floor(originZ - radius) - 1;
        int size = (int) Math.ceil(2 * radius) + 3;
        double[] lows = new double[size * size];
        double[] highs = new double[size * size];
        boolean[] used = new boolean[size * size];
        int count = 0;
        for (ShapeBox box : boxes) {
            double centerX = originX + box.x * cos + box.z * sin;
            double centerZ = originZ - box.x * sin + box.z * cos;
            int minX = (int) Math.floor(centerX - box.radius + CELL_OVERLAP_TOLERANCE);
            int maxX = (int) Math.floor(centerX + box.radius - CELL_OVERLAP_TOLERANCE);
            int minZ = (int) Math.floor(centerZ - box.radius + CELL_OVERLAP_TOLERANCE);
            int maxZ = (int) Math.floor(centerZ + box.radius - CELL_OVERLAP_TOLERANCE);
            //Small boxes might not overlap any block by enough, but still need the block they're in.
            if (maxX < minX) {
                minX = maxX = (int) Math.floor(centerX);
            }
            if (maxZ < minZ) {
                minZ = maxZ = (int) Math.floor(centerZ);
            }
            for (int x = minX; x <= maxX; ++x) {
                for (int z = minZ; z <= maxZ; ++z) {
                    int index = (x - baseX) * size + (z - baseZ);
                    if (!used[index]) {
                        used[index] = true;
                        lows[index] = box.low;
                        highs[index] = box.high;
                        ++count;
                    } else {
                        lows[index] = Math.min(lows[index], box.low);
                        highs[index] = Math.max(highs[index], box.high);
                    }
                }
            }
        }
        List<Column> footprint = new ArrayList<>(count);
        for (int index = 0; index < used.length; ++index) {
            if (used[index]) {
                footprint.add(new Column(baseX + index / size, baseZ + index % size, lows[index], highs[index]));
            }
        }
        return footprint;
    }

    /**
     * Returns the block columns under the wheels, with the vehicle origin at the passed-in point
     * and facing the passed-in yaw.
     */
    List<Column> getSupports(double originX, double originZ, double yaw) {
        List<Column> columns = new ArrayList<>();
        double sin = Math.sin(Math.toRadians(yaw));
        double cos = Math.cos(Math.toRadians(yaw));
        for (ShapeBox support : supports) {
            int x = (int) Math.floor(originX + support.x * cos + support.z * sin);
            int z = (int) Math.floor(originZ - support.x * sin + support.z * cos);
            columns.add(new Column(x, z, support.low, support.high));
        }
        return columns;
    }

    static long columnKey(int x, int z) {
        return ((long) x << 32) | (z & 0xFFFFFFFFL);
    }

    /**
     * Takes the vehicle's shape as it is now.  The vehicle must have ticked at least once, so its
     * collision boxes are in place.
     */
    static VehicleShape of(EntityVehicleF_Physics vehicle) {
        List<double[]> rawBoxes = new ArrayList<>();
        List<double[]> rawWheels = new ArrayList<>();
        //Packs often leave a body out of block collision and only give it entity collision, so take both.
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (box.collisionTypes == null || !(box.collisionTypes.contains(CollisionType.BLOCK) || box.collisionTypes.contains(CollisionType.ENTITY) || box.collisionTypes.contains(CollisionType.VEHICLE))) {
                continue;
            }
            rawBoxes.add(toLocal(vehicle, box.globalCenter, Math.max(box.widthRadius, box.depthRadius), box.heightRadius));
        }
        for (APart part : vehicle.allParts) {
            if (part instanceof PartGroundDevice) {
                rawWheels.add(toLocal(vehicle, part.position, Math.max(part.getWidth(), part.getHeight()) / 2D, part.getHeight() / 2D));
            }
        }
        return of(rawBoxes, rawWheels);
    }

    /**
     * Makes the shape from boxes and wheels in the vehicle's local space, each as {x, y, z, horizontal radius,
     * vertical radius}.  Also used to make shapes for tests.
     */
    static VehicleShape of(List<double[]> rawBoxes, List<double[]> rawWheels) {
        if (rawBoxes.isEmpty() && rawWheels.isEmpty()) {
            //Nothing to go on.  Treat it as a block-sized box.
            rawBoxes = Collections.singletonList(new double[] { 0, 0, 0, 0.5, 0.5 });
        }

        //The ground is where the wheels touch it, or failing that, the bottom of the lowest box.
        double bottom = Double.MAX_VALUE;
        for (double[] wheel : rawWheels) {
            bottom = Math.min(bottom, wheel[1] - wheel[4]);
        }
        if (rawWheels.isEmpty()) {
            for (double[] box : rawBoxes) {
                bottom = Math.min(bottom, box[1] - box[4]);
            }
        }

        List<ShapeBox> boxes = new ArrayList<>();
        double top = bottom + 1;
        double front = 0;
        double back = 0;
        double side = 0;
        double bodyFront = 0;
        double bodyBack = 0;
        double radius = 0;
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minZ = Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;
        for (double[] raw : rawBoxes) {
            boxes.add(new ShapeBox(raw, bottom));
        }
        for (double[] raw : rawWheels) {
            boxes.add(new ShapeBox(raw, bottom));
        }
        boxes.addAll(getUnderbody(rawWheels, bottom));
        removeContained(boxes);
        for (ShapeBox box : boxes) {
            top = Math.max(top, bottom + box.high);
            front = Math.max(front, box.z + box.radius);
            back = Math.max(back, box.radius - box.z);
            side = Math.max(side, Math.abs(box.x) + box.radius);
            if (box.low < BODY_HEIGHT) {
                bodyFront = Math.max(bodyFront, box.z + box.radius);
                bodyBack = Math.max(bodyBack, box.radius - box.z);
            }
            //Corner furthest from the origin.
            radius = Math.max(radius, Math.hypot(Math.abs(box.x) + box.radius, Math.abs(box.z) + box.radius));
            minX = Math.min(minX, box.x - box.radius);
            maxX = Math.max(maxX, box.x + box.radius);
            minZ = Math.min(minZ, box.z - box.radius);
            maxZ = Math.max(maxZ, box.z + box.radius);
        }

        //Without wheels, it sits on the corners of its underside.
        List<ShapeBox> supports = new ArrayList<>();
        if (!rawWheels.isEmpty()) {
            for (double[] raw : rawWheels) {
                supports.add(new ShapeBox(raw, bottom));
            }
        } else {
            double insetX = (maxX - minX) * 0.1;
            double insetZ = (maxZ - minZ) * 0.1;
            for (double x : new double[] { minX + insetX, maxX - insetX }) {
                for (double z : new double[] { minZ + insetZ, maxZ - insetZ }) {
                    supports.add(new ShapeBox(new double[] { x, bottom, z, 0, 0 }, bottom));
                }
            }
        }
        front = Math.max(0.5, front);
        back = Math.max(0.5, back);
        //Something a vehicle's built from all up high, with nothing low down, is all body.
        double bodyLength = bodyFront + bodyBack > 0 ? Math.max(1, bodyFront + bodyBack) : front + back;
        return new VehicleShape(boxes, supports, bottom, top, front, back, Math.max(0.5, side), bodyLength, radius);
    }

    /**
     * Drops every box that another box holds entirely, across and in height, as it adds nothing to the shape.  Packs
     * often build a vehicle out of many overlapping collision boxes, and every box costs time in every pose checked.
     */
    private static void removeContained(List<ShapeBox> boxes) {
        for (int i = boxes.size() - 1; i >= 0; --i) {
            ShapeBox inner = boxes.get(i);
            for (int j = 0; j < boxes.size(); ++j) {
                ShapeBox outer = boxes.get(j);
                //Of two identical boxes, the later one goes.
                if (j != i && (j < i || !isInside(outer, inner)) && isInside(inner, outer)) {
                    boxes.remove(i);
                    break;
                }
            }
        }
    }

    /**True if the inner box is entirely within the outer one.**/
    private static boolean isInside(ShapeBox inner, ShapeBox outer) {
        return Math.abs(inner.x - outer.x) + inner.radius <= outer.radius + 1e-9 && Math.abs(inner.z - outer.z) + inner.radius <= outer.radius + 1e-9 && inner.low >= outer.low - 1e-9 && inner.high <= outer.high + 1e-9;
    }

    /**
     * Boxes filling the space between the wheels, as high as the wheels.  Whatever a pack gives the vehicle to collide
     * with, its body spans its wheels, so this keeps the planner from thinking the middle of it is empty and letting it
     * pull up with the player between its wheels or tracks.
     */
    private static List<ShapeBox> getUnderbody(List<double[]> rawWheels, double bottom) {
        List<ShapeBox> fill = new ArrayList<>();
        if (rawWheels.size() < 2) {
            return fill;
        }
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minZ = Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;
        double top = -Double.MAX_VALUE, smallest = Double.MAX_VALUE;
        for (double[] wheel : rawWheels) {
            minX = Math.min(minX, wheel[0]);
            maxX = Math.max(maxX, wheel[0]);
            minZ = Math.min(minZ, wheel[2]);
            maxZ = Math.max(maxZ, wheel[2]);
            top = Math.max(top, wheel[1] + wheel[4]);
            smallest = Math.min(smallest, wheel[3]);
        }
        //Squares as wide as the narrower way across, or as the smallest wheel if the wheels are in a line, along the longer way.
        boolean alongZ = maxZ - minZ >= maxX - minX;
        double across = alongZ ? maxX - minX : maxZ - minZ;
        double along = alongZ ? maxZ - minZ : maxX - minX;
        double radius = Math.max(across / 2, Math.min(smallest, 0.5));
        if (radius <= 0) {
            return fill;
        }
        double middleAcross = alongZ ? (minX + maxX) / 2 : (minZ + maxZ) / 2;
        double start = alongZ ? minZ : minX;
        int count = Math.max(1, (int) Math.ceil(along / (2 * radius)));
        double height = (top - bottom) / 2;
        for (int i = 0; i < count; ++i) {
            double middleAlong = start + (count == 1 ? along / 2 : radius + i * (along - 2 * radius) / (count - 1));
            double x = alongZ ? middleAcross : middleAlong;
            double z = alongZ ? middleAlong : middleAcross;
            fill.add(new ShapeBox(new double[] { x, bottom + height, z, radius, height }, bottom));
        }
        return fill;
    }

    /**Returns the box as {x, y, z, horizontal radius, vertical radius} in the vehicle's local space.**/
    private static double[] toLocal(EntityVehicleF_Physics vehicle, Point3D globalCenter, double horizontalRadius, double verticalRadius) {
        Point3D local = globalCenter.copy().subtract(vehicle.position).reOrigin(vehicle.orientation);
        return new double[] { local.x, local.y, local.z, horizontalRadius, verticalRadius };
    }

    /**A box of the vehicle: square in the horizontal, with heights measured up from the ground.**/
    static final class ShapeBox {
        final double x;
        final double z;
        final double radius;
        final double low;
        final double high;

        private ShapeBox(double[] raw, double ground) {
            this.x = raw[0];
            this.z = raw[2];
            this.radius = raw[3];
            this.low = raw[1] - raw[4] - ground;
            this.high = raw[1] + raw[4] - ground;
        }
    }

    /**A block column the vehicle covers, and the height range above the ground it needs clear there.**/
    static final class Column {
        final int x;
        final int z;
        double low;
        double high;

        private Column(int x, int z, double low, double high) {
            this.x = x;
            this.z = z;
            this.low = low;
            this.high = high;
        }
    }
}
