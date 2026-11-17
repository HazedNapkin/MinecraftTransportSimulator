package minecrafttransportsimulator.systems.navigation;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * Where a route ends: the vehicle's origin over x,z, its wheels on the ground at y, facing the yaw.  Each goal has a
 * clear straight run-in behind it, so a route can finish by driving straight onto it.
 *
 * @author HazedNapkin
 */
public final class RouteGoal {
    public final double x;
    public final int y;
    public final double z;
    public final double yaw;
    /**How long the clear straight run-in behind it is, in blocks.**/
    public final double runIn;
    /**Extra cost of ending a route here, in blocks of route.**/
    public final double cost;

    public RouteGoal(double x, int y, double z, double yaw, double runIn, double cost) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.runIn = runIn;
        this.cost = cost;
    }

    /**The ground point under the vehicle origin.**/
    public Point3D getGround() {
        return new Point3D(x, y, z);
    }
}
