package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;

import minecrafttransportsimulator.baseclasses.Point3D;

/**
 * A route for a vehicle to drive in along: closely spaced poses from where it sets off to where it pulls up, each
 * with the curvature it's driving there.  The curvature is what the vehicle steers for, so it can follow the route
 * by steering ahead of time rather than only correcting after it drifts off.
 */
final class SummonRoute {
    /**The poses along the route, in the order they're driven.  The first is where it sets off, the last where it pulls up.**/
    final List<Pose> poses;
    /**True if the vehicle is placed at the first pose before driving in, rather than already being there.**/
    final boolean placed;
    /**Where it pulls up.**/
    final ArrivalPlanner.Goal goal;

    SummonRoute(List<Pose> poses, boolean placed, ArrivalPlanner.Goal goal) {
        this.poses = poses;
        this.placed = placed;
        this.goal = goal;
    }

    Pose getStart() {
        return poses.get(0);
    }

    Pose getEnd() {
        return poses.get(poses.size() - 1);
    }

    /**How far the route is to drive, across the ground, in blocks.**/
    double getLength() {
        double length = 0;
        for (int i = 1; i < poses.size(); ++i) {
            length += Math.hypot(poses.get(i).x - poses.get(i - 1).x, poses.get(i).z - poses.get(i - 1).z);
        }
        return length;
    }

    /**The ground points along the route, for anything that only needs where it goes.**/
    List<Point3D> getPoints() {
        List<Point3D> points = new ArrayList<>();
        for (Pose pose : poses) {
            points.add(pose.getGround());
        }
        return points;
    }

    /**The vehicle's origin over x,z, its wheels on the ground at y, facing the yaw, and the curvature it's driving.**/
    static final class Pose {
        final double x;
        final int y;
        final double z;
        final double yaw;
        /**Radians of turn per block driven, positive turning left (yaw rising).**/
        final double curvature;

        Pose(double x, int y, double z, double yaw, double curvature) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.curvature = curvature;
        }

        Point3D getGround() {
            return new Point3D(x, y, z);
        }
    }
}
