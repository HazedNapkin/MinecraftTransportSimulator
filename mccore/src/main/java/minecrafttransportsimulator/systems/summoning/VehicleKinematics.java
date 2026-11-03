package minecrafttransportsimulator.systems.summoning;

import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;

/**
 * How a ground vehicle turns, following MTS's own steering in AEntityVehicleD_Moving.getTurningForce().
 * <p>
 * At a steering angle of s degrees, a vehicle turns s / wheelbase * scale * groundVelocity * (speedFactor / 0.35) / 2
 * degrees a tick, while moving groundVelocity * speedFactor blocks.  The speeds cancel out, so it turns
 * s * scale / (0.7 * wheelbase) degrees for each block it drives.  The scale is 1 at low speed and drops off
 * above a ground velocity of 0.35, unless the vehicle ignores speed, so faster vehicles can't turn as tight.
 * The vehicle yaws about its origin, so that's the point that follows the curves worked out here.
 * <p>
 * Speeds here are in m/s, and curvatures in radians per block, positive turning left (towards the vehicle's +X).
 */
final class VehicleKinematics {
    /**Ground velocity above which steering weakens with speed, unless the vehicle ignores speed.**/
    private static final double STEERING_FADE_VELOCITY = 0.35;
    private static final double STEERING_FADE_BASE = 0.3;

    /**The distance steering divides by: the turning wheelbase, or 1 for vehicles with permanent skid steer.**/
    private final double wheelbase;
    private final double speedFactor;
    private final double steeringForceFactor;
    private final boolean steeringIgnoresSpeed;
    /**Curvature at full lock before any fade with speed, or 0 if the vehicle can't steer.**/
    private final double baseMaxCurvature;

    VehicleKinematics(double wheelbase, double speedFactor, double steeringForceFactor, boolean steeringIgnoresSpeed) {
        this.wheelbase = wheelbase;
        this.speedFactor = speedFactor;
        this.steeringForceFactor = steeringForceFactor;
        this.steeringIgnoresSpeed = steeringIgnoresSpeed;
        this.baseMaxCurvature = wheelbase > 0 ? Math.toRadians(EntityVehicleF_Physics.MAX_RUDDER_ANGLE / (0.7 * wheelbase)) : 0;
    }

    /**Takes the vehicle's steering as it is now.  It should be on the ground, as only grounded wheels count towards its wheelbase.**/
    static VehicleKinematics of(EntityVehicleF_Physics vehicle) {
        double wheelbase = vehicle.definition.motorized.hasPermanentSkidSteer ? 1 : vehicle.groundDeviceCollective.getTurningWheelbase();
        return new VehicleKinematics(wheelbase, vehicle.speedFactor, vehicle.steeringForceFactorVar.currentValue, vehicle.steeringForceIgnoresSpeedVar.isActive);
    }

    /**True if the vehicle can steer at all.**/
    boolean canSteer() {
        return baseMaxCurvature > 0 && (!steeringIgnoresSpeed || steeringForceFactor > 0);
    }

    /**MTS's ground velocity for a speed: the motion per tick before the speed factor is applied.**/
    double toGroundVelocity(double speed) {
        return speed / 20D / speedFactor;
    }

    /**How much steering is left at the speed, from 1 down towards 0.**/
    double getSteeringScale(double speed) {
        if (steeringIgnoresSpeed) {
            return Math.max(0, steeringForceFactor);
        }
        double groundVelocity = toGroundVelocity(Math.abs(speed));
        return groundVelocity > STEERING_FADE_VELOCITY ? Math.pow(STEERING_FADE_BASE, groundVelocity * (1 - steeringForceFactor) - STEERING_FADE_VELOCITY) : 1;
    }

    /**The tightest curvature the vehicle can drive at the speed, at full lock.**/
    double getMaxCurvature(double speed) {
        return baseMaxCurvature * getSteeringScale(speed);
    }

    /**The radius of the tightest circle the vehicle can drive round at the speed, or infinity if it can't steer.**/
    double getTurnRadius(double speed) {
        double curvature = getMaxCurvature(speed);
        return curvature > 0 ? 1 / curvature : Double.POSITIVE_INFINITY;
    }

    /**
     * The fastest the vehicle can go and still drive the curvature, either way.  Returns infinity if speed doesn't
     * limit it, and 0 if it can't drive it at any speed.
     */
    double getMaxSpeed(double curvature) {
        double needed = Math.abs(curvature);
        if (needed == 0) {
            return Double.POSITIVE_INFINITY;
        }
        if (!canSteer()) {
            return 0;
        }
        double ratio = needed / baseMaxCurvature;
        if (steeringIgnoresSpeed) {
            return ratio <= steeringForceFactor ? Double.POSITIVE_INFINITY : 0;
        }
        if (steeringForceFactor >= 1) {
            //Steering never fades.  Over 1 it even strengthens with speed, but we don't plan on needing speed to turn.
            return ratio <= 1 ? Double.POSITIVE_INFINITY : 0;
        }
        //Solve 0.3 ^ (v * (1 - factor) - 0.35) = ratio for the ground velocity v.  The scale only falls as v rises.
        double groundVelocity = (STEERING_FADE_VELOCITY + Math.log(ratio) / Math.log(STEERING_FADE_BASE)) / (1 - steeringForceFactor);
        if (groundVelocity <= STEERING_FADE_VELOCITY) {
            //Below the fade the scale is 1, so it's only drivable there if it needs no more than full lock.
            groundVelocity = ratio <= 1 ? STEERING_FADE_VELOCITY : 0;
        }
        return groundVelocity * 20D * speedFactor;
    }

    /**
     * The rudder input that drives the curvature at the speed, held to full lock.  MTS steers cars with the rudder
     * input, and a positive input steers right, so it's the opposite sign to the curvature.
     */
    double getRudderInput(double curvature, double speed) {
        double maxCurvature = getMaxCurvature(speed);
        if (maxCurvature <= 0) {
            return 0;
        }
        double steering = EntityVehicleF_Physics.MAX_RUDDER_ANGLE * curvature / maxCurvature;
        return -Math.max(-EntityVehicleF_Physics.MAX_RUDDER_ANGLE, Math.min(EntityVehicleF_Physics.MAX_RUDDER_ANGLE, steering));
    }

    double getWheelbase() {
        return wheelbase;
    }
}
