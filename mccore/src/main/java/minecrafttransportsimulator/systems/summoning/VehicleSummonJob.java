package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.blocks.components.ABlockBase.Axis;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.items.instances.ItemVehicle;
import minecrafttransportsimulator.jsondefs.JSONPart.EngineType;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.packets.instances.PacketPartEngine;
import minecrafttransportsimulator.packets.instances.PacketPartEngine.Signal;
import minecrafttransportsimulator.systems.ConfigSystem;
import minecrafttransportsimulator.systems.LanguageSystem;
import minecrafttransportsimulator.systems.LanguageSystem.LanguageEntry;

/**
 * One vehicle answering one phone call.  Runs on the server, one step per tick.
 * <p>
 * Ground vehicles: find (force-loading the chunk if needed), plan a route in to the player with a {@link RoutePlanner},
 * from somewhere hidden about the vehicle's summonDistance blocks away, teleport there and drive in with the real
 * controls, following the route's curves and slowing for its turns, steps and the end.  A vehicle that's already
 * nearby drives over from where it is.  If no route exists, teleport to where it would have pulled up.  If there's no
 * room by the player, teleport to the nearest place further out it fits.  If stuck, re-plan once, then teleport.
 * <p>
 * Aircraft that fly in (see {@link VehiclePhone#canFlyIn}): teleport in above the buildings, fly over the
 * player, and land straight down in the first clear spot within {@link #LANDING_RADIUS} blocks, trying in front
 * of the player first, then the sides, then behind.  If there's nowhere to land, the summon flight holds it above
 * the player until a player gets in.  Other aircraft are teleported straight onto the ground near the player.
 * <p>
 * Wherever a vehicle ends up by the player, it has the side its summonArrivalAngle says towards them.
 */
final class VehicleSummonJob {
    private static final int DEFAULT_SUMMON_DISTANCE = 30;
    private static final int LOAD_TIMEOUT = 200;
    /**How often to renew the chunk loading for a vehicle we're waking up, in ticks.**/
    private static final int CHUNK_LOAD_INTERVAL = 20;
    /**
     * How much further out than its arrival gap a vehicle that's put by the player rather than driving in may go to
     * find somewhere it fits, in blocks: aircraft that don't fly in, and anything with no room right by the player.
     */
    private static final double PLACEMENT_MAX_EXTRA_GAP = 16;
    /**Speed to drive in at, in km/h, if the vehicle doesn't set its own summonSpeed.**/
    private static final double DEFAULT_SUMMON_SPEED = 30;
    /**
     * Speed routes plan their turns for, in m/s.  Vehicles steer less the faster they go, so the
     * drive slows to this before turns to make them as tight as planned.
     */
    private static final double TURN_SPEED = 3;
    /**How much wider than the vehicle's tightest turn the route's turns are, to leave room for steering error.**/
    private static final double TURN_RADIUS_MARGIN = 1.25;
    /**The least to drive at short of the end, in m/s.**/
    private static final double MIN_SPEED = 1;
    /**How hard the drive plans to brake, in m/s per second, so it slows smoothly to a stop at the end.**/
    private static final double BRAKING = 2.5;
    /**
     * If set, a vehicle placed to drive in is placed already moving at the speed planned for where it sets off, so it
     * rolls in rather than pulling away from a standstill.
     */
    private static final boolean SPAWN_MOVING = true;
    /**
     * If set, a vehicle that sets off from where it is may back up, out of where it's nosed in or in a three-point
     * turn, if it can't drive out forward.
     */
    private static final boolean REVERSE_ENABLED = true;
    /**
     * If set, a vehicle that skid steers may turn on the spot where that saves a lot of driving: to face the way out
     * where it sets off, to line up with where it pulls up, or in place of a three-point turn.
     */
    private static final boolean PIVOT_ENABLED = true;
    /**How fast MTS turns a skid steering vehicle on the spot at full lock, in degrees a second: a twentieth of the steering angle a tick.**/
    private static final double PIVOT_RATE = EntityVehicleF_Physics.MAX_RUDDER_ANGLE / 20 * 20;
    /**
     * How slow the vehicle must be to shift between forward and reverse, and how slow it must be going the new way
     * before it's steered, in m/s.  MTS only turns a vehicle the way it's going once it's going that way, and a
     * vehicle that can skid steer spins on the spot in neutral, so it's held straight until then.
     */
    private static final double SHIFT_SPEED = 0.3;
    private static final double STEER_SETTLE_SPEED = 0.5;
    /**Extra time allowed for each change of direction, for stopping, shifting and setting off, in seconds.**/
    private static final double DIRECTION_CHANGE_SECONDS = 4;
    /**
     * How near the end a vehicle that's all but stopped counts as there, in blocks, and how slow is all but stopped,
     * in m/s.  Braking can bring it to rest just short, and it shouldn't sit there until it's called stuck.
     */
    private static final double STALLED_ARRIVE_DISTANCE = 1;
    private static final double STALLED_SPEED = 0.3;
    private static final int STUCK_CHECK_INTERVAL = 100;
    /**How many ticks into the drive debug markers are first sent, once clients have the vehicle, and how often after.**/
    private static final int DEBUG_SEND_DELAY = 5;
    private static final int DEBUG_RESEND_INTERVAL = 100;
    /**
     * Longest a vehicle may take to drive in before it's brought straight to the player instead, in ticks.
     * Routes too long to drive in this time at {@link #TURN_SPEED} get longer: see {@link #getDriveTimeout()}.
     */
    private static final int MIN_DRIVE_TIMEOUT = 20 * 30;
    /**Extra time allowed on long routes for starting the engine and slowing down at the end, in seconds.**/
    private static final double DRIVE_TIMEOUT_MARGIN = 10;
    /**
     * How close the front of a vehicle driving in may get to a player in its way before it stops, in blocks, and how
     * far beyond its sides they still count as in its way.  Routes keep off the player, so this is only for when
     * they've walked into its way.
     */
    private static final double STOP_FRONT_DISTANCE = 1.5;
    private static final double STOP_SIDE_MARGIN = 0.3;
    private static final int LANDING_RADIUS = 20;
    /**
     * Longest an aircraft may spend flying in before it's put on the ground in front of the player instead, in ticks.
     * A normal flight takes 10 to 20 seconds, or around 40 when descending past tall buildings.
     */
    private static final int FLIGHT_TIMEOUT = 20 * 90;
    private static final double CRUISE_CLEARANCE = 10;
    private static final double FLY_SPEED = 1.0;
    private static final double DESCEND_SPEED = 0.2;
    /**How far below the ground the descent aims, so it reaches the ground at a steady rate.**/
    private static final double LANDING_OVERSHOOT = 1.0;
    /**
     * How high above the ground the origin waits while an aircraft finishes turning to its landing heading,
     * in blocks, so it never touches down crooked.
     */
    private static final double LANDING_TURN_HOLD = 2;
    /**Ticks to keep pressing down after first touching the ground.**/
    private static final int SETTLE_TICKS = 10;

    private enum Phase {
        LOCATE,
        FIND_SPOT,
        PLAN_GROUND,
        REPLAN,
        TELEPORT_SPAWN,
        DRIVE,
        CLIMB,
        FLY_TO_PLAYER,
        FIND_LANDING,
        FLY_TO_LANDING,
        DESCEND
    }

    /**What a spot search near the player is for, which says what to do with what it finds.**/
    private enum SpotPurpose {
        /**Where a ground vehicle is teleported when it can't drive in.**/
        FALLBACK,
        /**Where an aircraft that can't fly in is teleported.**/
        AIRCRAFT,
        /**Where an aircraft that took too long flying in is teleported.**/
        FLIGHT_TIMEOUT
    }

    final UUID vehicleID;
    final AWrapperWorld world;
    private final UUID playerID;
    private final String name;
    private Phase phase = Phase.LOCATE;
    private int phaseTicks;
    /**How long this tick's searches may take, in nanoseconds, as shared out by {@link VehicleSummonManager}.**/
    private long tickTime;
    private boolean cancelled;

    private EntityVehicleF_Physics vehicle;
    private VehicleShape shape;
    private Point3D forcedChunk;
    /**True when the vehicle was already loaded and in driving range, so it must never be teleported.**/
    private boolean driveOnly;

    //Teleporting is remove now, spawn next tick, so the old entity is fully gone first.
    private IWrapperNBT teleportData;
    private ItemVehicle teleportItem;
    private Phase afterTeleport;
    private LanguageEntry afterTeleportMessage;

    //Ground planning.
    /**Where the player was, and the way they were looking, when planning started.  Spots are ranked from in front of them.**/
    private Point3D planCenter;
    private double frontAngle;
    /**Finds where to put a vehicle by the player that doesn't drive in, and what for.**/
    private ArrivalPlanner placementSearch;
    private SpotPurpose spotPurpose;
    private VehicleKinematics kinematics;
    private RoutePlanner planner;
    /**Where the vehicle is brought if it can't drive in: the best place by the player it fits, or null if it fits nowhere.**/
    private ArrivalPlanner.Goal fallbackGoal;
    /**
     * Markers showing the plan being driven, if summon debugging is on, else null.  They're sent a few ticks into the
     * drive and every so often after: sent as a placed vehicle is spawned, clients don't have it yet and drop them.
     */
    private List<VehicleSummonDebug.Marker> planMarkers;

    //Driving.
    private SummonRoute route;
    /**The speeds planned along the route, and what steers the vehicle along it.**/
    private SpeedProfile profile;
    private PathTracker tracker;
    /**How the drive's going, kept if summon debugging is on, else null.**/
    private SummonDriveReport driveReport;
    /**The speed to place the vehicle at the start of the route moving at, in m/s, if it's placed there.**/
    private double spawnSpeed;
    private Point3D lastProgressPosition;
    /**The tick of the drive it last changed direction at, as stopping to doesn't mean it's stuck.**/
    private int directionChangeTick;
    private int replans;
    /**How long the current route may take to drive, in ticks.**/
    private int driveTimeout;

    //Flying.
    private double cruiseY;
    private final List<Point3D> landingCandidates = new ArrayList<>();
    private int landingIndex;
    /**Checks where an aircraft can land, and where it's landing, facing the yaw.**/
    private PoseChecker landingChecker;
    private Point3D landingSpot;
    private double landingYaw;
    /**True if the aircraft can turn to its landing heading as it comes down, rather than having to finish turning first.**/
    private boolean landingTurnsOnWayDown;
    /**True while the descent is held just above the ground for the aircraft to finish turning.**/
    private boolean descentHeld;
    private double lastDescentHeight = Double.MAX_VALUE;
    /**How long the summon flight has been flying the aircraft in, in ticks, and whether it's run out of time.**/
    private int flightTicks;
    private boolean flightTimedOut;
    private int settleTicks;

    VehicleSummonJob(AWrapperWorld world, IWrapperPlayer player, UUID vehicleID, String name) {
        this.world = world;
        this.playerID = player.getID();
        this.vehicleID = vehicleID;
        this.name = name;
    }

    /**Stops the call.  Whatever is underway is left safe: stopped on the ground, or hovering.**/
    void cancel() {
        if (cancelled) {
            return;
        }
        cancelled = true;
        if (phase == Phase.TELEPORT_SPAWN) {
            //Mid-teleport: the vehicle only exists as saved data right now, so put it back first.
            finishTeleport(null);
            return;
        }
        stopVehicle();
        releaseChunk();
    }

    /**
     * Runs one tick, spending at most about tickTime nanoseconds searching.  Driving and flying still
     * happen if it's 0.  Returns true when the job is finished and can be dropped.
     */
    boolean tick(long tickTime) {
        this.tickTime = tickTime;
        if (cancelled) {
            return true;
        }
        IWrapperPlayer player = (IWrapperPlayer) world.getExternalEntity(playerID);
        if (player == null) {
            //Player logged off or changed dimension.
            cancel();
            return true;
        }
        if (forcedChunk != null && world.getTime() % CHUNK_LOAD_INTERVAL == 0) {
            world.keepChunkLoaded(forcedChunk);
        }
        if (phase != Phase.LOCATE && phase != Phase.TELEPORT_SPAWN && (vehicle == null || !vehicle.isValid)) {
            message(player, LanguageSystem.INTERACT_PHONE_LOSTCONTACT);
            releaseChunk();
            return true;
        }
        ++phaseTicks;
        if (isFlying()) {
            if (VehiclePhone.hasPlayerRider(vehicle)) {
                //A player has got in mid-flight.  Hand it over to them.
                VehiclePhone.releaseHold(vehicle);
                stopControls();
                return true;
            }
            if (!flightTimedOut && ++flightTicks > FLIGHT_TIMEOUT) {
                //Taking too long, likely blocked.  Hold where it is while we find somewhere on the ground for it.
                flightTimedOut = true;
                VehicleSummonFlight.flyTo(vehicle, vehicle.position.copy(), FLY_SPEED);
                frontAngle = getLookAngle(player);
                startSpotSearch(player.getPosition(), SpotPurpose.FLIGHT_TIMEOUT);
                return false;
            }
        }
        switch (phase) {
            case LOCATE:
                return locate(player);
            case FIND_SPOT:
                return findSpot(player);
            case PLAN_GROUND:
                return planGround(player);
            case REPLAN:
                return replan(player);
            case TELEPORT_SPAWN:
                return finishTeleport(player);
            case DRIVE:
                return drive(player);
            case CLIMB:
            case FLY_TO_PLAYER:
            case FLY_TO_LANDING:
            case DESCEND:
                return fly(player);
            case FIND_LANDING:
                return findLanding(player);
        }
        return true;
    }

    /**True while the summon flight is flying the aircraft in, or holding it while we find it somewhere to go after timing out.**/
    private boolean isFlying() {
        return phase == Phase.CLIMB || phase == Phase.FLY_TO_PLAYER || phase == Phase.FIND_LANDING || phase == Phase.FLY_TO_LANDING || phase == Phase.DESCEND || (phase == Phase.FIND_SPOT && spotPurpose == SpotPurpose.FLIGHT_TIMEOUT);
    }

    private void setPhase(Phase phase) {
        this.phase = phase;
        this.phaseTicks = 0;
    }

    //-----FINDING THE VEHICLE-----

    private boolean locate(IWrapperPlayer player) {
        vehicle = getLoadedVehicle();
        if (vehicle == null) {
            if (forcedChunk == null) {
                Point3D lastPosition = VehicleLocationTracker.getLastPosition(world, vehicleID);
                if (lastPosition == null) {
                    message(player, LanguageSystem.INTERACT_PHONE_NOTFOUND);
                    return true;
                }
                forcedChunk = lastPosition;
                world.keepChunkLoaded(forcedChunk);
                message(player, LanguageSystem.INTERACT_PHONE_CALLING);
            } else if (phaseTicks > LOAD_TIMEOUT) {
                //It's not where it was last seen, so it's been picked up or destroyed.  Stop looking for it there.
                message(player, LanguageSystem.INTERACT_PHONE_NOANSWER);
                VehicleLocationTracker.forget(world, vehicleID);
                releaseChunk();
                return true;
            }
            return false;
        }

        //Found it.  If it's only just loaded, wait for it to tick so its collision boxes are in place.
        if (vehicle.ticksExisted < 2) {
            if (phaseTicks > LOAD_TIMEOUT) {
                message(player, LanguageSystem.INTERACT_PHONE_NOANSWER);
                releaseChunk();
                return true;
            }
            return false;
        }

        //Check the caller may still call it.  Its owner may have given it a key since it was registered on this phone.
        if (!VehiclePhone.canControl(player, vehicle)) {
            message(player, LanguageSystem.INTERACT_PHONE_NEEDKEYCALL);
            releaseChunk();
            return true;
        }

        //Check it can come.
        for (APart part : vehicle.allParts) {
            if (part.rider != null) {
                message(player, LanguageSystem.INTERACT_PHONE_INUSE);
                releaseChunk();
                return true;
            }
        }
        //It has to be able to get here under its own power: not broken, with a working engine, and fuel if that needs it.
        boolean isAircraft = vehicle.definition.motorized.isAircraft;
        boolean flyIn = isAircraft && VehiclePhone.canFlyIn(vehicle);
        LanguageEntry notReady = null;
        if (vehicle.outOfHealth) {
            notReady = LanguageSystem.INTERACT_PHONE_WRECKED;
        } else if (getWorkingEngines(isAircraft).isEmpty()) {
            notReady = LanguageSystem.INTERACT_PHONE_NOENGINE;
        } else if (needsFuel(isAircraft) && vehicle.fuelTank.getFluidLevel() == 0) {
            notReady = LanguageSystem.INTERACT_PHONE_NOFUEL;
        }
        if (notReady != null) {
            message(player, notReady);
            releaseChunk();
            return true;
        }
        shape = VehicleShape.of(vehicle);
        reportShape(player);
        planCenter = player.getPosition().copy();
        frontAngle = getLookAngle(player);
        //Already loaded and within range: it makes its own way over, no teleporting.
        driveOnly = forcedChunk == null && !isFar(player);
        if (flyIn) {
            if (isIndoors(player)) {
                message(player, LanguageSystem.INTERACT_PHONE_INDOORS);
                releaseChunk();
                return true;
            }
            return startFlight(player);
        }
        if (isAircraft) {
            return teleportAircraftToGround(player);
        }
        return startGroundPlanning(player);
    }

    private EntityVehicleF_Physics getLoadedVehicle() {
        return VehiclePhone.getLoadedVehicle(world, vehicleID);
    }

    /**Stops keeping the vehicle's old chunk loaded.  It unloads on its own shortly after.**/
    private void releaseChunk() {
        forcedChunk = null;
    }

    private double getSummonDistance() {
        double distance = vehicle.summonDistanceVar.currentValue;
        return distance > 0 ? distance : DEFAULT_SUMMON_DISTANCE;
    }

    /**How fast the vehicle drives in, in m/s.**/
    private double getCruiseSpeed() {
        double speed = vehicle.summonSpeedVar.currentValue;
        return (speed > 0 ? speed : DEFAULT_SUMMON_SPEED) / 3.6;
    }

    /**
     * The yaw for the vehicle to pull up at with the side its summonArrivalAngle says facing the target.  Its left is
     * 90 degrees anticlockwise of its nose, so with the left side to the target the nose points 90 degrees round from
     * it, and each degree the angle goes round towards the front turns the nose a degree towards the target.
     */
    private double getArrivalYaw(Point3D from, Point3D target) {
        return yawTowards(from, target) - 90 + vehicle.summonArrivalAngleVar.currentValue;
    }

    private boolean isFar(IWrapperPlayer player) {
        Point3D playerPosition = player.getPosition();
        return Math.hypot(vehicle.position.x - playerPosition.x, vehicle.position.z - playerPosition.z) > getSummonDistance() + 8 || Math.abs(vehicle.position.y - playerPosition.y) > 30;
    }

    //-----GROUND PLANNING-----

    /**Starts planning the drive in.  Returns true if the job's over, as it's nearby but not on ground it can drive from.**/
    private boolean startGroundPlanning(IWrapperPlayer player) {
        kinematics = VehicleKinematics.of(vehicle);
        if (!startPlanner(driveOnly)) {
            park();
            message(player, LanguageSystem.INTERACT_PHONE_NOWAYFROMHERE);
            return true;
        }
        //Answer straight away and start up while the route's planned, so the wait for it feels like setting off.
        message(player, LanguageSystem.INTERACT_PHONE_ONITSWAY);
        startEngines(false);
        return false;
    }

    /**
     * Starts planning the drive in to planCenter, from where the vehicle is if fromHere is set, or else from anywhere it
     * can be placed.  Returns false, and starts nothing, if it's to set off from here but isn't on ground it can drive from.
     */
    private boolean startPlanner(boolean fromHere) {
        TerrainMap terrain = new TerrainMap(TerrainQuery.of(world));
        RoutePlanner.Settings settings = new RoutePlanner.Settings();
        settings.summonDistance = getSummonDistance();
        settings.cruiseSpeed = getCruiseSpeed();
        settings.turnRadius = getTurnRadius();
        settings.allowReverse = REVERSE_ENABLED && canReverse();
        settings.allowPivot = PIVOT_ENABLED && canPivot();
        settings.pivotRate = PIVOT_RATE;
        setArrival(settings.arrival);
        planner = new RoutePlanner(new PoseChecker(terrain, shape), kinematics, settings, planCenter, Math.toDegrees(frontAngle));
        if (fromHere) {
            int x = (int) Math.floor(vehicle.position.x);
            int z = (int) Math.floor(vehicle.position.z);
            int y = terrain.findSurface(x, z, (int) Math.floor(vehicle.position.y + shape.bottom + 0.5), 1, 2, shape.top - shape.bottom);
            if (y == TerrainMap.NO_SURFACE) {
                planner = null;
                return false;
            }
            planner.setStart(vehicle.position.x, y, vehicle.position.z, vehicle.orientation.angles.y);
        }
        setPhase(Phase.PLAN_GROUND);
        return true;
    }

    /**Sets the arrival settings to pull up the way the vehicle says: which side to the player, how far off, lined up how.**/
    private void setArrival(ArrivalPlanner.Settings settings) {
        settings.arrivalAngle = vehicle.summonArrivalAngleVar.currentValue;
        if (vehicle.summonArrivalGapVar.currentValue > 0) {
            settings.gap = vehicle.summonArrivalGapVar.currentValue;
        }
        double[] align = ArrivalPlanner.getAlignPoint(vehicle, settings.arrivalAngle);
        settings.alignX = align[0];
        settings.alignZ = align[1];
    }

    /**
     * Starts looking for somewhere by the player at the center to put the vehicle, rather than drive it, for the
     * purpose.  It's pulled up the way it would be driving in, but may go further out to find somewhere it fits.
     */
    private void startSpotSearch(Point3D center, SpotPurpose purpose) {
        ArrivalPlanner.Settings settings = new ArrivalPlanner.Settings();
        setArrival(settings);
        settings.drivesIn = false;
        settings.maxExtraGap = PLACEMENT_MAX_EXTRA_GAP;
        placementSearch = new ArrivalPlanner(new PoseChecker(new TerrainMap(TerrainQuery.of(world)), shape), settings, center, Math.toDegrees(frontAngle));
        spotPurpose = purpose;
        setPhase(Phase.FIND_SPOT);
    }

    /**Carries on the spot search, and acts on its result when it's done.**/
    private boolean findSpot(IWrapperPlayer player) {
        if (!placementSearch.step(tickTime)) {
            return false;
        }
        ArrivalPlanner.Goal spot = placementSearch.getGoals().isEmpty() ? null : placementSearch.getGoals().get(0);
        placementSearch = null;
        switch (spotPurpose) {
            case FALLBACK: {
                if (spot != null) {
                    teleportToGround(spot.getGround(), spot.yaw, null, LanguageSystem.INTERACT_PHONE_PARKEDCLOSEST);
                    return false;
                }
                break;
            }
            case AIRCRAFT: {
                if (spot != null) {
                    teleportToGround(spot.getGround(), spot.yaw, null, LanguageSystem.INTERACT_PHONE_BROUGHT);
                    return false;
                }
                break;
            }
            case FLIGHT_TIMEOUT: {
                //Stop the flight before saving the aircraft to move it, or it'd fly back to its old target once placed.
                VehicleSummonFlight.stop(vehicle);
                if (spot != null) {
                    //It'll be on the ground, so gear down.
                    set(vehicle.getOrCreateVariable(VehiclePhone.VAR_GEAR), 0);
                    teleportToGround(spot.getGround(), spot.yaw, null, LanguageSystem.INTERACT_PHONE_BROUGHT);
                    return false;
                }
                //Nowhere on the ground either.  Stay up where it is until a player gets in.
                VehiclePhone.holdInAir(vehicle);
                stopControls();
                message(player, LanguageSystem.INTERACT_PHONE_HOVERINGSTUCK);
                return true;
            }
        }
        //Nothing fits in front of the player.
        park();
        message(player, LanguageSystem.INTERACT_PHONE_NOROOM);
        releaseChunk();
        return true;
    }

    /**
     * Returns the radius of the tightest circle the vehicle can drive round at {@link #TURN_SPEED},
     * plus {@link #TURN_RADIUS_MARGIN}, or infinity if it can't steer.
     */
    private double getTurnRadius() {
        return TURN_RADIUS_MARGIN * VehicleKinematics.of(vehicle).getTurnRadius(TURN_SPEED);
    }

    /**
     * How far round from in front of the player an offset from them is: 0 ahead, 1 ahead to one side,
     * 2 to the side, 3 behind to one side, and 4 behind.
     */
    private int getSector(double dx, double dz) {
        return (int) Math.round(getAngleOffFront(dx, dz) / 45);
    }

    /**How far round from straight ahead of the player an offset from them is, in degrees from 0 to 180.**/
    private double getAngleOffFront(double dx, double dz) {
        return Math.abs(VehicleSummonFlight.clampAngle(Math.toDegrees(Math.atan2(dx, dz) - frontAngle)));
    }

    /**The horizontal direction the player is looking, as an angle for sin/cos (x, z).**/
    private static double getLookAngle(IWrapperPlayer player) {
        Point3D look = player.getLineOfSight(1).copy();
        return Math.atan2(look.x, look.z);
    }

    private boolean planGround(IWrapperPlayer player) {
        if (!planner.step(tickTime)) {
            return false;
        }
        reportPlanTimings(player);
        reportGoal(player);
        if (!planner.getGoals().isEmpty()) {
            fallbackGoal = planner.getGoals().get(0);
        }
        SummonRoute found = planner.getRoute();
        if (found == null) {
            return groundFallback(player);
        }
        //It already said it's on its way when planning started.
        startRoute(player, found, null);
        return false;
    }

    /**
     * Tells the player what shape planning thinks the vehicle has, if summon debugging is on, and logs every box of it.
     * Everything that keeps the vehicle off blocks and the player goes by this shape.
     */
    private void reportShape(IWrapperPlayer player) {
        if (VehicleSummonDebug.isEnabled()) {
            for (String line : SummonPlanReport.getShape(vehicle, shape)) {
                player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, line);
            }
            for (String line : SummonPlanReport.getShapeBoxes(vehicle, shape)) {
                InterfaceManager.coreInterface.logError(line);
            }
        }
    }

    /**Tells the player where the vehicle's to pull up, and how near the player that is, if summon debugging is on.**/
    private void reportGoal(IWrapperPlayer player) {
        if (VehicleSummonDebug.isEnabled()) {
            List<ArrivalPlanner.Goal> goals = planner.getGoals();
            if (goals.isEmpty()) {
                player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, String.format("Summon goal: none fits by the player, even pulling up %.2f away", planner.getArrival().getGap()));
                return;
            }
            ArrivalPlanner.Goal goal = planner.getRoute() != null ? planner.getRoute().goal : goals.get(0);
            for (String line : SummonPlanReport.getGoal(goal, planner.getArrival(), shape, planCenter, planner.getLookYaw(), goals.size())) {
                player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, line);
            }
        }
    }

    /**Tells the player how long planning took, stage by stage, if summon debugging is on.**/
    private void reportPlanTimings(IWrapperPlayer player) {
        if (VehicleSummonDebug.isEnabled()) {
            player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, "Summon plan: " + planner.getTimings());
        }
    }

    /**No route to the player.**/
    private boolean groundFallback(IWrapperPlayer player) {
        boolean noRoom = planner.getGoals().isEmpty();
        planner = null;
        if (driveOnly) {
            //Close by, so no teleporting.
            park();
            message(player, noRoom ? LanguageSystem.INTERACT_PHONE_NOROOM : LanguageSystem.INTERACT_PHONE_NOWAYFROMHERE);
            return true;
        }
        if (fallbackGoal != null) {
            //There's room by the player, we just can't drive there.  Bring it straight to them.
            teleportToGround(fallbackGoal.getGround(), fallbackGoal.yaw, null, LanguageSystem.INTERACT_PHONE_NOWAYTELEPORT);
            return false;
        }
        //No room right by the player.  Look further out for anywhere it fits.
        startSpotSearch(player.getPosition(), SpotPurpose.FALLBACK);
        return false;
    }

    /**
     * Sets off along the route, placing the vehicle at its start first if that's where it starts from.  The message
     * is sent as it sets off, unless it's null.
     */
    private void startRoute(IWrapperPlayer player, SummonRoute newRoute, LanguageEntry startMessage) {
        route = newRoute;
        planRouteSpeeds();
        planMarkers = getPlanMarkers();
        planner = null;
        if (route.placed) {
            SummonRoute.Pose start = route.getStart();
            teleport(new Point3D(start.x, start.y + shape.groundOffset(), start.z), start.yaw, spawnSpeed, Phase.DRIVE, startMessage);
        } else {
            if (startMessage != null) {
                message(player, startMessage);
            }
            beginDriving();
        }
    }

    /**
     * Plans the speed to be doing along the route: see {@link SpeedProfile}.  A vehicle that's placed to drive in sets
     * off at the speed planned for there, if it's placed moving, and one already on its way at the speed it's doing.
     */
    private void planRouteSpeeds() {
        SpeedProfile.Settings settings = new SpeedProfile.Settings();
        settings.cruiseSpeed = getCruiseSpeed();
        settings.steeringMargin = TURN_RADIUS_MARGIN;
        settings.braking = BRAKING;
        settings.minSpeed = MIN_SPEED;
        double startSpeed;
        if (route.placed) {
            startSpeed = SPAWN_MOVING ? Double.MAX_VALUE : 0;
        } else {
            startSpeed = getSpeed();
        }
        profile = SpeedProfile.plan(route, kinematics, settings, startSpeed);
        spawnSpeed = route.placed && SPAWN_MOVING ? profile.speeds[0] : 0;
    }

    /**How fast the vehicle's going, in m/s.**/
    private double getSpeed() {
        return vehicle.velocity * vehicle.speedFactor * 20;
    }

    /**
     * Markers showing the plan, if summon debugging is on: the route, where it sets off and pulls up, the other places
     * it could have pulled up, and the places the planner aimed to set off from.  Null if debugging is off.
     */
    private List<VehicleSummonDebug.Marker> getPlanMarkers() {
        if (!VehicleSummonDebug.isEnabled()) {
            return null;
        }
        List<VehicleSummonDebug.Marker> markers = VehicleSummonDebug.markers(route.getPoints(), VehicleSummonDebug.Kind.PATH);
        markers.add(new VehicleSummonDebug.Marker(route.getStart().getGround(), VehicleSummonDebug.Kind.SPAWN));
        markers.add(new VehicleSummonDebug.Marker(route.getEnd().getGround(), VehicleSummonDebug.Kind.GOAL));
        markers.addAll(SummonPlanReport.getShapeMarkers(shape, route.goal));
        for (ArrivalPlanner.Goal goal : planner.getGoals()) {
            if (goal != route.goal) {
                markers.add(new VehicleSummonDebug.Marker(goal.getGround(), VehicleSummonDebug.Kind.CANDIDATE));
            }
        }
        if (planner.getSpawnTargets() != null) {
            for (SpawnSelector.Target target : planner.getSpawnTargets()) {
                markers.add(new VehicleSummonDebug.Marker(new Point3D(target.x, target.y, target.z), VehicleSummonDebug.Kind.TARGET));
            }
        }
        return markers;
    }

    //-----TELEPORTING-----

    private void teleportToGround(Point3D groundSpot, double yaw, Phase next, LanguageEntry doneMessage) {
        teleport(new Point3D(groundSpot.x, groundSpot.y + shape.groundOffset(), groundSpot.z), yaw, 0, next, doneMessage);
    }

    private void teleport(Point3D position, double yaw, Phase next, LanguageEntry doneMessage) {
        teleport(position, yaw, 0, next, doneMessage);
    }

    /**
     * Moves the vehicle by saving it, removing it, and spawning it again from that save at the
     * new spot next tick.  Clients only see vehicle movement through MTS's movement deltas, so
     * just changing the position on the server would leave them with the vehicle in the old place.
     * The vehicle keeps its phone ID, so the phone still knows it, and anything else it saves, like its engines
     * running.  It's placed moving forwards at the speed, in m/s.
     */
    private void teleport(Point3D position, double yaw, double speed, Phase next, LanguageEntry doneMessage) {
        vehicle.disconnectAllConnections();
        IWrapperNBT data = vehicle.save(InterfaceManager.coreInterface.getNewNBTWrapper());
        data.setPoint3d("position", position);
        data.setPoint3d("angles", new Point3D(0, yaw, 0));
        //Motion is per tick, before the speed factor MTS scales it by.
        double motion = speed / 20D / vehicle.speedFactor;
        data.setPoint3d("motion", new Point3D(Math.sin(Math.toRadians(yaw)) * motion, 0, Math.cos(Math.toRadians(yaw)) * motion));
        data.setPoint3d("serverDeltaM", new Point3D());
        data.setPoint3d("serverDeltaR", new Point3D());
        data.setDouble("serverDeltaP", 0);
        teleportData = data;
        teleportItem = (ItemVehicle) vehicle.cachedItem;
        afterTeleport = next;
        afterTeleportMessage = doneMessage;
        vehicle.remove();
        vehicle = null;
        setPhase(Phase.TELEPORT_SPAWN);
    }

    private boolean finishTeleport(IWrapperPlayer player) {
        vehicle = new EntityVehicleF_Physics(world, null, teleportItem, teleportData);
        vehicle.addPartsPostAddition(null, teleportData);
        world.spawnEntity(vehicle);
        VehicleLocationTracker.update(vehicle);
        releaseChunk();
        teleportData = null;
        if (cancelled) {
            stopVehicle();
            return true;
        }
        if (player != null && afterTeleportMessage != null) {
            message(player, afterTeleportMessage);
        }
        if (afterTeleport == null) {
            park();
            return true;
        }
        setPhase(afterTeleport);
        if (afterTeleport == Phase.DRIVE) {
            beginDriving();
        }
        return false;
    }

    //-----DRIVING-----

    private void beginDriving() {
        setPhase(Phase.DRIVE);
        startEngines(false);
        set(vehicle.parkingBrakeVar, 0);
        lastProgressPosition = vehicle.position.copy();
        directionChangeTick = 0;
        driveTimeout = getDriveTimeout();
        PathTracker.Settings settings = new PathTracker.Settings();
        settings.braking = BRAKING;
        tracker = new PathTracker(route, profile, kinematics, settings);
        tracker.setRudder(vehicle.rudderInputVar.currentValue);
        driveReport = VehicleSummonDebug.isEnabled() ? new SummonDriveReport(route, profile, getCruiseSpeed(), profile.speeds[0]) : null;
    }

    /**
     * How long the vehicle may take to drive the current route, in ticks: {@link #MIN_DRIVE_TIMEOUT}, or longer for
     * routes that need more time than that at {@link #TURN_SPEED}, plus {@link #DRIVE_TIMEOUT_MARGIN}.
     * So it's 30 seconds for any route up to 60 blocks, and a long winding route still gets time to finish.  Each
     * change of direction gets {@link #DIRECTION_CHANGE_SECONDS} more, and turning on the spot the time it takes.
     */
    private int getDriveTimeout() {
        int timeout = Math.max(MIN_DRIVE_TIMEOUT, (int) Math.ceil((route.getLength() / TURN_SPEED + DRIVE_TIMEOUT_MARGIN) * 20));
        timeout += (int) Math.ceil(route.getPivotDegrees() / PIVOT_RATE * 20);
        return timeout + (int) Math.ceil(route.getCusps().size() * DIRECTION_CHANGE_SECONDS * 20);
    }

    private boolean drive(IWrapperPlayer player) {
        for (APart part : vehicle.allParts) {
            if (part.rider != null) {
                //Someone got in.  Hand it over.
                stopControls();
                return true;
            }
        }
        if (planMarkers != null && (phaseTicks == DEBUG_SEND_DELAY || phaseTicks % DEBUG_RESEND_INTERVAL == 0)) {
            VehicleSummonDebug.send(vehicle, planMarkers);
        }

        Point3D position = vehicle.position;
        double speed = getSpeed();
        PathTracker.Command command = tracker.update(position.x, position.z, vehicle.orientation.angles.y, speed);
        if (driveReport != null) {
            driveReport.record(command, speed);
        }
        boolean stalledAtEnd = command.remaining < STALLED_ARRIVE_DISTANCE && speed < STALLED_SPEED && phaseTicks > 20;
        boolean playerInTheWay = !command.arrived && !stalledAtEnd && !command.pivot && isPlayerInTheWay(player, command.reverse);
        if (command.arrived || stalledAtEnd || playerInTheWay) {
            reportDrive(player, command.arrived ? "arrived" : stalledAtEnd ? "stalled short" : "player in the way", command, speed);
            park();
            message(player, LanguageSystem.INTERACT_PHONE_ARRIVED);
            return true;
        }
        if (command.changedDirection) {
            directionChangeTick = phaseTicks;
            lastProgressPosition = position.copy();
        }

        if (command.pivot) {
            return pivot(player, position, command, speed);
        }
        //Get into gear for the way it's going.  If it's in gear the other way, stop and shift first, held straight.
        if (shiftFor(command.reverse, speed)) {
            tracker.setRudder(0);
            set(vehicle.rudderInputVar, 0);
            set(vehicle.throttleVar, 0);
            set(vehicle.brakeVar, 1);
            return checkProgress(player, position);
        }
        //Only steer once MTS sees it going the way it's meant to, or it'd turn the wrong way.
        if (vehicle.goingInReverse != command.reverse && speed >= STEER_SETTLE_SPEED) {
            tracker.setRudder(0);
            set(vehicle.rudderInputVar, 0);
        } else {
            set(vehicle.rudderInputVar, command.rudder);
        }

        //Keep to the planned speed, which slows for turns and steps, and brakes to a stop at the end.
        double targetSpeed = command.targetSpeed;
        if (speed < targetSpeed) {
            set(vehicle.throttleVar, Math.max(0, Math.min(0.8, 0.25 + (targetSpeed - speed) * 0.15)));
            set(vehicle.brakeVar, 0);
        } else if (speed > targetSpeed + 1) {
            set(vehicle.throttleVar, 0);
            set(vehicle.brakeVar, Math.min(1, (speed - targetSpeed) * 0.25));
        } else {
            set(vehicle.throttleVar, targetSpeed > 0.5 ? 0.15 : 0);
            set(vehicle.brakeVar, 0);
        }
        return checkProgress(player, position);
    }

    /**
     * Turns the vehicle on the spot as the command says.  MTS only skid steers a vehicle that's all but stopped, with
     * an engine running in neutral and the brakes off, so it stops first, held straight, then shifts to neutral and
     * lets go of the brakes, and the steering turns it.  Once it's lined up, the next way it goes shifts back into
     * gear, from neutral without stopping.  Turning doesn't move it, so it's not counted as stuck meanwhile.
     */
    private boolean pivot(IWrapperPlayer player, Point3D position, PathTracker.Command command, double speed) {
        directionChangeTick = phaseTicks;
        set(vehicle.throttleVar, 0);
        if (speed > SHIFT_SPEED) {
            set(vehicle.rudderInputVar, 0);
            set(vehicle.brakeVar, 1);
            return checkProgress(player, position);
        }
        set(vehicle.brakeVar, 0);
        set(vehicle.parkingBrakeVar, 0);
        for (PartEngine engine : getWorkingEngines(false)) {
            if (engine.running && engine.currentGearVar.currentValue != 0) {
                engine.shiftNeutral();
            }
        }
        set(vehicle.rudderInputVar, command.rudder);
        return checkProgress(player, position);
    }

    /**
     * Shifts the vehicle's engines towards the gear for going forward, or backing up if reverse is set.  Returns true
     * while it's still shifting between forward and reverse: it has to stop first, and is held still meanwhile.  Engines
     * in neutral, as when they've just started, are put in gear without stopping.
     */
    private boolean shiftFor(boolean reverse, double speed) {
        boolean shifting = false;
        for (PartEngine engine : getWorkingEngines(false)) {
            if (!engine.running) {
                continue;
            }
            double gear = engine.currentGearVar.currentValue;
            if (reverse ? gear < 0 : gear > 0) {
                continue;
            }
            //In gear the wrong way, or in neutral on the way between, as it only shifts a gear a tick.
            if (gear != 0 || reverse) {
                shifting = true;
            }
            if (gear == 0 || speed < SHIFT_SPEED) {
                if (reverse) {
                    engine.shiftDown();
                } else {
                    engine.shiftUp();
                }
            }
        }
        return shifting;
    }

    /**
     * Checks the vehicle's getting somewhere, and returns true if the job's over as it's stuck or taken too long.
     * Stopping to change direction doesn't count as stuck.
     */
    private boolean checkProgress(IWrapperPlayer player, Point3D position) {
        if (phaseTicks % STUCK_CHECK_INTERVAL == 0) {
            boolean stuck = phaseTicks - directionChangeTick >= STUCK_CHECK_INTERVAL && lastProgressPosition.isDistanceToCloserThan(position, 1.5);
            lastProgressPosition = position.copy();
            if (stuck || phaseTicks > driveTimeout) {
                return onStuck(player);
            }
        }
        return false;
    }

    /**
     * True if the player is right in front of the vehicle, or right behind it if it's backing up, where it's about to
     * drive into them.  Routes keep off the player, but they may have moved since it was planned.  Only just ahead
     * counts: a vehicle pulling up alongside them passes its nose close by, and stopping then would leave it short.
     */
    private boolean isPlayerInTheWay(IWrapperPlayer player, boolean reverse) {
        Point3D local = player.getPosition().copy().subtract(vehicle.position).reOrigin(vehicle.orientation);
        boolean atHeight = local.y > shape.bottom - 2 && local.y < shape.top;
        boolean inLine = reverse ? local.z < 0 && -local.z < shape.back + STOP_FRONT_DISTANCE : local.z > 0 && local.z < shape.front + STOP_FRONT_DISTANCE;
        return atHeight && inLine && Math.abs(local.x) < shape.side + STOP_SIDE_MARGIN;
    }

    /**
     * Tells the player how the drive went, if summon debugging is on: where it stopped against where it should have,
     * for the reason, and how it drove on the way.  A drive that got stuck has no stop to tell of.
     */
    private void reportDrive(IWrapperPlayer player, String reason, PathTracker.Command command, double speed) {
        if (driveReport == null) {
            return;
        }
        if (command != null) {
            double gap = vehicle.summonArrivalGapVar.currentValue > 0 ? vehicle.summonArrivalGapVar.currentValue : new ArrivalPlanner.Settings().gap;
            player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, driveReport.getStop(reason, command, vehicle, shape, player.getPosition(), vehicle.summonArrivalAngleVar.currentValue, gap, speed));
        } else {
            player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, "Summon stopped: " + reason);
        }
        player.displayChatMessage(LanguageSystem.SYSTEM_DEBUG, driveReport.getDrive(replans));
        driveReport = null;
    }

    private boolean onStuck(IWrapperPlayer player) {
        reportDrive(player, phaseTicks > driveTimeout ? "took too long" : "stuck", null, getSpeed());
        stopControls();
        if (replans == 0 && phaseTicks <= driveTimeout) {
            ++replans;
            //Plan again from where it is, to wherever the player is now.
            planCenter = player.getPosition().copy();
            frontAngle = getLookAngle(player);
            if (startPlanner(true)) {
                setPhase(Phase.REPLAN);
                return false;
            }
        }
        return stuckFallback(player);
    }

    private boolean replan(IWrapperPlayer player) {
        if (!planner.step(tickTime)) {
            return false;
        }
        reportPlanTimings(player);
        SummonRoute found = planner.getRoute();
        if (found == null || found.poses.size() < 2) {
            planner = null;
            return stuckFallback(player);
        }
        startRoute(player, found, null);
        return false;
    }

    private boolean stuckFallback(IWrapperPlayer player) {
        if (fallbackGoal != null && !driveOnly) {
            teleportToGround(fallbackGoal.getGround(), fallbackGoal.yaw, null, LanguageSystem.INTERACT_PHONE_STUCKTELEPORT);
            return false;
        }
        park();
        message(player, LanguageSystem.INTERACT_PHONE_STUCK);
        return true;
    }

    //-----FLYING-----

    /**
     * For aircraft that don't fly in (see {@link VehiclePhone#canFlyIn}).  Puts it straight on the ground near the player,
     * far enough out that none of it lands on them.
     */
    private boolean teleportAircraftToGround(IWrapperPlayer player) {
        startSpotSearch(player.getPosition(), SpotPurpose.AIRCRAFT);
        return false;
    }

    private boolean startFlight(IWrapperPlayer player) {
        Point3D playerPosition = player.getPosition();
        landingChecker = new PoseChecker(new TerrainMap(TerrainQuery.of(world)), shape);
        cruiseY = getCruiseHeight(playerPosition);
        startEngines(true);
        //A new call takes over from any earlier one that left it in the sky.
        set(vehicle.getOrCreateVariable(VehiclePhone.VAR_FLY_HELD), 0);
        set(vehicle.getOrCreateVariable(VehiclePhone.VAR_GEAR), 1);
        set(vehicle.parkingBrakeVar, 0);
        set(vehicle.throttleVar, 0);
        if (!driveOnly) {
            //Arrive from in front of the player.
            double distance = getSummonDistance();
            Point3D spawn = new Point3D(playerPosition.x + Math.sin(frontAngle) * distance, cruiseY, playerPosition.z + Math.cos(frontAngle) * distance);
            //Make sure the arrival point clears what's under it too.
            spawn.y = Math.max(cruiseY, getCruiseHeight(spawn));
            cruiseY = spawn.y;
            teleport(spawn, yawTowards(spawn, playerPosition), Phase.FLY_TO_PLAYER, LanguageSystem.INTERACT_PHONE_ONITSWAY);
        } else {
            message(player, LanguageSystem.INTERACT_PHONE_ONITSWAY);
            VehicleSummonFlight.flyTo(vehicle, new Point3D(vehicle.position.x, cruiseY, vehicle.position.z), FLY_SPEED * 0.5);
            setPhase(Phase.CLIMB);
        }
        return false;
    }

    private boolean fly(IWrapperPlayer player) {
        Point3D position = vehicle.position;
        switch (phase) {
            case CLIMB: {
                if (Math.abs(position.y - cruiseY) < 1) {
                    setPhase(Phase.FLY_TO_PLAYER);
                }
                break;
            }
            case FLY_TO_PLAYER: {
                //Follow the player as they move.
                Point3D target = new Point3D(player.getPosition().x, cruiseY, player.getPosition().z);
                if (phaseTicks == 1 || phaseTicks % 10 == 0) {
                    VehicleSummonFlight.flyTo(vehicle, target, FLY_SPEED);
                }
                if (horizontalDistance(position, target) < 1.5 && Math.abs(position.y - cruiseY) < 1) {
                    //Overhead.  Look for somewhere to land.
                    startLandingSearch(player);
                }
                break;
            }
            case FLY_TO_LANDING: {
                if (horizontalDistance(position, landingSpot) < 0.75) {
                    //If there isn't room to turn on the way down, finish turning up here first.
                    if (!landingTurnsOnWayDown && !isFacingLandingHeading()) {
                        break;
                    }
                    set(vehicle.getOrCreateVariable(VehiclePhone.VAR_GEAR), 0);
                    setPhase(Phase.DESCEND);
                    //Until it's facing its landing heading, only come down to just above the ground and wait there.
                    descentHeld = !isFacingLandingHeading();
                    VehicleSummonFlight.flyTo(vehicle, descentHeld ? getDescentHold() : getDescentTarget(), DESCEND_SPEED);
                }
                break;
            }
            case DESCEND: {
                if (descentHeld) {
                    if (isFacingLandingHeading()) {
                        //Finished turning.  Carry on down.
                        descentHeld = false;
                        setPhase(Phase.DESCEND);
                        VehicleSummonFlight.flyTo(vehicle, getDescentTarget(), DESCEND_SPEED);
                    }
                    break;
                }
                double heightLeft = position.y - (landingSpot.y + shape.groundOffset());
                //Note MTS's isAnythingOnGround() is true when nothing is on the ground, so check the grounded set directly.
                boolean wheelsDown = !vehicle.groundDeviceCollective.groundedGroundDevices.isEmpty();
                //If we've stopped sinking, we're resting on something.  Call that landed rather than hang there.
                boolean stalled = false;
                if (phaseTicks % 40 == 0) {
                    stalled = phaseTicks > 40 && lastDescentHeight - heightLeft < 0.1;
                    lastDescentHeight = heightLeft;
                }
                //Keep pressing down for a moment after first contact, so every wheel settles onto
                //the ground instead of the first one to touch holding the rest up.
                if ((wheelsDown || stalled || heightLeft < -0.5) && settleTicks == 0) {
                    settleTicks = SETTLE_TICKS;
                }
                if (settleTicks > 0 && --settleTicks == 0) {
                    VehicleSummonFlight.stop(vehicle);
                    park();
                    message(player, LanguageSystem.INTERACT_PHONE_LANDED);
                    return true;
                }
                break;
            }
            default:
                break;
        }
        return false;
    }

    private boolean isFacingLandingHeading() {
        return VehicleSummonFlight.isFacing(vehicle, landingYaw);
    }

    /**
     * Where the descent aims: a block below where the wheels meet the ground.  The flight eases off as it
     * nears its target, so aiming at the ground itself would crawl the last bit and stop short.  The ground
     * stops us; we finish when the wheels touch.
     */
    private Point3D getDescentTarget() {
        return new Point3D(landingSpot.x, landingSpot.y + shape.groundOffset() - LANDING_OVERSHOOT, landingSpot.z);
    }

    /**Where the descent waits for the aircraft to finish turning, just above the ground.**/
    private Point3D getDescentHold() {
        return new Point3D(landingSpot.x, landingSpot.y + shape.groundOffset() + LANDING_TURN_HOLD, landingSpot.z);
    }

    private void startLandingSearch(IWrapperPlayer player) {
        setPhase(Phase.FIND_LANDING);
        landingCandidates.clear();
        Point3D center = player.getPosition();
        for (int dx = -LANDING_RADIUS; dx <= LANDING_RADIUS; dx += 2) {
            for (int dz = -LANDING_RADIUS; dz <= LANDING_RADIUS; dz += 2) {
                double distance = Math.hypot(dx, dz);
                //Not on top of the player.
                if (distance <= LANDING_RADIUS && distance > shape.radius() + 1.5) {
                    landingCandidates.add(new Point3D(Math.floor(center.x) + dx + 0.5, 0, Math.floor(center.z) + dz + 0.5));
                }
            }
        }
        //In front of the player first, then to the sides, then behind, and the nearest first in each.
        frontAngle = getLookAngle(player);
        landingCandidates.sort((a, b) -> {
            int sectorA = getSector(a.x - center.x, a.z - center.z);
            int sectorB = getSector(b.x - center.x, b.z - center.z);
            return sectorA != sectorB ? Integer.compare(sectorA, sectorB) : Double.compare(horizontalDistance(a, center), horizontalDistance(b, center));
        });
        landingIndex = 0;
    }

    private boolean findLanding(IWrapperPlayer player) {
        long startTime = System.nanoTime();
        while (System.nanoTime() - startTime < tickTime) {
            if (landingIndex >= landingCandidates.size()) {
                //Nowhere to land.  Stay overhead until a player gets in.
                VehiclePhone.holdInAir(vehicle);
                stopControls();
                message(player, LanguageSystem.INTERACT_PHONE_HOVERING);
                return true;
            }
            Point3D spot = landingCandidates.get(landingIndex++);
            int x = (int) Math.floor(spot.x);
            int z = (int) Math.floor(spot.z);
            int groundY = getLandingGround(x, z);
            if (groundY == TerrainMap.NO_SURFACE) {
                continue;
            }
            //Land with the side its arrival angle says to the player if it fits, or else turned whichever way it does.
            double[] yaw = landingChecker.findLandingYaw(spot.x, groundY, spot.z, getArrivalYaw(spot, player.getPosition()), vehicle.orientation.angles.y, cruiseY);
            if (yaw != null) {
                landingSpot = new Point3D(spot.x, groundY, spot.z);
                landingYaw = yaw[0];
                landingTurnsOnWayDown = yaw[1] == 1;
                //Start turning now, smoothly, as it flies over to the spot.
                VehicleSummonFlight.turnTo(vehicle, landingYaw);
                VehicleSummonFlight.flyTo(vehicle, new Point3D(spot.x, cruiseY, spot.z), FLY_SPEED * 0.4);
                setPhase(Phase.FLY_TO_LANDING);
                return false;
            }
        }
        return false;
    }

    /**
     * Returns the height of the ground under the cruising aircraft in this column, or
     * {@link TerrainMap#NO_SURFACE} if it's liquid or something it can't stand on.
     */
    private int getLandingGround(int x, int z) {
        for (int y = (int) Math.floor(cruiseY); y > cruiseY - 128; --y) {
            Point3D block = new Point3D(x + 0.5, y + 0.5, z + 0.5);
            if (!world.isAir(block)) {
                return world.isBlockLiquid(block) || !world.isBlockSolid(block, Axis.UP) ? TerrainMap.NO_SURFACE : y + 1;
            }
        }
        return TerrainMap.NO_SURFACE;
    }

    /**Height for the aircraft origin that clears everything around the position and the player.**/
    private double getCruiseHeight(Point3D around) {
        double highest = around.y;
        //Cover everything the aircraft could be over while hovering here, at any heading.
        int range = 12 + (int) Math.ceil(shape.radius());
        for (int dx = -range; dx <= range; dx += 4) {
            for (int dz = -range; dz <= range; dz += 4) {
                highest = Math.max(highest, getColumnTop((int) Math.floor(around.x) + dx, (int) Math.floor(around.z) + dz, around.y));
            }
        }
        return highest + CRUISE_CLEARANCE - shape.bottom;
    }

    private double getColumnTop(int x, int z, double nearY) {
        int top = (int) Math.min(world.getMaxHeight() - 1, nearY + 128);
        for (int y = top; y > nearY - 32; --y) {
            if (!world.isAir(new Point3D(x + 0.5, y + 0.5, z + 0.5))) {
                return y + 1;
            }
        }
        return nearY;
    }

    private boolean isIndoors(IWrapperPlayer player) {
        Point3D position = player.getPosition();
        int top = (int) Math.min(world.getMaxHeight() - 1, position.y + 256);
        for (int y = (int) Math.floor(position.y) + 2; y <= top; ++y) {
            if (!world.isAir(new Point3D(position.x, y + 0.5, position.z))) {
                return true;
            }
        }
        return false;
    }

    //-----CONTROLS-----

    private void startEngines(boolean includeJets) {
        for (PartEngine engine : getWorkingEngines(includeJets)) {
            if (!engine.running) {
                if (engine.definition.engine.type == EngineType.MAGIC) {
                    //Magic engines start themselves once the magneto is on.  We can't use the
                    //auto-starter, as it refuses to start anything when the fuel tank is empty.
                    engine.magnetoVar.setTo(1, true);
                } else {
                    engine.autoStartEngine();
                    InterfaceManager.packetInterface.sendToAllClients(new PacketPartEngine(engine, Signal.AS_ON));
                }
            }
        }
    }

    /**
     * The engines that aren't broken and that we'd use to bring the vehicle in.  Ground vehicles drive in,
     * so their jets don't count.
     */
    private List<PartEngine> getWorkingEngines(boolean includeJets) {
        List<PartEngine> engines = new ArrayList<>();
        for (PartEngine engine : vehicle.engines) {
            if ((includeJets || !isJet(engine)) && !engine.outOfHealth) {
                engines.add(engine);
            }
        }
        return engines;
    }

    /**
     * True if any working engine we'd start burns fuel from the tank.  Magic engines don't, so a vehicle
     * powered only by them never runs out.
     */
    private boolean needsFuel(boolean includeJets) {
        if (vehicle.isCreative || ConfigSystem.settings.general.fuelUsageFactor.value == 0) {
            return false;
        }
        for (PartEngine engine : getWorkingEngines(includeJets)) {
            if (engine.definition.engine.type != EngineType.MAGIC) {
                return true;
            }
        }
        return false;
    }

    /**
     * True if the vehicle can back up the way a route plans: it has an engine to drive it, every engine has a
     * reverse gear, it steers with its wheels rather than always skid steering, and it isn't towing anything, as
     * backing a trailer is another thing altogether.  Vehicles that always skid steer turn on the spot instead.
     */
    private boolean canReverse() {
        List<PartEngine> engines = getWorkingEngines(false);
        if (engines.isEmpty() || vehicle.definition.motorized.hasPermanentSkidSteer || !vehicle.towingConnections.isEmpty()) {
            return false;
        }
        for (PartEngine engine : engines) {
            if (engine.reverseGears == 0) {
                return false;
            }
        }
        return true;
    }

    /**
     * True if the vehicle can turn on the spot: it skid steers, as tracked vehicles do when stopped and mechs and
     * hovercraft always do, it has an engine to run in neutral, which MTS needs to skid steer, and it isn't towing anything.
     */
    private boolean canPivot() {
        return (vehicle.definition.motorized.hasSkidSteer || vehicle.definition.motorized.hasPermanentSkidSteer) && !getWorkingEngines(false).isEmpty() && vehicle.towingConnections.isEmpty();
    }

    private static boolean isJet(PartEngine engine) {
        return engine.definition.generic.type.contains("jet");
    }

    /**Lets go of the controls, and takes it out of reverse if it was backing up, so whoever drives it next isn't caught out.**/
    private void stopControls() {
        set(vehicle.throttleVar, 0);
        set(vehicle.rudderInputVar, 0);
        set(vehicle.brakeVar, 0);
        for (PartEngine engine : getWorkingEngines(false)) {
            if (engine.currentGearVar.currentValue < 0) {
                engine.shiftNeutral();
            }
        }
    }

    /**Stops and holds the vehicle where it is.  Engines are left running.**/
    private void park() {
        stopControls();
        set(vehicle.parkingBrakeVar, 1);
    }

    /**Makes whatever the vehicle is doing safe after a cancel.**/
    private void stopVehicle() {
        if (vehicle == null || !vehicle.isValid) {
            return;
        }
        if (vehicle.getOrCreateVariable(VehicleSummonFlight.VAR_ACTIVE).isActive) {
            //Airborne.  Stay up where it is until a player gets in.
            VehiclePhone.holdInAir(vehicle);
            stopControls();
        } else {
            park();
        }
    }

    /**Sets a control and syncs it to clients, only if it changed enough to matter.**/
    private static void set(ComputedVariable variable, double value) {
        if (Math.abs(variable.currentValue - value) > 0.005) {
            variable.setTo(value, true);
        }
    }

    //-----HELPERS-----

    private static double horizontalDistance(Point3D a, Point3D b) {
        return Math.hypot(a.x - b.x, a.z - b.z);
    }

    private static double yawTowards(Point3D from, Point3D to) {
        return Math.toDegrees(Math.atan2(to.x - from.x, to.z - from.z));
    }

    private void message(IWrapperPlayer player, LanguageEntry language) {
        VehiclePhone.message(player, language, name);
    }
}
