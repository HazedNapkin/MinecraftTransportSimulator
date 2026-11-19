package minecrafttransportsimulator.systems.summoning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.EnumSet;
import java.util.Set;
import java.util.UUID;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.junit.Test;

import minecrafttransportsimulator.systems.LanguageSystem;
import minecrafttransportsimulator.systems.LanguageSystem.LanguageEntry;
import minecrafttransportsimulator.systems.navigation.FakeTerrain;
import minecrafttransportsimulator.systems.summoning.VehicleSummonJob.Phase;

/**
 * Runs {@link VehicleSummonJob} for ground vehicles in a {@link FakeSummonWorld}, through each way a call can go and
 * each way it can be cut off, and checks it always ends safe: the vehicle there exactly once, parked with nothing
 * held on, and the player told what happened.
 *
 * @author HazedNapkin
 */
public class VehicleSummonJobTest {
    /**Search time a tick, in nanoseconds, as the manager gives one call on its own.**/
    private static final long BUDGET = 10_000_000;
    /**Longest any call here should take, in ticks.**/
    private static final int MAX_TICKS = 20 * 180;
    /**Far enough away to be placed rather than drive over, and near enough to drive over, in blocks.**/
    private static final double FAR = 100;
    private static final double NEAR = 15;
    /**How near the player a vehicle that's arrived should be, in blocks from its origin.**/
    private static final double ARRIVED_DISTANCE = 6;

    /**One call, in its world, run a tick at a time.**/
    private static final class Call {
        final FakeSummonWorld world;
        final FakeSummonPlayer player;
        final UUID vehicleID;
        final VehicleSummonJob job;
        final Set<Phase> phasesSeen = EnumSet.noneOf(Phase.class);
        boolean done;

        Call(FakeSummonWorld world, FakeSummonPlayer player, FakeSummonVehicle vehicle) {
            this.world = world;
            this.player = player;
            this.vehicleID = vehicle.phoneID;
            this.job = new VehicleSummonJob(world, player.id, vehicleID, "test");
        }

        /**Runs a server tick: the job's, then the world's.**/
        void tick() {
            assertFalse("ticked after it finished", done);
            done = job.tick(BUDGET);
            if (!done) {
                phasesSeen.add(job.getPhase());
            }
            world.tick();
        }

        /**Runs ticks until the job finishes.**/
        void finish() {
            runUntil(() -> false);
            assertTrue("finished in time", done);
        }

        /**Runs ticks until the condition holds or the job finishes.  Returns true if the condition came first.**/
        boolean runUntil(BooleanSupplier condition) {
            for (int i = 0; i < MAX_TICKS && !done; ++i) {
                if (condition.getAsBoolean()) {
                    return true;
                }
                tick();
            }
            return false;
        }

        /**Runs ticks until the job reaches the phase, failing if it never does.**/
        void runTo(Phase phase) {
            assertTrue("reached " + phase, runUntil(() -> job.getPhase() == phase));
        }

        FakeSummonVehicle vehicle() {
            return world.get(vehicleID);
        }

        /**
         * Checks it's over and safe: the vehicle there once, parked, with nothing held on, and comes to rest and stays
         * there.  One cut off mid-drive stops on the parking brake from whatever speed it was doing.
         */
        void checkSafe() {
            assertEquals("vehicles", 1, world.count(vehicleID));
            FakeSummonVehicle vehicle = vehicle();
            assertTrue("parked", vehicle.isParked());
            assertFalse("still in reverse", vehicle.getGear() < 0);
            for (int i = 0; i < 100; ++i) {
                world.tick();
            }
            assertEquals("came to rest", 0, vehicle.getSpeed(), 0);
            double x = vehicle.getPosition().x;
            double z = vehicle.getPosition().z;
            for (int i = 0; i < 100; ++i) {
                world.tick();
            }
            assertEquals("moved once at rest", 0, Math.hypot(vehicle.getPosition().x - x, vehicle.getPosition().z - z), 0);
            assertTrue("still parked", vehicle.isParked());
        }

        double distanceToPlayer() {
            FakeSummonVehicle vehicle = vehicle();
            return Math.hypot(vehicle.getPosition().x - player.position.x, vehicle.getPosition().z - player.position.z);
        }
    }

    private static SummonScenarios.Vehicle car() {
        return SummonScenarios.vehicle("car");
    }

    /**Flat open ground with the player in the middle, and a car the distance away.**/
    private static Call openGround(double distance) {
        FakeSummonWorld world = new FakeSummonWorld(FakeTerrain.flat(150));
        FakeSummonPlayer player = world.addPlayer(0.5, 0, 0.5);
        FakeSummonVehicle vehicle = world.addVehicle(car(), distance + 0.5, 0.5, 0);
        return new Call(world, player, vehicle);
    }

    /**The player in a yard walled all round, with nowhere in, and a car the distance away outside.**/
    private static Call walledYard(double distance) {
        FakeTerrain terrain = FakeTerrain.flat(150);
        int size = 12;
        terrain.fill(-size, 0, -size, size, 3, -size).fill(-size, 0, size, size, 3, size).fill(-size, 0, -size, -size, 3, size).fill(size, 0, -size, size, 3, size);
        FakeSummonWorld world = new FakeSummonWorld(terrain);
        FakeSummonPlayer player = world.addPlayer(0.5, 0, 0.5);
        FakeSummonVehicle vehicle = world.addVehicle(car(), distance + 0.5, 0.5, 0);
        return new Call(world, player, vehicle);
    }

    /**The player in a narrow passage, too narrow for a car anywhere near them, open at both ends.**/
    private static Call narrowPassage() {
        FakeTerrain terrain = FakeTerrain.flat(150);
        int length = 9;
        terrain.fill(-10, 0, -length, -1, 3, length).fill(1, 0, -length, 10, 3, length);
        FakeSummonWorld world = new FakeSummonWorld(terrain);
        FakeSummonPlayer player = world.addPlayer(0.5, 0, 0.5);
        FakeSummonVehicle vehicle = world.addVehicle(car(), FAR + 0.5, 0.5, 0);
        return new Call(world, player, vehicle);
    }

    /**Runs a call until the vehicle has been driving a while, then jams it so it can't move.**/
    private static void jamWhileDriving(Call call) {
        call.runTo(Phase.DRIVE);
        for (int i = 0; i < 10; ++i) {
            call.tick();
        }
        call.vehicle().jammed = true;
    }

    private static void assertLastMessage(Call call, LanguageEntry expected) {
        assertEquals(expected.key, call.player.lastMessage() == null ? null : call.player.lastMessage().key);
    }

    //-----HOW A CALL GOES-----

    @Test
    public void farVehicleIsPlacedAndDrivesIn() {
        Call call = openGround(FAR);
        call.finish();
        assertEquals("teleports", 1, call.world.teleports);
        assertTrue("drove", call.phasesSeen.contains(Phase.DRIVE));
        assertEquals(LanguageSystem.INTERACT_PHONE_ONITSWAY.key, call.player.messages.get(0).key);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_ARRIVED);
        assertTrue("near the player, " + call.distanceToPlayer(), call.distanceToPlayer() < ARRIVED_DISTANCE);
        call.checkSafe();
    }

    @Test
    public void nearbyVehicleDrivesOverWithoutTeleporting() {
        Call call = openGround(NEAR);
        call.finish();
        assertEquals("teleports", 0, call.world.teleports);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_ARRIVED);
        assertTrue("near the player, " + call.distanceToPlayer(), call.distanceToPlayer() < ARRIVED_DISTANCE);
        call.checkSafe();
    }

    @Test
    public void unloadedVehicleIsLoadedAndComes() {
        FakeSummonWorld world = new FakeSummonWorld(FakeTerrain.flat(150));
        FakeSummonPlayer player = world.addPlayer(0.5, 0, 0.5);
        FakeSummonVehicle vehicle = world.addUnloadedVehicle(car(), FAR + 0.5, 0.5, 0);
        Call call = new Call(world, player, vehicle);
        call.finish();
        assertEquals(LanguageSystem.INTERACT_PHONE_CALLING.key, player.messages.get(0).key);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_ARRIVED);
        call.checkSafe();
    }

    @Test
    public void noRouteTeleportsBesideThePlayer() {
        Call call = walledYard(FAR);
        call.finish();
        assertEquals("teleports", 1, call.world.teleports);
        assertFalse("drove", call.phasesSeen.contains(Phase.DRIVE));
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_NOWAYTELEPORT);
        assertTrue("near the player, " + call.distanceToPlayer(), call.distanceToPlayer() < ARRIVED_DISTANCE);
        call.checkSafe();
    }

    @Test
    public void nearbyVehicleWithNoRouteStaysPut() {
        //Outside the wall, but close enough to drive over rather than be placed.
        Call call = walledYard(20);
        call.finish();
        assertEquals("teleports", 0, call.world.teleports);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_NOWAYFROMHERE);
        call.checkSafe();
    }

    @Test
    public void noRoomByThePlayerParksFurtherOut() {
        Call call = narrowPassage();
        call.finish();
        assertTrue("looked further out", call.phasesSeen.contains(Phase.FIND_SPOT));
        assertEquals("teleports", 1, call.world.teleports);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_PARKEDCLOSEST);
        call.checkSafe();
    }

    @Test
    public void stuckReplansThenTeleports() {
        Call call = openGround(FAR);
        jamWhileDriving(call);
        call.finish();
        assertTrue("replanned", call.phasesSeen.contains(Phase.REPLAN));
        assertTrue("looked for a spot", call.phasesSeen.contains(Phase.FIND_SPOT));
        assertEquals("teleports", 2, call.world.teleports);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_STUCKTELEPORT);
        assertTrue("near the player, " + call.distanceToPlayer(), call.distanceToPlayer() < ARRIVED_DISTANCE + 4);
        call.checkSafe();
    }

    @Test
    public void nearbyVehicleStuckStaysPut() {
        Call call = openGround(NEAR + 10);
        jamWhileDriving(call);
        call.finish();
        assertTrue("replanned", call.phasesSeen.contains(Phase.REPLAN));
        assertEquals("teleports", 0, call.world.teleports);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_STUCK);
        call.vehicle().jammed = false;
        call.checkSafe();
    }

    //-----CUT OFF-----

    /**Sets up a call and runs it to the phase, ready to be cut off there.**/
    private static Call callAt(Phase phase) {
        Call call;
        switch (phase) {
            case LOCATE: {
                FakeSummonWorld world = new FakeSummonWorld(FakeTerrain.flat(150));
                FakeSummonPlayer player = world.addPlayer(0.5, 0, 0.5);
                call = new Call(world, player, world.addUnloadedVehicle(car(), FAR + 0.5, 0.5, 0));
                call.tick();
                break;
            }
            case REPLAN: {
                call = openGround(FAR);
                jamWhileDriving(call);
                break;
            }
            case FIND_SPOT: {
                call = narrowPassage();
                break;
            }
            default: {
                call = openGround(FAR);
                break;
            }
        }
        call.runTo(phase);
        return call;
    }

    private static final Phase[] GROUND_PHASES = { Phase.LOCATE, Phase.PLAN_GROUND, Phase.TELEPORT_SPAWN, Phase.DRIVE, Phase.REPLAN, Phase.FIND_SPOT };

    @Test
    public void cancelInEveryPhaseLeavesItParked() {
        for (Phase phase : GROUND_PHASES) {
            Call call = callAt(phase);
            call.job.cancel();
            call.tick();
            assertTrue(phase + ": finished", call.done);
            call.vehicle().jammed = false;
            call.checkSafe();
        }
    }

    @Test
    public void playerLeavingInEveryPhaseLeavesItParked() {
        for (Phase phase : GROUND_PHASES) {
            Call call = callAt(phase);
            call.world.players.clear();
            call.tick();
            assertTrue(phase + ": finished", call.done);
            call.vehicle().jammed = false;
            call.checkSafe();
        }
    }

    @Test
    public void failingInEveryPhaseLeavesItParkedAndSaysSo() {
        for (Phase phase : GROUND_PHASES) {
            Call call = callAt(phase);
            call.job.fail();
            assertLastMessage(call, LanguageSystem.INTERACT_PHONE_FAILED);
            call.tick();
            assertTrue(phase + ": finished", call.done);
            call.vehicle().jammed = false;
            call.checkSafe();
        }
    }

    @Test
    public void vehicleGoingMidDriveLosesContact() {
        Call call = openGround(FAR);
        call.runTo(Phase.DRIVE);
        call.vehicle().valid = false;
        call.tick();
        assertTrue("finished", call.done);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_LOSTCONTACT);
    }

    //-----SOMEONE GETS IN-----

    @Test
    public void callerGettingInTakesOver() {
        Call call = openGround(FAR);
        call.runTo(Phase.DRIVE);
        int messages = call.player.messages.size();
        call.vehicle().riderID = call.player.id;
        call.tick();
        assertTrue("finished", call.done);
        assertEquals("told them anything", messages, call.player.messages.size());
        FakeSummonVehicle vehicle = call.vehicle();
        assertFalse("still driven", vehicle.isMarkedDriving());
        assertEquals(0, vehicle.throttle, 0);
        assertEquals(0, vehicle.rudder, 0);
    }

    @Test
    public void someoneElseGettingInWhilePlanningStopsTheTeleport() {
        Call call = openGround(FAR);
        call.runTo(Phase.PLAN_GROUND);
        call.vehicle().riderID = UUID.randomUUID();
        call.finish();
        assertEquals("teleports", 0, call.world.teleports);
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_INUSE);
        assertFalse("still driven", call.vehicle().isMarkedDriving());
    }

    //-----READY TO COME-----

    @Test
    public void vehicleThatCantComeSaysWhy() {
        checkRefused(vehicle -> vehicle.wrecked = true, LanguageSystem.INTERACT_PHONE_WRECKED);
        checkRefused(vehicle -> vehicle.hasEngine = false, LanguageSystem.INTERACT_PHONE_NOENGINE);
        checkRefused(vehicle -> vehicle.outOfFuel = true, LanguageSystem.INTERACT_PHONE_NOFUEL);
        checkRefused(vehicle -> vehicle.keyID = UUID.randomUUID(), LanguageSystem.INTERACT_PHONE_NEEDKEYCALL);
        checkRefused(vehicle -> vehicle.riderID = UUID.randomUUID(), LanguageSystem.INTERACT_PHONE_INUSE);
    }

    private static void checkRefused(Consumer<FakeSummonVehicle> setUp, LanguageEntry expected) {
        Call call = openGround(FAR);
        setUp.accept(call.vehicle());
        call.finish();
        assertLastMessage(call, expected);
        assertEquals("teleports", 0, call.world.teleports);
        assertEquals("vehicles", 1, call.world.count(call.vehicleID));
    }

    @Test
    public void callerWithTheKeyCanCall() {
        Call call = openGround(NEAR);
        UUID key = UUID.randomUUID();
        call.vehicle().keyID = key;
        call.player.keys.add(key);
        call.finish();
        assertLastMessage(call, LanguageSystem.INTERACT_PHONE_ARRIVED);
    }

    @Test
    public void vehicleLeftDrivenByAnEarlierCallIsParkedFirst() {
        Call call = openGround(NEAR);
        FakeSummonVehicle vehicle = call.vehicle();
        vehicle.beginDriving();
        vehicle.setParkingBrake(false);
        vehicle.setThrottle(1);
        call.tick();
        assertEquals(Phase.PLAN_GROUND, call.job.getPhase());
        assertFalse("still driven", vehicle.isMarkedDriving());
        assertEquals(0, vehicle.throttle, 0);
        assertTrue("parking brake", vehicle.hasParkingBrake());
    }
}
