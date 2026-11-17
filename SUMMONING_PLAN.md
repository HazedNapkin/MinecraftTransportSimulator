# Vehicle summoning: improvement plan

This is a work plan for a model continuing the vehicle summoning system on the `summon2` branch. Read [CLAUDE.md](CLAUDE.md) first: it explains the system, the goals and the rules for changing it. The goals, in priority order, are **reliable, works as expected, cinematic, cheap**.

## How to work through this

- Do the phases in order. Within a phase, do the steps in order unless one says otherwise. Each step is one commit.
- Before starting, run the summoning tests and confirm they pass against the committed baseline:

  ```bash
  ./gradlew :mccore:test --tests '*summoning*' -i
  ```

- Each step lists where to look, what to change, how to verify and when it is done. The file and line references were correct at commit `ee0782a7d`; re-find them by name if the code has moved.
- Several findings come from reading the code and were never reproduced in game. Steps that rest on one say **Confirm first**. If the finding turns out to be wrong, skip the step and tell the user why.
- You cannot run Minecraft. Where a step needs an in-game check, finish the code and tests, then give the user the exact steps to check it and say plainly that it is unverified until they do.
- Steps marked **Ask the user** need a decision that is theirs. Ask before writing code for that step, and carry on with the next independent step meanwhile.
- Follow the baseline rules in CLAUDE.md. Report the scorecard movement of every step.

## Starting point

At commit `ee0782a7d` the scorecard reads: 37 of 41 named summons find a route (the bus, tank and truck do not fit in the passage, and the bus does not fit in the forest), 60 of 60 made-up ones do, and every route found is driven to a stop. Clipping is 3 ticks in total, worst stray 0.32 blocks, worst stop error 0.21 blocks for normal vehicles. Planning takes 4 to 154 ms and allocates about 65 MB a plan on average. The outlier is a car with weak brakes, which overshoots by about 4.9 blocks.

---

## Phase 1: Safety holes in the job

Small, independent fixes in `VehicleSummonJob` and `VehicleSummonManager`. None should move the scorecard.

### 1.1 Park a ground vehicle whose job has gone

**Problem.** While driving, the job holds throttle on, brakes and parking brake off, and simple throttle on. If the vehicle unloads mid-drive (the player teleports away) or the server stops without a clean unload, `tick()` ends the job with "lost contact" and nothing resets the controls. `throttle`, `brake`, `p_brake` and `simplethrottle` are all saved with the vehicle, so it would load again later with the throttle applied and nobody driving. Aircraft already have a guard for the equivalent case in `VehicleSummonManager.onVehicleTick`; ground vehicles do not.

**Confirm first.** Check that these variables really are restored from saved data (`new ComputedVariable(this, "throttle", data)` in `AEntityVehicleE_Powered`) and that nothing in MTS already brakes an unoccupied vehicle on load.

**Change.**
- Add a saved, synced variable on the vehicle, for example `phone_drive`, set in `beginDriving()` and cleared in `stopControls()`. Follow how `VehicleSummonFlight.VAR_ACTIVE` and `VehiclePhone.VAR_FLY_HELD` are used.
- The drive also turns simple throttle on and remembers the old value only in the job (`simpleThrottleBefore`). Store the old value on the vehicle too, so the guard can restore it.
- In `onVehicleTick`, in the existing once-a-second server block, if `phone_drive` is set and `jobs` has no job for this vehicle's phone ID, zero the throttle, rudder and brake, set the parking brake, restore simple throttle, take any engine out of reverse, and clear the variable.
- Keep the per-tick cost at one variable read: do not add work outside the once-a-second block.

**Verify.** Unit-test whatever logic you can pull out of the entity. In-game check for the user: summon a car from far away, and while it is driving in, teleport several hundred blocks away, then return. The car should be parked with the parking brake on.

**Done when** no path leaves the drive variable set without a job, and the scorecard is unchanged.

### 1.2 Check for riders again before every teleport

**Problem.** Riders are checked in `locate()` and each tick of `drive()`. Planning takes up to a second or so in between, and `teleport()` removes the vehicle and respawns it from saved data without checking again. The same is true of the teleports from `findSpot()`.

**Change.** Add one helper that says whether any part has a rider, and call it at the top of `teleport()`'s callers or inside `teleport()` itself. If someone is aboard, stop the controls, send `INTERACT_PHONE_INUSE`, release the chunk and end the job. Make sure the aircraft timeout path, which already hands over to a player rider, still does.

**Done when** every call to `teleport()` is preceded by the check.

### 1.3 Decide what happens to trailers

**Problem.** `teleport()` calls `vehicle.disconnectAllConnections()`, so a vehicle summoned from afar silently leaves its trailer behind. A vehicle that is itself being towed can also be summoned.

**Ask the user** which they want:
1. Refuse to summon a vehicle that would be teleported while towing or being towed, with a new message (recommended: smallest, and nothing is lost silently).
2. Disconnect as now, but tell the player.
3. Bring the trailer along. This is a much larger piece of work and should be its own plan.

A nearby vehicle that drives over without teleporting can keep its trailer; `canReverse()` and `canPivot()` already stop it backing up or turning on the spot while towing.

**Change, for option 1.** In `locate()`, after `driveOnly` is known, end the job with a new `INTERACT_PHONE_*` message if the vehicle is not `driveOnly` and has `towingConnections` or a `towedByConnection`. Also refuse a towed vehicle in the `driveOnly` case. Add the language entry in `LanguageSystem` next to the others.

### 1.4 Review every exit from the job

Read `tick()` and each phase method once more with this question: on cancel, player logout, vehicle unload and world unload, is the vehicle parked, held, or handed over, and is the forced chunk released? Fix any gap found and list what you checked in the commit message. Known places to look: the "lost contact" return, `onWorldUnload` during `TELEPORT_SPAWN`, and `cancel()` during `FIND_SPOT` for a flight that timed out.

---

## Phase 2: Make the job testable

**Problem.** `VehicleSummonJob` is about 1,400 lines covering the state machine, teleporting, driving glue and flight, and has no tests, because it works directly on `EntityVehicleF_Physics`, `AWrapperWorld` and `IWrapperPlayer`. Phases 3 and 6 change its behaviour and need tests to be safe.

**Change.**
- Introduce a narrow interface for what the job needs from the vehicle (position, yaw, speed, riders, engines and gears, control setters, save and respawn, validity) and one for the world and player (player position and look, chunk loading, messages), in the way `TerrainQuery` already separates planning from the world. Provide the real implementations as thin adapters.
- Move the job onto those interfaces without changing behaviour. Do it in small commits: ground first, flight after.
- If the file is still unwieldy, split flight into its own class behind the same job interface.
- Write a fake vehicle and world for tests. `SimVehicle` and `FakeTerrain` are the starting points.
- Add tests for the paths that matter most: a normal placed drive to arrival, a nearby vehicle driving over, no route leading to a teleport, no room leading to the spot search, stuck then replan then teleport, cancel in each phase, player gone in each phase, vehicle gone in each phase, and a rider getting in.

**Verify.** The scorecard and invariants are unchanged. In-game check for the user: one ground summon from afar, one nearby, one fly-in aircraft.

**Done when** the paths above are tested and Phase 1's fixes each have a test.

---

## Phase 3: Do what the player expects while driving

### 3.1 Follow a player who moves

**Problem.** The route ends where the player stood when planning started (`planCenter`). If they walk on, the vehicle parks where they were. Aircraft already follow the player.

**Change.**
- While driving, every half second or so, compare the player's position with `planCenter`. If they have moved more than a threshold (start with the larger of 4 blocks and the arrival gap) and the vehicle still has a worthwhile distance to go, plan again from where the vehicle is to where the player is now.
- Keep driving the current route while the new one is planned, and switch over when it is ready, starting the new speed profile from the current speed. Do not stop to think: that is the cinematic requirement. `startPlanner(true)` and `startRoute()` already support planning from the vehicle's position; the stuck path stops the controls first, which this must not.
- Limit how often it can happen (for example one replan in flight at a time, and a minimum interval), so a player running in circles cannot keep the planner busy.
- If the new plan finds nothing, carry on with the old route.

**Verify.** Tests in the Phase 2 harness: player steps a short way (no replan), player walks 15 blocks (replan, arrives at the new spot), player keeps moving (bounded replans). Add a scorecard scenario so the cost shows.

### 3.2 Give way to things in the road

**Problem.** `isPlayerInTheWay()` only looks at the player who called. Other players, mobs and vehicles are driven into.

**Change.**
- Every few ticks, not every tick, ask the world for entities in a box ahead of the vehicle along its direction of travel, sized from `VehicleShape` and the current stopping distance (`DriveControls.getStoppingDistance`). `AWrapperWorld.getEntitiesWithin(BoundingBox)` exists; find how other MTS vehicles are looked up.
- If something is there, brake to a stop and hold, without ending the job. Set off again when it has gone. Count the wait as not stuck for a limited time, then fall through to the existing stuck handling.
- Keep the caller's special case as it is: arriving beside them is the goal.

**Verify.** Harness tests with a fake entity crossing and one that stays. Check `controlsSent` does not rise in the normal case.

### 3.3 Notice blocks that changed

**Problem.** A block placed on the route after planning is only noticed by the five-second stuck check.

**Change.** Every second or so, re-check the next few blocks of the route ahead with `PoseChecker.canDrive` against a fresh `TerrainMap`, bounded to a fixed number of poses. If one no longer fits, replan as in 3.1. Measure the world calls this adds and keep it small.

---

## Phase 4: Stop in the right place with weak brakes

**Problem.** `SpeedProfile` plans to brake at 2.5 m/s². `DriveControls` learns the real braking, but only once it first brakes, so a vehicle with weaker brakes than planned is already too fast when it finds out. The `stop/weakbrakes` scorecard rows overshoot by about 4.9 blocks at 4.6 m/s.

**Change, in order of preference.** Try each against the scorecard and keep what helps without slowing normal vehicles.
1. Remember what `DriveControls` learnt (braking and coasting) per vehicle definition in a small in-memory cache, and seed the next drive and its `SpeedProfile` braking from it. The first summon of a vehicle type is as now; later ones are right from the start.
2. For the first drive, use the natural slow-downs: the speed profile already brakes before turns, so make sure learning from light braking there feeds the final stop. Check `LEARN_BRAKE` and the weighting.
3. Only if those are not enough, estimate braking from the vehicle before the drive. Look at `getBrakingForce()` in `AEntityVehicleD_Moving` and work out whether an estimate in m/s² is cheap and accurate.

Do not add a visible test dab of the brakes: it looks wrong.

**Verify.** Add a scorecard stop that drives the weak-brakes car a second time with what the first drive learnt. `stopError` and `parkSpeed` for weak brakes should fall well under a block; `seconds` for the other stops should not rise by more than the scorecard's slack.

---

## Phase 5: Cost

### 5.1 Cut allocation in planning

**Problem.** Plans allocate about 65 MB each on average and far more in cluttered places. On a server that is garbage collector pressure on the main thread.

**Confirm first.** Profile before changing anything: find which allocations dominate, with a profiler or by temporarily counting. The likely sources, from reading the code, are:
- `TerrainMap`: `HashMap<Long, Integer>` and `HashMap<Long, Map<Integer, Boolean>>` caches box every key and value.
- `PoseChecker.drivableColumns`: the same nested boxed shape.
- `VehicleShape.getFootprint` and `getSupports`: a new list per pose checked.
- `HybridSearch`: a `Node`, an `Entry` and several `double[]` per move, and `HashMap<Long, Node>`.
- `TerrainQuery.of`: a new `Point3D` and `BoundingBox` for every world question.

**Change.** Take the biggest first and measure after each.
- Replace the boxed caches with primitive open-addressing maps written for the purpose (no new dependency; `mccore` targets Java 8), or with small per-chunk-column arrays.
- Reuse mutable `Point3D` and `BoundingBox` instances in the world adapter. Planning is single threaded.
- Reuse footprint storage between calls, or cache footprints by yaw bin where the result is identical.

**Verify.** The invariants must still pass, and routes found, `expansions` and `worldCalls` must be identical, since this changes no decisions. Report `planMb` and the `SWEEP` timings before and after. Once `planMb` is much lower and stable, consider guarding it with generous slack.

### 5.2 Let servers set the budget

**Change.** Move `SEARCH_TIME_PER_TICK` (10 ms a server tick, shared by all calls) into the config next to `summonDebug` in `JSONConfigSettings`, with a sensible minimum. Update the comment in `VehicleSummonManager` that says a route takes about 200 ms to plan: the sweep now shows 4 to 154 ms.

### 5.3 Keep the driving tick trivial

After Phases 3 and 4, re-read `drive()` and confirm nothing new runs every tick that could run every few ticks, and that `controlsSent` on the scorecard has not grown.

---

## Phase 6: Terrain and vehicle coverage

Each of these changes what counts as drivable, so each needs new places in `SummonScenarios` and an in-game check.

### 6.1 Surfaces that are not full blocks

**Problem.** A surface needs the block below to have a sturdy top face (`getBlockSurfaceFlags` uses `isFaceSturdy(UP)`). Bottom slabs over air are therefore not surfaces, and dirt paths and farmland probably are not either, as they are slightly short of a full block. Roads in Minecraft builds are often made of exactly these.

**Confirm first.** Ask the user to try summoning along a path-block road and across a slab bridge with `summonDebug` on.

**Change.** If confirmed, extend what a surface is. The planner works in whole-block heights, so the simplest route is to treat a block whose collision top is within a small distance of a full block as solid-topped, and to treat a bottom slab as a surface half a block up only if the height model can carry it. `getBlockSurfaceFlags` is implemented separately in all six `mcinterface*` `WrapperWorld` classes; change them all. `FakeTerrain` will need to represent partial blocks to test it.

### 6.2 Climbing ability from the vehicle

**Problem.** `maxUp = 1` and `maxDown = 2` are fixed in `RoutePlanner.Settings` for every vehicle.

**Change.** Find how MTS decides whether a ground device climbs a step, and derive `maxUp` from the vehicle if it can be done cheaply. A vehicle that cannot climb a full block should plan with `maxUp = 0`. If MTS has no clear rule, leave this and say so.

### 6.3 Vehicles that float

**Problem.** Liquid columns are always blocked, and nothing in summoning knows about boats. A boat called to the shore would most likely be teleported onto land.

**Ask the user** whether boats and amphibious vehicles should be supported now. If yes, the smallest useful version is to recognise a vehicle that floats, treat water as its surface and land as blocked, and pull up at the nearest water to the player. This is a sizeable change; plan it separately before starting.

---

## Phase 7: Flight

Flight has no tests until Phase 2 covers it. Do this phase after Phase 2.

### 7.1 Clear the whole flight path

**Problem.** `getCruiseHeight()` samples columns every 4 blocks within about 12 blocks of the player and of the spawn point only. A thin tower between samples, or anything in the gap between the two areas when `summonDistance` is large, is not seen. The aircraft then pushes against it until the 90-second `FLIGHT_TIMEOUT`.

**Change.** Sample the corridor along the straight line from the spawn point to the player, as wide as the aircraft, at a stride no larger than the aircraft's width, and raise `cruiseY` to clear it. Spread the work over ticks if it is more than a few hundred lookups. Do the same when flying on to the landing spot.

### 7.2 Notice a blocked flight sooner

**Change.** Add a progress check like the ground drive's: if the aircraft has not got closer to its target for a few seconds, climb and try again once, then fall back, instead of waiting out the full timeout.

### 7.3 Indoors check

**Problem.** `isIndoors()` treats any non-air block above the player as a roof, so standing under a tree refuses a fly-in.

**Change.** Count only blocks that would stop the aircraft, using a collision check, and consider requiring more than a single block. Keep it conservative: landing on a roof the player is under is worse than refusing.

---

## Phase 8: Server controls

**Ask the user** which of these they want and what the defaults should be. Add each as a config entry beside `summonDebug`, checked in `VehicleSummonManager.handleAction`, with a chat message when it refuses:
- a switch to turn summoning off;
- a cooldown per player between calls;
- a maximum distance a vehicle may be summoned from, with 0 meaning unlimited.

---

## Phase 9: Calibrate the simulator

**Problem.** `SimVehicle`'s numbers are stand-ins, so the scorecard measures the controls against invented vehicles.

**Change.** Ask the user to drive a few vehicles from the official pack in the repo (a car, a truck or bus, a tracked vehicle) with `summonDebug` on and send the `SummonDriveReport` output. Set `SimVehicle.Spec` values to match, add them as named vehicles in `SummonScenarios`, and record the new baseline. From then on, a mismatch between the scorecard and what the user sees in game is a simulator bug worth fixing.

---

## Not in this plan

- Bringing trailers along on a teleport (see 1.3).
- Summoning across dimensions. The location tracker is per world and the job cancels if the player is not in the vehicle's world.
- Saving vehicle locations more often than on world unload. Periodic saving was removed on purpose in commit `375b29354`; raise it with the user before changing it.
