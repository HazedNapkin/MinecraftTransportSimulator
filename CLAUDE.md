# Vehicle summoning

This file covers the vehicle summoning system on this branch: what it is, how it fits together, and the rules for changing it. The open work is listed in [SUMMONING_PLAN.md](SUMMONING_PLAN.md).

## What it does

A player registers vehicles on a phone item (sneak-click the vehicle), then calls one from the phone's screen. The vehicle comes to them under its own power:

- **Ground vehicles** are placed out of sight about `summonDistance` blocks away (default 30), then drive in with the real throttle, brake, steering and gears, and pull up beside the player with the side their `summonArrivalAngle` says facing them. A vehicle that is already loaded and nearby drives over from where it is and is never teleported.
- **Aircraft with `summonFlyIn`** are placed above the buildings, fly over the player and land straight down in the nearest clear spot. Other aircraft are placed on the ground near the player.
- **Fallbacks** when the nice way is not possible: no route means teleport to where it would have pulled up; no room by the player means the nearest spot further out; stuck means one replan, then teleport; nowhere to land means hover overhead until a player gets in.

## Goals, in priority order

1. **Reliable.** A call always ends in a safe, known state: parked with the parking brake on, hovering held, or handed to a rider. It never loses, duplicates or strands a vehicle, never leaves controls applied with no job watching, and never hurts the player.
2. **Works as expected.** The vehicle ends up where the player is, facing the way the pack asked, and every outcome tells the player what happened in chat.
3. **Cinematic.** It should look driven by a person: appears out of sight, rolls in already moving, smooth steering, slows before turns, brakes gently to an exact stop. Teleporting in view is the last resort.
4. **Cheap.** All of this runs on the server thread. Planning is time-sliced and must stay within its budget; world lookups, allocations and packets are all counted and should only go down.

When goals conflict, the earlier one wins. A cheaper search that sometimes fails to stop, or a prettier route that clips a wall, is a regression.

## Where the code is

Everything is in `mccore/src/main/java/minecrafttransportsimulator/systems/summoning/`, with tests in the same package under `mccore/src/test/java/`.

| Stage | Class | Role |
|---|---|---|
| Entry | `VehiclePhone`, `VehicleSummonManager` | Phone entries and permission checks. The manager owns one job per vehicle, ticks them each server tick and shares out the search time (10 ms a tick across all calls). |
| Job | `VehicleSummonJob` | The state machine for one call: locate, plan, teleport, drive, fly, land, fall back. The only class that touches the real vehicle. |
| Finding it | `VehicleLocationTracker` | Last saved position of every vehicle per world, so an unloaded one can be found and its chunk loaded. |
| World | `TerrainQuery`, `TerrainMap` | The only way planning asks about blocks. `TerrainMap` caches answers for one plan. |
| Vehicle | `VehicleShape`, `VehicleKinematics`, `PoseChecker` | The space the vehicle takes up, how it steers (copied from MTS's own steering maths), and whether it fits at a pose. |
| Planning | `RoutePlanner` | Runs the stages below a time slice at a time. |
| 1 | `ArrivalPlanner` | Where it pulls up: exact goal poses beside the player, each with a straight run-in. |
| 2 | `FlowField` | Flood out from the goals: drive distance from every nearby cell. Guides the search. |
| 3 | `SpawnSelector` | Hidden places about the summon distance out to set off from. |
| 4 | `HybridSearch`, `AnalyticPaths` | Hybrid A* forward from the starts, joining the run-in exactly with Dubins or Reeds-Shepp curves. Reversing and turning on the spot only when they save a lot. |
| 5 | `PathSmoother`, `SummonRoute` | Eases the steering between arcs, re-checking fit. The route is poses half a block apart, each with its curvature. |
| Driving | `SpeedProfile`, `PathTracker`, `DriveControls` | Speeds along the route, steering and target speed each tick, and what that means for pedals and gears. `DriveControls` learns the vehicle's brakes and coasting as it goes. |
| Flying | `VehicleSummonFlight` | Moves an aircraft to a target, on both sides, from synced variables. |
| Debug | `VehicleSummonDebug`, `SummonPlanReport`, `SummonDriveReport` | Markers and chat reports when the `summonDebug` config is on. |

Wiring outside the package: `EntityManager` calls `VehicleSummonManager.tickWorld` and `onWorldUnload`; `AEntityVehicleD_Moving` calls `onVehicleTick` and the location tracker; `PacketVehiclePhoneAction` and `GUIVehiclePhone` are the phone screen; `JSONVehicle` holds the pack fields (`summonFlyIn`, `summonDistance`, `summonArrivalAngle`, `summonSpeed`, `summonArrivalGap`); chat messages are `INTERACT_PHONE_*` in `LanguageSystem`. Each of the six `mcinterface*` projects has its own `WrapperWorld` implementing `keepChunkLoaded`, `chunkLoaded` and `getBlockSurfaceFlags`, so a change to `AWrapperWorld` must be made in all six.

## Conventions the code relies on

- Yaw is in degrees and rises turning left; a vehicle heads along `(sin yaw, cos yaw)` in x, z. Curvature is radians per block, positive turning left. Rudder input has the opposite sign.
- Speeds are in m/s inside summoning. MTS's `velocity` is blocks per tick before `speedFactor`; convert at the edges only.
- A surface at height `y` is the top of the solid block at `y - 1`. A column can have several surfaces, so always look one up near a known height.
- The vehicle yaws about its origin, so the origin is the point that follows the route.
- Unloaded and liquid columns are blocked.

## Testing

Run the summoning tests from the repo root (about three minutes):

```bash
./gradlew :mccore:test --tests 'minecrafttransportsimulator.systems.summoning.*' -i
```

Use the package name, not `'*summoning*'`: on Windows the Java launcher expands wildcards itself, so that pattern matches `SUMMONING_PLAN.md` in the repo root and Gradle finds no tests.

- **`SummonInvariantsTest`** is what summoning may never do: a route the vehicle cannot drive, one that touches the player, or a plan that differs with how its time was sliced. Never weaken it to make a change pass.
- **`SummonScorecardTest`** scores every vehicle at every place plus 30 made-up places against `mccore/src/test/resources/summoning/scorecard-baseline.tsv`, and fails if a guarded metric gets worse. Each run writes `mccore/build/summon-scorecard.tsv`. Lines starting `SCORE` in the output show totals, improvements and regressions.
- **`SummonSweepTest`** prints one `SWEEP` line per summon with timings and search counts, for tuning.
- **`TraceDriveMain`** prints one drive tick by tick, for working out why it went as it did.
- `SummonScenarios` holds the shared vehicles and places. Add one there and every test picks it up.

Rules for the baseline:

- Copy the new scorecard over the baseline only when the change is meant to move those numbers, and say in the commit message what moved and by how much.
- A regression in one metric is acceptable only when it is the stated price of a larger gain, and the trade is written down. Never update the baseline just to make the test pass.
- When you fix a bug, add the place or stop that shows it to `SummonScenarios` or the scorecard first, so the fix is visible in the numbers.

What the tests do not cover: `VehicleSummonJob` itself, teleporting, flight and landing, and anything that depends on real MTS physics. `SimVehicle` is a stand-in whose numbers are not taken from MTS. Changes in those areas need an in-game check, which you cannot do yourself: say plainly what was not verified and give the user the steps to check it.

## Guidelines for changes

### Reliability

- Every way out of `VehicleSummonJob.tick` must leave the vehicle parked, held in the air, or with a rider. When adding a phase or an early return, trace cancel, player logout, vehicle unload and world unload through it.
- Anything set on the vehicle for the duration of a call (controls, simple throttle, flight variables, forced chunk) needs a matching restore on every exit, and a guard for when the job disappears without running it.
- Teleporting is remove now, spawn next tick. Nothing may be done between that could lose the saved data.
- Planning must give the same route however its time is sliced. No decisions based on wall-clock time, only on work done.

### Expected behaviour

- Every outcome sends the player a message. A new outcome needs a new `INTERACT_PHONE_*` entry, not a reused one that is nearly right.
- A vehicle that was already nearby is never teleported (`driveOnly`).
- Respect the pack's fields. Derive behaviour from the vehicle (shape, kinematics, engines, gears) and not from its name or type.

### Cinematic

- Prefer what a driver would do: slow early, steer smoothly, stop once. Avoid stop-start, pedal flicking and steering jumps; `pedalSwitches`, `steeringTravel` and `stray` on the scorecard measure these.
- Keep the vehicle out of sight when it appears where the terrain allows (`hidden`).
- A fallback teleport should be the result of a failed attempt, not a shortcut.

### Cost

- Never do unbounded work in one tick. Anything that searches takes a time budget and can be resumed.
- Ask the world only through `TerrainQuery`, and cache within a plan. `worldCalls` and `raycasts` are guarded metrics.
- Allocation matters as much as time on a server. Avoid boxing and per-query objects in the planning loops; watch `planMb`.
- Control changes are packets to every client. Send only when a value changes by a meaningful step (`PEDAL_STEP`, `RUDDER_STEP`, the `set` helper); `controlsSent` is guarded.
- Driving and flying ticks must stay trivial: no searching, no world scans every tick.

### Style

- Keep new logic in small classes that know nothing of the real vehicle or world, as `DriveControls` and `TerrainQuery` do, so it can be tested with fakes. Keep `VehicleSummonJob` to orchestration.
- Tuning numbers are named constants with a comment saying what they are, their unit, and why that value.
- Comments are plain English in the existing `/**...**/` style and explain why. Commit messages are one plain sentence about what changed for the player, with the scorecard movement in the body.
- One change per commit, with the tests passing. Commit only when the user asks.
- Minecraft 1.12.2 through 1.21.1 are all supported from one `mccore`, which targets Java 8. Do not use newer language features or APIs there.
