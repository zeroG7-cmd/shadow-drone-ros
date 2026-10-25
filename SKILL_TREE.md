# Shadow's skill tree

Tracks what's unlocked, in the order each skill can actually be
attempted. Unlock rule: 3 streak passes of a skill in sim unlocks it.
A single failure resets that skill's streak to 0 - no partial credit.
Progress lives in `progress.json`, committed to git so the Pi inherits
the same unlocks sim already earned; nothing is unlocked on real
hardware that hasn't first been proven in sim.

## Tier 1 - no prerequisites

- `arm` - switch to GUIDED, arm. Mirrors `mode guided` / `arm throttle`.
- `takeoff` - arm (if needed), climb to a target altitude via
  ArduCopter's CommandTOL service. Mirrors `takeoff 5`.
- `hover` - hold position for a fixed duration. Proves setpoint
  streaming actually holds still before anything depends on it.

## Tier 2 - requires arm, takeoff, and hover each unlocked

- `go_to` - fly to a specific (x, y, z) point. Gated behind Tier 1
  because a positioning mistake here is the first skill that could fly
  Shadow into a wall.
- `land` - switch to LAND, wait for ArduCopter's own auto-disarm on
  touchdown. Fixes the old mission_planner.py bug where landing was
  never actually confirmed.

## Tier 3 - requires Tier 1 and Tier 2 each unlocked

- `deliver_payload` - the first real mission, added Oct 24, 2026 once
  Shadow moved outdoors onto `worlds/shadow_runway.sdf`. Takes off,
  sweeps the open area in a fixed boustrophedon pattern with NO
  pre-known payload coordinates (true visual search, not known-
  waypoints - Zer0's explicit pick), locks onto whichever orange
  payload the live camera feed actually finds via closed-loop visual
  servo on `/shadow/target_position`, "picks it up" by despawning it
  in place (`gz world/remove` - no gripper exists yet), flies it to
  the fixed delivery pad, "drops it off" by respawning an orange box
  there (`gz world/create`), and lands. One payload per run, by
  explicit choice over collecting both oranges. Lives in
  `shadow/skills/deliver_payload.py`; run via
  `ros2 run shadow_drone run_skill.py deliver_payload`.

This is also where a Mind-layer planner (see `shadow/mind/__init__.py`)
would eventually sequence multiple Tier 1/2/3 skills together into a
longer mission - not built yet, and `deliver_payload` itself doesn't
use it; it's a single self-contained Skill with its own internal phase
state machine (TAKEOFF -> SEARCH -> APPROACH -> DESPAWN -> TRANSIT ->
RESPAWN -> LAND), not a sequence of other Skill objects.

## Known placeholder, not yet real

`shadow/limits.py`'s `SIM_LIMITS['geofence_m']` was a guessed 2.0m
sized for the old indoor `living_room.world`. As of Oct 24, 2026 it's
been widened to a measured 13.0m against `worlds/shadow_runway.sdf`'s
actual payload/pad/sweep layout - see that file's own docstring for
the derivation. `deliver_payload`'s own lock-on area threshold
(`LOCK_AREA_PX` in `shadow/skills/deliver_payload.py`) is the next
placeholder in line: it's a plausible guess, not a calibrated
distance, and should be tuned from the first live run's logs.
