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

## Tier 3 - locked

Nothing defined yet. This is where a Mind-layer planner (see
`shadow/mind/__init__.py`) would eventually sequence Tier 1/2 skills
into an actual mission (find the payload, go to it, hover, land) - not
built until Tier 1 and Tier 2 have real passing streaks in sim.

## Known placeholder, not yet real

`shadow/limits.py`'s `SIM_LIMITS['geofence_m']` is a guessed 2.0m, not
a measurement of `worlds/living_room.world`'s actual dimensions. Treat
any limits() failure near that boundary with that in mind until it's
replaced with a real number.
