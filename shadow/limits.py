#!/usr/bin/env python3
"""
Shared safety limits, checked every tick by Skill.limits() for any
skill that moves the drone.

SIM_LIMITS['geofence_m'] was a PLACEHOLDER (2.0m, a guess sized for the
old indoor living_room.world) until Oct 24, 2026, when Shadow moved
outdoors onto worlds/shadow_runway.sdf for the search-and-deliver
mission (shadow/skills/deliver_payload.py). That world is real and
measured: 5 scattered payload props up to ~9.06m from spawn (the
farthest, payload_distractor_purple at (1, 9)), plus a delivery pad at
(0, -4) and the search sweep's own corners reaching (±7, 9) - a
worst-case radius of about 11.4m from the origin. 13.0m keeps a
comfortable margin past all of that, not a guess made to merely avoid
tripping the check - any limits() failure now really does mean Shadow
is near the edge of where the world/mission is laid out, not "the
geofence number is probably wrong".

SIM_LIMITS['max_altitude_m'] raised 3.0 -> 6.0 on Zer0's call (Nov 6,
2026): 3.0m was cramped for a drone (taller than a person, and every
current skill/env already operates well under it - go_to_env.py's and
multi_target_env.py's own TRAIN_MIN_Z/TRAIN_MAX_Z top out at 2.5m,
agility_circuit_env.py's CIRCUIT_ALTITUDE_M is 1.5m, none of them
derive from this constant - so this only widens the safety ceiling
itself, it doesn't touch what any existing trained checkpoint actually
does). Confirmed no training code reads this value for anything other
than the np.clip()/within_limits() safety net. Not yet wired to
autotune_hover.py (see that script and run_autotune.sh's own
docstrings) - it still runs standalone, outside Skill.limits()
entirely, so this change doesn't by itself put a net under its climb.
If that gets wired in later, note TAKEOFF_ALTITUDE_M there is also
6.0 - exactly at this ceiling, zero margin - worth raising one of the
two numbers at that point, not before.
"""

SIM_LIMITS = {
    'geofence_m': 13.0,      # measured against shadow_runway.sdf - see docstring above
    'max_altitude_m': 6.0,   # was 3.0 until Nov 6, 2026 - see docstring above
    'min_altitude_m': -0.2,  # small negative tolerance for ground-level noise
}


def within_limits(position, limits=SIM_LIMITS):
    """position is (x, y, z) in local ENU, same frame as everything else
    in this package. Returns False the instant anything is outside
    bounds - callers should treat that as an immediate abort, not a
    warning."""
    x, y, z = position
    geofence = limits['geofence_m']
    if abs(x) > geofence or abs(y) > geofence:
        return False
    if z > limits['max_altitude_m'] or z < limits['min_altitude_m']:
        return False
    return True
