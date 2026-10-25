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
"""

SIM_LIMITS = {
    'geofence_m': 13.0,      # measured against shadow_runway.sdf - see docstring above
    'max_altitude_m': 3.0,
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
