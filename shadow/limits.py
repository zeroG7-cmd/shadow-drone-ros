#!/usr/bin/env python3
"""
Shared safety limits, checked every tick by Skill.limits() for any
skill that moves the drone.

SIM_LIMITS['geofence_m'] is a PLACEHOLDER. 2.0m is a guess, not a
measurement of the actual living_room.world dimensions - replace this
with the real room size before trusting it for anything. Until then,
treat a limits() failure near this boundary as "the geofence is
probably wrong", not "Shadow is definitely about to hit something".
"""

SIM_LIMITS = {
    'geofence_m': 2.0,       # PLACEHOLDER - real living_room.world dimensions not yet measured
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
