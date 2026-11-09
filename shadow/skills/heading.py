#!/usr/bin/env python3
"""
NoseFirst - makes a movement skill fly nose-first (facing where it is
going) instead of keeping its spawn heading and sliding sideways.
Added Nov 8, 2026.

Why it matters beyond looks: Shadow has ONE front camera. Flying sideways
points the camera away from the direction of travel, so the tracker can
miss props the drone flies past during a search sweep.

Heading convention matches ArduPilotBody.current_yaw() and the yaw
argument of send_position_setpoint(): ENU radians, 0 = facing +x (east),
counter-clockwise positive. So the heading towards a target is simply
atan2(dy, dx).

Two styles:
  turn_first=True  (go_to)       - on a big heading change, hold position
                                   and rotate on the spot, then fly. Like a
                                   camera drone lining up a shot.
  turn_first=False (follow_path) - keep moving and turn while flying, so
                                   corners don't become stop-turn-go.

Near the target (inside MIN_DIST_M) the heading is frozen, so small
position errors on arrival can't swing the nose around.
"""

import math

MIN_DIST_M = 1.0        # closer than this: stop re-aiming, keep the last heading
TURN_START_DEG = 30.0   # turn_first: stop and rotate if the nose is off by more than this...
TURN_DONE_DEG = 10.0    # ...and only move off again once within this (hysteresis)


def wrap(angle):
    """Wrap to (-pi, pi] so 350 deg vs 10 deg reads as 20 deg apart, not 340."""
    return math.atan2(math.sin(angle), math.cos(angle))


class NoseFirst:
    def __init__(self, body, turn_first=True):
        self.body = body
        self.turn_first = turn_first
        self.yaw = None          # heading being commanded
        self.turning = False     # turn_first: currently rotating on the spot
        self.hold = None         # turn_first: where to hold while rotating

    def setpoint(self, target):
        """Given the (x, y, z) this skill wants to reach, return the
        (x, y, z, yaw) to actually send this tick."""
        tx, ty, tz = target
        pos = self.body.current_position()
        cur = self.body.current_yaw()
        if pos is None:
            return tx, ty, tz, self.yaw if self.yaw is not None else cur

        dist = math.hypot(tx - pos[0], ty - pos[1])
        if dist > MIN_DIST_M:
            self.yaw = math.atan2(ty - pos[1], tx - pos[0])
        elif self.yaw is None:
            self.yaw = cur  # arrived already, nothing to aim at: keep the current heading

        if not self.turn_first or cur is None or self.yaw is None:
            return tx, ty, tz, self.yaw

        err = abs(wrap(self.yaw - cur))
        if not self.turning and dist > MIN_DIST_M and err > math.radians(TURN_START_DEG):
            self.turning = True
            self.hold = (pos[0], pos[1])
        elif self.turning and err < math.radians(TURN_DONE_DEG):
            self.turning = False
            self.hold = None

        if self.turning:
            # Rotate on the spot (still climbing/descending to the target height).
            return self.hold[0], self.hold[1], tz, self.yaw
        return tx, ty, tz, self.yaw
