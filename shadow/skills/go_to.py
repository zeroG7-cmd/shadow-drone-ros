#!/usr/bin/env python3
"""
GoToSkill - fly to a specific (x, y, z) point in MAVROS's local ENU
frame, relative to the arm point - or, with no point given, ping-pong
between two fixed anchor points.

Tier 2 - gated behind arm/takeoff/hover each having their own proven
streak, not because GoTo is technically harder, but because a
positioning mistake here is the first skill that could actually fly
Shadow into a wall if the earlier ones haven't proven setpoints behave
the way they're assumed to.

Fixed Oct 3, 2026 (first pass): the original version took a fixed
absolute target, which trivially re-passes on every run after the
first once the drone is already sitting there.

Fixed Oct 3, 2026 (second pass, same day): the first fix replaced that
with "+1m in x from wherever Shadow currently is," computed fresh each
run. That solves the trivial-repeat-pass problem but creates a worse
one - it never comes back, so every PASS walks the drone another 1m
further out. Zer0 ran it three times in a row from the repo root (no
directory mistake this time) and watched it fail on run 3 with
"limits() failed" - which tracks exactly: run 1 ends near x=1, run 2
near x=2 (right at the geofence edge), run 3's target of x=3 breaches
shadow/limits.py's 2.0m geofence mid-flight. Unbounded drift was always
going to hit that wall eventually; it just took three runs.

Now the no-target default ping-pongs between two fixed anchor points
(1m apart, well inside the 2.0m geofence) instead of drifting: each run
flies to whichever anchor Shadow is currently farther from. That still
guarantees genuine movement on every single run - it's never already
sitting at the anchor it's about to be told to fly to - but it's
permanently bounded, so a training streak of any length can't walk
itself into the geofence the way a monotonic offset always eventually
will. Passing an explicit x/y/z still flies to that literal absolute
point, unaffected by any of this.
"""

from .base import Skill

POSITION_TOLERANCE_M = 0.3

# Two fixed points to ping-pong between, 1m apart in x, at the default
# takeoff altitude. ANCHOR_A sits roughly above the arm point;
# ANCHOR_B is 1m forward of it. Both are comfortably inside
# shadow/limits.py's 2.0m geofence, so no number of repeat PASSes can
# ever drift this skill into an abort the way the unbounded relative
# offset did.
ANCHOR_A = (0.0, 0.0, 2.0)
ANCHOR_B = (1.0, 0.0, 2.0)


def _distance(a, b):
    return sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5


class GoToSkill(Skill):
    name = 'go_to'
    timeout_sec = 30.0

    def __init__(self, body, x=None, y=None, z=None):
        super().__init__(body)
        # None means "not explicitly given" - resolved to a ping-pong
        # target in start(), not here, since that needs a live
        # position fix that may not exist yet at construction time.
        self._explicit_target = None if x is None else (
            x,
            y if y is not None else 0.0,
            z if z is not None else 2.0,
        )
        self.target = None

    def requirements(self):
        # Same fresh-process issue HoverSkill had: is_armed()/
        # current_mode()/current_position() all read instantly, before
        # this node's subscriptions have had a chance to receive
        # anything - wait for a real connection and a real position fix
        # first, then trust the values.
        if not self.body.wait_until_connected(timeout_sec=10.0):
            return False
        # Widened 10.0 -> 25.0 Oct 24, 2026 - see takeoff.py's matching
        # comment for why: a live run on shadow_runway saw this still
        # timing out well after the FCU's own log reported GPS in use.
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            return False
        return self.body.is_armed() and self.body.current_mode() == 'GUIDED'

    def start(self):
        if self._explicit_target is not None:
            self.target = self._explicit_target
            return
        pos = self.body.current_position()
        if pos is None:
            # requirements() already confirmed a position fix exists,
            # so this shouldn't happen - fall back to ANCHOR_B rather
            # than crash.
            self.target = ANCHOR_B
            return
        dist_a = _distance(pos, ANCHOR_A)
        dist_b = _distance(pos, ANCHOR_B)
        # Fly to whichever anchor is currently FARTHER away - that's
        # the one Shadow isn't already sitting at, so this always
        # demands real movement. On a tie (e.g. fresh takeoff sitting
        # exactly at ANCHOR_A) this picks ANCHOR_B, matching the old
        # "+1m forward" default for the very first run.
        self.target = ANCHOR_A if dist_a > dist_b else ANCHOR_B

    def update(self):
        x, y, z = self.target
        self.body.send_position_setpoint(x, y, z)
        pos = self.body.current_position()
        if pos is None:
            return False
        dx, dy, dz = pos[0] - x, pos[1] - y, pos[2] - z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance <= POSITION_TOLERANCE_M
