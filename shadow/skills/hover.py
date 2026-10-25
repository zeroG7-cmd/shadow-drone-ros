#!/usr/bin/env python3
"""
HoverSkill - hold position for a fixed duration while already flying.

Tier 1. The point of this one isn't the manoeuvre itself (it's trivial)
but training stability: three good training runs of "stay roughly
still for N seconds" is a real, checkable signal that position-hold
setpoints are actually working, before anything more ambitious depends
on that.

Fixed Oct 3, 2026 (first pass): requirements() checked
current_position() instantly, before this fresh process's
subscriptions had a chance to receive anything - replaced with
wait_for_position_estimate(), same pattern TakeoffSkill uses.

Fixed Oct 3, 2026 (second pass - written into a stray hover-1.py by
mistake and never actually merged into this file until Oct 17, 2026,
after a git-status/diff check showed the real hover.py on disk still
had the old, unpatched requirements()/limits()): the original version
also recorded whatever position the drone happened to be at -
including sitting on the ground, disarmed - as the "hold point," then
declared success purely because a timer ran out, with no check that
holding actually worked. That is exactly the "flies on timers, not
measured arrival" bug the whole Skill redesign exists to fix,
reintroduced here by mistake. Now requires actually being armed and in
GUIDED mode before it will even start, and limits() fails the run
immediately if Shadow drifts more than HOLD_TOLERANCE_M from the hold
point at any point during the hold - not just a timer with nothing
behind it.

Because the real fix never reached this file until now, every prior
hover PASS (the streak of 7 recorded in progress.json as of Oct 17)
was earned under the broken logic, not this one - see progress.json
and the roadmap doc for the reset this required.
"""

from .base import Skill

DEFAULT_DURATION_SEC = 5.0
HOLD_TOLERANCE_M = 0.3


class HoverSkill(Skill):
    name = 'hover'

    def __init__(self, body, duration_sec=DEFAULT_DURATION_SEC):
        super().__init__(body)
        self.duration_sec = duration_sec
        self.timeout_sec = duration_sec + 10.0
        self._hold_point = None
        self._start_time = None

    def requirements(self):
        # Widened 10.0 -> 25.0 Oct 24, 2026 - see takeoff.py's matching
        # comment for why: a live run on shadow_runway saw this still
        # timing out well after the FCU's own log reported GPS in use.
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            return False
        # Hovering only means something if Shadow is actually flying.
        # Recording wherever it happens to be sitting - including on
        # the ground, disarmed - and calling that a hover was the bug.
        if not (self.body.is_armed() and self.body.current_mode() == 'GUIDED'):
            self.body.get_logger().error(
                'hover: not armed and in GUIDED - take off first.')
            return False
        return True

    def limits(self):
        # Base geofence/altitude check first.
        if not super().limits():
            return False
        if self._hold_point is None:
            return True  # start() hasn't run yet
        pos = self.body.current_position()
        if pos is None:
            return True
        x, y, z = self._hold_point
        dx, dy, dz = pos[0] - x, pos[1] - y, pos[2] - z
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        if distance > HOLD_TOLERANCE_M:
            self.body.get_logger().error(
                f'hover: drifted {distance:.2f}m from the hold point '
                f'({HOLD_TOLERANCE_M}m tolerance) - failing, not waiting '
                f'out the timer anyway.')
            return False
        return True

    def start(self):
        self._hold_point = self.body.current_position()
        self._start_time = self.body.get_clock().now()

    def update(self):
        if self._hold_point is None:
            return False
        x, y, z = self._hold_point
        self.body.send_position_setpoint(x, y, z)
        elapsed = (self.body.get_clock().now() - self._start_time).nanoseconds / 1e9
        return elapsed >= self.duration_sec
