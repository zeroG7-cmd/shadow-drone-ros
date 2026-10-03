#!/usr/bin/env python3
"""
HoverSkill - hold the current position for a fixed duration.

Tier 1. The point of this one isn't the manoeuvre itself (it's trivial)
but training stability: three good training runs of "stay roughly
still for N seconds" is a real, checkable signal that position-hold
setpoints are actually working, before anything more ambitious depends
on that.
"""

from .base import Skill

DEFAULT_DURATION_SEC = 5.0


class HoverSkill(Skill):
    name = 'hover'

    def __init__(self, body, duration_sec=DEFAULT_DURATION_SEC):
        super().__init__(body)
        self.duration_sec = duration_sec
        self.timeout_sec = duration_sec + 10.0
        self._hold_point = None
        self._start_time = None

    def requirements(self):
        return self.body.current_position() is not None

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
