#!/usr/bin/env python3
"""
GoToSkill - fly to a specific (x, y, z) point in MAVROS's local ENU
frame, relative to the arm point.

Tier 2 - gated behind arm/takeoff/hover each having their own proven
streak, not because GoTo is technically harder, but because a
positioning mistake here is the first skill that could actually fly
Shadow into a wall if the earlier ones haven't proven setpoints behave
the way they're assumed to.
"""

from .base import Skill

POSITION_TOLERANCE_M = 0.3


class GoToSkill(Skill):
    name = 'go_to'
    timeout_sec = 30.0

    def __init__(self, body, x, y, z):
        super().__init__(body)
        self.target = (x, y, z)

    def requirements(self):
        return (self.body.is_armed()
                and self.body.current_mode() == 'GUIDED'
                and self.body.current_position() is not None)

    def start(self):
        pass  # update() streams the setpoint every tick; nothing to fire once

    def update(self):
        x, y, z = self.target
        self.body.send_position_setpoint(x, y, z)
        pos = self.body.current_position()
        if pos is None:
            return False
        dx, dy, dz = pos[0] - x, pos[1] - y, pos[2] - z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance <= POSITION_TOLERANCE_M
