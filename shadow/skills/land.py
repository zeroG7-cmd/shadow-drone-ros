#!/usr/bin/env python3
"""
LandSkill - switch to ArduCopter's LAND mode and wait for disarm.

ArduCopter disarms itself automatically once LAND mode detects
touchdown - that auto-disarm is what this skill actually waits for,
rather than guessing at a landed altitude. This is the direct fix for
the old mission_planner.py bug where landing was never confirmed at
all (the script just assumed it worked).
"""

from .base import Skill


class LandSkill(Skill):
    name = 'land'
    timeout_sec = 30.0

    def requirements(self):
        # Same fresh-process issue as HoverSkill/GoToSkill: is_armed()
        # read instantly would always see the node's unset default
        # (False) rather than ArduPilot's real current state. Wait for
        # a real connection first - wait_until_connected() loops until
        # the same State message that carries is_armed()'s value has
        # actually arrived at least once.
        return self.body.wait_until_connected(timeout_sec=10.0) and self.body.is_armed()

    def start(self):
        self.body.set_mode('LAND')

    def update(self):
        return not self.body.is_armed()
