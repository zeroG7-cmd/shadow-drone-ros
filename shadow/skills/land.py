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
        return self.body.is_armed()

    def start(self):
        self.body.set_mode('LAND')

    def update(self):
        return not self.body.is_armed()
