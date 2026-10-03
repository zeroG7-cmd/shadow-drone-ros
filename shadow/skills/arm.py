#!/usr/bin/env python3
"""
ArmSkill - the simplest skill there is: switch to GUIDED and arm.

Tier 1, no prerequisites. Mirrors the first two lines of the real
MAVProxy sequence that actually flew in Gate 1/Gate 2:

    mode guided
    arm throttle
"""

from .base import Skill


class ArmSkill(Skill):
    name = 'arm'
    timeout_sec = 10.0

    def requirements(self):
        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().error(
                'arm: no MAVROS connection to ArduPilot.')
            return False
        return True

    def start(self):
        if not self.body.set_mode('GUIDED'):
            return
        self.body.arm()

    def update(self):
        return self.body.is_armed() and self.body.current_mode() == 'GUIDED'
