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
        # Added Oct 24, 2026, live on shadow_runway: a heartbeat alone
        # isn't "ready to arm" - ArduCopter rejected an actual arm
        # attempt with "Arm: Accels inconsistent" when it was tried
        # only ~17s after MAVROS connected, while a run that happened
        # to wait ~114s armed cleanly with no rejection at all. The two
        # simulated accelerometers need real settling time after SITL
        # boots, and nothing here was ever waiting for that - every
        # other skill downstream waits for a position estimate before
        # doing anything, but arm() itself never did, so it was racing
        # the sim's own startup on every single run. Reusing that same
        # wait here as the best available readiness signal: it needs
        # EKF origin to be set, which only happens after initial sensor
        # settling, so it's a reasonable proxy for "accels have
        # stopped disagreeing" even though it isn't that check
        # directly. 45s, longer than takeoff/go_to/hover's 25s, because
        # this is the FIRST thing attempted after Start Simulation -
        # nothing upstream has already been waiting on anything yet.
        if not self.body.wait_for_position_estimate(timeout_sec=45.0):
            self.body.get_logger().error(
                'arm: no position estimate yet - the sim likely hasn\'t '
                'finished settling (EKF/GPS take a while after Gazebo '
                'first comes up). Not even attempting to arm yet.')
            return False
        return True

    def start(self):
        if not self.body.set_mode('GUIDED'):
            return
        self.body.arm()

    def update(self):
        return self.body.is_armed() and self.body.current_mode() == 'GUIDED'
