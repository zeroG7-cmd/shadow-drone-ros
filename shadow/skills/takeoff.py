#!/usr/bin/env python3
"""
TakeoffSkill - climb to a target altitude using ArduCopter's own
takeoff service, same as typing `takeoff 5` at the MAVProxy prompt in
Gate 1/Gate 2.

Tier 1. Gated behind ArmSkill's own unlock in the skill tree, not
enforced here - a Skill doesn't know or care about the unlock tree it
belongs to; that's run_skill.py/progress.json's job.

Nov 8, 2026 - Zer0's explicit call: this used to auto-arm (set GUIDED +
arm()) in start() if Shadow wasn't already armed, so pressing Takeoff
alone worked even with no prior Arm press. He wants the skill sequence
kept strict and procedural instead - arm has to actually happen first,
checked for real, not silently done on your behalf. requirements() now
rejects outright if Shadow isn't already armed in GUIDED mode; start()
no longer arms anything itself.
"""

from .base import Skill

DEFAULT_ALTITUDE_M = 2.0
ALTITUDE_TOLERANCE_M = 0.3


class TakeoffSkill(Skill):
    name = 'takeoff'
    timeout_sec = 20.0

    def __init__(self, body, altitude_m=DEFAULT_ALTITUDE_M):
        super().__init__(body)
        self.altitude_m = altitude_m
        self._commanded = False

    def requirements(self):
        # Same fresh-process issue every other skill's requirements()
        # already accounts for: is_armed()/current_mode() read instantly
        # would just see this new process's unset defaults, not
        # ArduPilot's real state, unless a real /mavros/state message
        # has actually arrived first.
        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().error(
                'takeoff: no MAVROS connection to ArduPilot.')
            return False
        # Nov 8, 2026 - see module docstring. Previously start() would
        # arm for you here instead of failing; Zer0 wants Takeoff to
        # require a real prior Arm press, not do it implicitly.
        if not (self.body.is_armed() and self.body.current_mode() == 'GUIDED'):
            self.body.get_logger().error(
                'takeoff: not armed in GUIDED mode - run arm first.')
            return False
        # Widened 10.0 -> 25.0 Oct 24, 2026, after a live run on
        # shadow_runway: the FCU's own log reported "EKF3 IMU0/IMU1 is
        # using GPS" a good 25-40s before this skill's own attempt even
        # started, yet a fresh process still timed out waiting for
        # /mavros/local_position/pose within the old 10s window. That
        # gap matters for diagnosis: if 25s still isn't enough, the
        # real problem isn't "didn't wait long enough" - something
        # (QoS, topic name, MAVROS config) may be stopping that topic
        # from ever reaching a fresh subscriber, not just stopping it
        # from reaching one quickly. Not yet root-caused either way.
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            self.body.get_logger().error(
                'takeoff: no position estimate yet - arming now would '
                'repeat the exact bug mission_planner.py already hit '
                'once (spinning in place, never climbing).')
            return False
        return True

    def start(self):
        self._commanded = self.body.takeoff(self.altitude_m)

    def update(self):
        if not self._commanded:
            return False
        pos = self.body.current_position()
        if pos is None:
            return False
        _, _, z = pos
        return abs(z - self.altitude_m) <= ALTITUDE_TOLERANCE_M
