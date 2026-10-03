#!/usr/bin/env python3
"""
run_skill.py - command-line entry point for running one Skill against
a live SITL+Gazebo session and recording whether it passed.

This does NOT start ArduPilot SITL or Gazebo - start those yourself
first (same manual sim_vehicle.py / gz sim commands that actually flew
Gate 1/Gate 2), then run this against the live session.

Usage (from the shadow_drone repo root):

    ros2 run shadow_drone run_skill.py arm
    ros2 run shadow_drone run_skill.py takeoff --altitude 2.0
    ros2 run shadow_drone run_skill.py hover --duration 8
    ros2 run shadow_drone run_skill.py go_to --x 1.0 --y 0.0 --z 2.0
    ros2 run shadow_drone run_skill.py land

Progress file path: defaults to progress.json in the CURRENT WORKING
DIRECTORY, not next to this script. That's deliberate, not an
oversight - once colcon installs this file, __file__ resolves to the
installed copy under install/shadow_drone/lib/shadow_drone/, which is
NOT the git-tracked source tree. Resolving the path from __file__
would silently write progress to a location nothing commits or
tracks, and the Pi would never see Shadow's real sim-earned unlocks.
Run this from the shadow_drone repo root (where progress.json actually
lives and is version-controlled), or pass --progress-file explicitly
if that's ever not true.
"""

import argparse
import json
import os
import sys

import rclpy

from shadow.body import ArduPilotBody
from shadow.skills import (
    ArmSkill, TakeoffSkill, HoverSkill, GoToSkill, LandSkill,
)

SKILLS = {
    'arm': lambda body, args: ArmSkill(body),
    'takeoff': lambda body, args: TakeoffSkill(body, altitude_m=args.altitude),
    'hover': lambda body, args: HoverSkill(body, duration_sec=args.duration),
    'go_to': lambda body, args: GoToSkill(body, args.x, args.y, args.z),
    'land': lambda body, args: LandSkill(body),
}

STREAK_TO_UNLOCK = 3


def load_progress(path):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return json.load(f)


def save_progress(path, progress):
    with open(path, 'w') as f:
        json.dump(progress, f, indent=2, sort_keys=True)
        f.write('\n')


def run_skill(skill, tick_hz=10.0):
    """Runs requirements() once, then start()/update()/limits() in a
    loop until update() reports completion, limits() fails, or
    skill.timeout_sec elapses. Returns (passed: bool, reason: str)."""
    if not skill.requirements():
        return False, 'requirements() failed'

    skill.start()

    period = 1.0 / tick_hz
    elapsed = 0.0
    while elapsed < skill.timeout_sec:
        if not skill.limits():
            return False, 'limits() failed - aborted for safety'
        if skill.update():
            return True, 'completed'
        rclpy.spin_once(skill.body, timeout_sec=period)
        elapsed += period

    return False, f'timed out after {skill.timeout_sec}s'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('skill', choices=sorted(SKILLS.keys()))
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=2.0)
    parser.add_argument(
        '--progress-file', default=None,
        help="Defaults to progress.json in the current directory - see "
             "this file's module docstring for why that's deliberate.")
    parsed = parser.parse_args(args)

    progress_path = parsed.progress_file or os.path.join(os.getcwd(), 'progress.json')

    rclpy.init()
    body = ArduPilotBody()

    try:
        skill = SKILLS[parsed.skill](body, parsed)
        passed, reason = run_skill(skill)
    finally:
        body.destroy_node()
        rclpy.shutdown()

    progress = load_progress(progress_path)
    entry = progress.setdefault(parsed.skill, {'streak': 0, 'unlocked': False})

    if passed:
        entry['streak'] += 1
        print(f'[run_skill] {parsed.skill}: PASS ({reason}) - streak now {entry["streak"]}')
    else:
        entry['streak'] = 0
        print(f'[run_skill] {parsed.skill}: FAIL ({reason}) - streak reset to 0')

    if entry['streak'] >= STREAK_TO_UNLOCK:
        entry['unlocked'] = True

    save_progress(progress_path, progress)

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
