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
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

import rclpy

from shadow.body import ArduPilotBody
from shadow.skills import (
    ArmSkill, TakeoffSkill, HoverSkill, GoToSkill, LandSkill,
    DeliverPayloadSkill, LearnedFlightSkill, RtlSkill, FollowPathSkill,
    MapAreaSkill,
    SearchSkill,
    ApproachTargetSkill,
    FollowTargetSkill,
    DeliveryRehearsalSkill,
)

#  Oct 26, 2026 (even later) - the agility_circuit checkpoint, for the
# new 'waypoint_circuit' Skill Test below. Same path convention as
# LearnedFlightSkill's own DEFAULT_CHECKPOINT - duplicated rather than
# imported so this file doesn't reach into shadow.skills.learned_flight's
# module-level constants just for a path string.
WAYPOINT_CIRCUIT_CHECKPOINT = os.path.expanduser(
    '~/shadow_ws/ml/models/agility_circuit/agility_circuit_ppo_final.zip')

SKILLS = {
    'arm': lambda body, args: ArmSkill(body),
    'takeoff': lambda body, args: TakeoffSkill(body, altitude_m=args.altitude),
    'hover': lambda body, args: HoverSkill(body, duration_sec=args.duration),
    'go_to': lambda body, args: GoToSkill(body, args.x, args.y, args.z),
    'land': lambda body, args: LandSkill(body),
    # Nov 8, 2026 - Tier 2. rtl = fly home and land there (ArduPilot RTL
    # mode, checked against ArduPilot's real home position). follow_path
    # = fly a waypoint chain without stopping at each point; --path
    # zigzag (default, the delivery search sweep) or circuit.
    'rtl': lambda body, args: RtlSkill(body),
    'follow_path': lambda body, args: FollowPathSkill(body, path=args.path),
    # Nov 9, 2026 - Mapping branch: fly the path while slam_toolbox maps,
    # then check the map against the real wall/pillars and save it.
    'map_area': lambda body, args: MapAreaSkill(body, path=args.path),
    # Nov 9, 2026 - vision skills (camera + attitude + altitude -> ground
    # position, shadow/senses/ground_target.py) and the first mission
    # built purely out of skills.
    'search': lambda body, args: SearchSkill(body, path=args.path, full_path=args.full_path),
    'approach_target': lambda body, args: ApproachTargetSkill(body),
    'follow_target': lambda body, args: FollowTargetSkill(body, move_target=args.move_target),
    'delivery_rehearsal': lambda body, args: DeliveryRehearsalSkill(body),
    # Tier 3 - the first real mission. Needs shadow_runway.sdf's 5
    # payload props + delivery_pad (added Oct 24, 2026) and the
    # widened 13.0m geofence in shadow/limits.py; takes no extra CLI
    # args of its own, everything it needs is in deliver_payload.py's
    # own constants.
    'deliver_payload': lambda body, args: DeliverPayloadSkill(body),
    # Tier 3, Oct 25, 2026 (later) - the first Skill flown by a TRAINED
    # policy's forward pass instead of hand-coded control math. Takes
    # no extra CLI args of its own yet (same reasoning as
    # deliver_payload above) - checkpoint/waypoints are LearnedFlightSkill's
    # own defaults; see that file's module docstring for the bridge
    # design and why go_to's checkpoint is the default for tonight.
    'learned_flight': lambda body, args: LearnedFlightSkill(body),
    # Tier 3, Oct 26, 2026 (even later) - the SAME bridge Skill as
    # learned_flight above, just handed the agility_circuit checkpoint
    # instead of go_to's default. No new flight-control code needed:
    # LearnedFlightSkill's own module docstring explains why one Skill
    # can fly either checkpoint (both training envs share the same
    # obs/action contract) - this is that "pass checkpoint_path=...
    # instead" the docstring already calls out, now wired to an actual
    # button under Sequences rather than only reachable by editing code.
    'waypoint_circuit': lambda body, args: LearnedFlightSkill(
        body, checkpoint_path=WAYPOINT_CIRCUIT_CHECKPOINT),
}

STREAK_TO_UNLOCK = 3

# Same DB, same path convention telemetry_bridge.py already uses - see
# that file's own header for why simulation data lives in
# zerogravity_rnd.db specifically, never legacy_shadow.db.
DB_PATH = Path.home() / "zeroGravity-rnd" / "lab" / "database" / "zerogravity_rnd.db"


def _ensure_skill_attempts_table(conn):
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS skill_attempts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            simulation_run_id INTEGER NOT NULL,
            skill_name TEXT NOT NULL,
            result TEXT NOT NULL,
            reason TEXT,
            streak_after INTEGER,
            attempted_at TEXT NOT NULL
        )
        """
    )


def _record_skill_attempt(skill_name, passed, reason, streak_after):
    """Links this skill attempt to whichever simulation_runs row the
    currently-running telemetry_bridge.py created for this sim session
    (if any) - the piece that was missing before: progress.json knew
    streaks, the database knew telemetry, and nothing tied a given run
    to which skill was actually being trained during it. This is what
    lets a Training Log entry say "this run trained go_to, passed,
    streak now 4" instead of just showing raw flight data with no idea
    what it was for.

    Deliberately best-effort and never safety-critical: if
    telemetry_bridge.py isn't running for this sim session (no
    'running' simulation_runs row to link to), or the DB write fails
    for any reason, this logs a warning and moves on - it never blocks
    or fails the skill run itself, and progress.json (the thing that
    actually gates unlocks) is already saved by the time this runs.
    """
    try:
        conn = sqlite3.connect(str(DB_PATH))
        try:
            _ensure_skill_attempts_table(conn)
            row = conn.execute(
                "SELECT id FROM simulation_runs WHERE status = 'running' "
                "ORDER BY started_at DESC LIMIT 1"
            ).fetchone()
            if row is None:
                print(
                    '[run_skill] No active simulation_runs row found (is '
                    'telemetry_bridge.py running for this sim session?) - '
                    'result saved to progress.json only, not linked in '
                    'the database.',
                    file=sys.stderr)
                return
            conn.execute(
                """
                INSERT INTO skill_attempts (
                    simulation_run_id, skill_name, result, reason,
                    streak_after, attempted_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    row[0], skill_name, 'pass' if passed else 'fail',
                    reason, streak_after,
                    datetime.now(timezone.utc).isoformat(),
                ),
            )
            conn.commit()
        finally:
            conn.close()
    except sqlite3.Error as e:
        print(f'[run_skill] Failed to record skill attempt in database: {e}',
              file=sys.stderr)


def load_progress(path):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return json.load(f)


def save_progress(path, progress):
    with open(path, 'w') as f:
        json.dump(progress, f, indent=2, sort_keys=True)
        f.write('\n')


# Nov 9, 2026 - live run status for the Sim Bay (telemetry panel's
# skill / mission phase strip). Written ~2x a second while a skill runs
# and once more with the result. Read by simulation_control.py's
# /simulation/run-status. Best-effort: a failed write never affects the run.
RUN_STATUS_FILE = Path.home() / 'shadow_sim_runtime' / 'state' / 'run_status.json'
_run_started = None


def _short(label):
    return label.split(' (')[0]


def write_run_status(skill, state, reason=None):
    global _run_started
    if _run_started is None:
        _run_started = datetime.now(timezone.utc).timestamp()
    now = datetime.now(timezone.utc).timestamp()
    data = {'skill': skill.name, 'state': state, 'reason': reason,
            'started': _run_started, 't': now, 'elapsed': round(now - _run_started, 1)}
    stages = getattr(skill, 'stages', None)
    if stages:
        data['kind'] = 'sequence'
        data['stages'] = [_short(label) for label, _ in stages]
        data['stage_index'] = getattr(skill, 'i', -1)
        cur = getattr(skill, 'cur', None)
        data['phase'] = getattr(cur, 'phase', None) if cur is not None else None
    else:
        data['kind'] = 'skill'
        data['phase'] = getattr(skill, 'phase', None)
    try:
        RUN_STATUS_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = RUN_STATUS_FILE.with_suffix('.tmp')
        tmp.write_text(json.dumps(data))
        os.replace(tmp, RUN_STATUS_FILE)
    except OSError:
        pass


def run_skill(skill, tick_hz=10.0):
    """Runs requirements() once, then start()/update()/limits() in a
    loop until update() reports completion, limits() fails, or
    skill.timeout_sec elapses. Returns (passed: bool, reason: str)."""
    write_run_status(skill, 'checking')
    if not skill.requirements():
        return False, 'requirements() failed'

    skill.start()

    period = 1.0 / tick_hz
    elapsed = 0.0
    last_status = 0.0
    while elapsed < skill.timeout_sec:
        if elapsed - last_status >= 0.5:
            write_run_status(skill, 'running')
            last_status = elapsed
        if not skill.limits():
            return False, 'limits() failed - aborted for safety'
        if skill.update():
            return True, 'completed'
        # Nov 9, 2026 - a skill can fail with its own reason (map_area:
        # "2 of 4 pillars found") instead of only via limits()/timeout.
        if getattr(skill, 'failed', None):
            return False, skill.failed
        # Oct 25, 2026 fix - rclpy.spin_once() services at most ONE
        # ready callback per call, not every pending one. skill.body
        # (ArduPilotBody) has /mavros/state and /mavros/local_position/
        # pose subscribed in __init__ - continuously busy, and
        # registered before anything a Skill adds later. A Skill like
        # deliver_payload.py that subscribes to something ELSE on this
        # same node (PayloadSensor's /shadow/target_position, created
        # in start(), after state/pose already exist) can get starved
        # outright: every single spin_once call keeps finding
        # state/pose ready first and never gets around to servicing
        # the later subscription. That's exactly what froze
        # deliver_payload's target_offset()/target_area() on whatever
        # one lucky Point message slipped through early on, for the
        # entire rest of a run (see areas/search-and-deliver-mission.md,
        # Run 6) - not a missing "approach skill/mode", a plain
        # callback-starvation bug. Draining a bounded number of times
        # per tick instead of once gives every subscription on this
        # node an actual chance each tick, while still respecting the
        # tick's time budget overall (worst case: this tick's full
        # `period`, spread over up to 10 waits instead of 1).
        for _ in range(10):
            rclpy.spin_once(skill.body, timeout_sec=period / 10)
        elapsed += period

    return False, f'timed out after {skill.timeout_sec}s'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('skill', choices=sorted(SKILLS.keys()))
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--duration', type=float, default=5.0)
    # Default is None, not a fixed point: GoToSkill treats None as
    # "move a fixed offset from wherever Shadow currently is," not an
    # absolute world point. A fixed default breaks on repeat runs -
    # the first go_to really does move the drone there, so every run
    # after that is trivially "already at the target" and passes
    # without moving at all. Caught live Oct 3, 2026 when repeat go_to
    # runs looked like no-ops. Pass --x/--y/--z explicitly to fly to a
    # literal absolute point instead.
    parser.add_argument('--x', type=float, default=None)
    parser.add_argument('--y', type=float, default=None)
    parser.add_argument('--z', type=float, default=None)
    parser.add_argument('--full-path', action='store_true',
                        help='search only: keep flying the whole path after a find.')
    parser.add_argument('--move-target', action='store_true',
                        help='follow_target only: run scripts/move_target.py as the moving target.')
    parser.add_argument('--path', choices=['zigzag', 'circuit'], default='zigzag',
                        help='follow_path only: which built-in path to fly.')
    parser.add_argument(
        '--progress-file', default=None,
        help="Defaults to progress.json in the current directory - see "
             "this file's module docstring for why that's deliberate.")
    parsed = parser.parse_args(args)

    progress_path = parsed.progress_file or os.path.join(os.getcwd(), 'progress.json')

    # Caught live Oct 3, 2026: running this from the wrong directory
    # (e.g. the colcon workspace root instead of the shadow_drone repo
    # root) silently creates a brand-new, untracked progress.json
    # instead of using the real one - fragmenting progress with no
    # indication anything went wrong. Warn loudly instead.
    if not parsed.progress_file and not os.path.exists(progress_path):
        print(
            f'[run_skill] WARNING: no progress.json found at '
            f'{progress_path} - creating a new, empty one here. If you '
            f'meant to use the real, git-tracked progress.json, run '
            f'this from the shadow_drone repo root instead (or pass '
            f'--progress-file explicitly).',
            file=sys.stderr)

    rclpy.init()
    body = ArduPilotBody()

    try:
        skill = SKILLS[parsed.skill](body, parsed)
        passed, reason = run_skill(skill)
        write_run_status(skill, 'passed' if passed else 'failed', reason)
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

    _record_skill_attempt(parsed.skill, passed, reason, entry['streak'])

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
