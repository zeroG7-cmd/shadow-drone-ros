#!/usr/bin/env python3
"""
GoToSkill - fly to a specific (x, y, z) point in MAVROS's local ENU
frame, relative to the arm point - or, with no point given, cycle
through all 5 known payload-prop positions in shadow_runway.sdf
(hovering above each one in turn), so a button press visibly goes "to
a payload" rather than to an arbitrary point.

Tier 2 - gated behind arm/takeoff/hover each having their own proven
streak, not because GoTo is technically harder, but because a
positioning mistake here is the first skill that could actually fly
Shadow into a wall if the earlier ones haven't proven setpoints behave
the way they're assumed to.

Fixed Oct 3, 2026 (first pass): the original version took a fixed
absolute target, which trivially re-passes on every run after the
first once the drone is already sitting there.

Fixed Oct 3, 2026 (second pass, same day): the first fix replaced that
with "+1m in x from wherever Shadow currently is," computed fresh each
run. That solves the trivial-repeat-pass problem but creates a worse
one - it never comes back, so every PASS walks the drone another 1m
further out. Zer0 ran it three times in a row from the repo root (no
directory mistake this time) and watched it fail on run 3 with
"limits() failed" - which tracks exactly: run 1 ends near x=1, run 2
near x=2 (right at the geofence edge), run 3's target of x=3 breaches
the geofence mid-flight. (That geofence was 2.0m at the time; it was
widened to 13.0m on Oct 24, 2026 when the world moved outdoors - see
shadow/limits.py's own docstring. Still worth keeping bounded on
principle, not because the old number still applies.) Unbounded drift
was always going to hit that wall eventually; it just took three runs.

That fixed-anchor-point ping-pong (two placeholder points 1m apart)
held until Nov 8, 2026, when Zer0 asked for the no-target default to
visit real places instead: the two known payload spots from
deliver_payload.py's KNOWN_ORANGE_PAYLOADS. The first version of that
change only had those 2 oranges, and reused a "fly to whichever anchor
is currently farther away" rule to get a round-robin with zero
persisted state (each skill run is a brand-new process with no memory
of the last button press - see run_skill.py's module docstring). That
only works because 2 points makes "alternate" and "round-robin" the
same thing.

Nov 8, 2026 (later, same night) - Zer0's actual ask turned out to be
broader: cycle through every payload-colored prop in the world, not
just the 2 oranges ("it goes to the orange one first click, next
click it goes to the blue one, and so on"). shadow_runway.sdf has 5
total: payload_orange_1/2 (the real pickup targets object_detector.py
is tuned to find) plus 3 deliberate distractor props it's tuned to
ignore - payload_distractor_blue/green/purple (see that file's own
comment above its 5 payload <model> blocks for why the distractors
exist and their exact hues). With only 2 points, "farther-point-wins"
gave a true round-robin for free; with 5 it doesn't - that rule can
get stuck forever bouncing between whichever two points are most
distant from each other and never visit the closer three at all. So
this now keeps a tiny bit of real state on disk (just a cycle index -
see STATE_FILE below) instead of trying to infer the next stop from
position alone. The index advances on every run - whether or not that
run's flight itself ends in PASS or FAIL - because Zer0 is clicking
the button once per visit, not once per success.

Deliberate exception, not an oversight: deliver_payload.py's own module
docstring documents Zer0's Oct 24, 2026 call that its real pickup
mission must NOT fly to pre-known payload coordinates - true visual
search only, because that mission exists to prove real perception
works. go_to is a lower-tier, no-perception movement/test skill, so
flying it straight to a known spot (any of the 5, not just the 2 real
ones) for a quick visual check is a different job, not a reversal of
that decision - worth a second look if that reasoning doesn't hold up
on reflection.

These 5 coordinates are COPIED from shadow_runway.sdf's own <pose>
values (the 2 oranges also happen to match deliver_payload.py's
KNOWN_ORANGE_PAYLOADS), not imported from either - deliver_payload.py
pulls in gz.transport13/gz.msgs10/numpy at import time for its own
Gazebo-entity bookkeeping, which this much simpler Tier 2 skill has no
reason to drag in, and the 3 distractors aren't in that module at all
(it only tracks the 2 it's allowed to despawn). Keep this list in sync
by hand if the world's payload layout ever changes. Passing an
explicit x/y/z still flies to that literal absolute point, unaffected
by any of this.
"""

import json
from pathlib import Path

from .base import Skill
from .heading import NoseFirst

POSITION_TOLERANCE_M = 0.3

# Hover height above ground when visiting a payload spot - high enough
# to clear the 0.3m payload box with margin, low enough that it's
# visibly "at that payload" rather than cruising over the area at the
# usual 2.0m hover altitude. Zer0's call, Nov 8, 2026.
# Raised 1.0 -> 2.5 m (Nov 8, 2026, same morning): live test, the leg
# orange_1 -> orange_2 (12 m) timed out with Shadow on the ground. Shadow
# hovers at ~70% throttle (rc_out ~1714 PWM), so tilting hard to
# accelerate costs height; 1 m left no margin. 2.5 m gives room to sag
# and recover, and is still low enough to read the prop in the camera.
PAYLOAD_HOVER_ALTITUDE_M = 2.5

# All 5 payload props in shadow_runway.sdf, in the order they're
# defined there. Only the 2 oranges are real pickup targets; the other
# 3 are deliberate distractors - see module docstring. go_to visits all
# 5 anyway, since it's a no-perception test skill, not the real
# vision-gated mission.
WAYPOINTS = [
    ('payload_orange_1', 6.0, 2.0),
    ('payload_orange_2', -5.0, 7.0),
    ('payload_distractor_blue', 3.0, -4.0),
    ('payload_distractor_green', -6.0, -3.0),
    ('payload_distractor_purple', 1.0, 9.0),
]

# Persisted cycle position - deliberately NOT progress.json (that
# file's job is per-skill streak/unlock bookkeeping for run_skill.py,
# not this skill's own internal "which stop is next" state) and
# deliberately NOT importable from anywhere else, same reasoning as
# telemetry_bridge.py's DB_PATH: one fixed, obvious, home-relative
# path beats threading a new constructor argument through run_skill.py
# just for this. Lives under shadow_sim_runtime/ (runtime state),
# never shadow_ws/ (git-tracked source) - this index isn't something
# anyone should be committing or reviewing.
STATE_FILE = Path.home() / 'shadow_sim_runtime' / 'state' / 'go_to_cycle.json'


def _next_waypoint_index():
    """Reads the last-used index, advances it (wrapping), persists the
    new value, and returns the index to use THIS run. Best-effort: a
    missing or corrupt state file just restarts the cycle at 0 rather
    than failing a skill run over it - this is a nice-to-have visiting
    order, not a safety-relevant value."""
    index = 0
    try:
        with open(STATE_FILE, 'r') as f:
            index = int(json.load(f).get('index', 0))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        index = 0

    index %= len(WAYPOINTS)

    try:
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(STATE_FILE, 'w') as f:
            json.dump({'index': (index + 1) % len(WAYPOINTS)}, f)
    except OSError:
        pass  # best-effort, same reasoning as the read above

    return index


class GoToSkill(Skill):
    name = 'go_to'
    # 30 -> 45 s (Nov 8): the longest payload-to-payload leg is ~12 m and
    # WP_SPD is now capped at 3 m/s in shadow_sitl.parm.
    timeout_sec = 45.0

    def __init__(self, body, x=None, y=None, z=None):
        super().__init__(body)
        # None means "not explicitly given" - resolved to the next
        # cycle waypoint in start(), not here, since picking it doesn't
        # need a live position fix the way the old ping-pong logic did
        # (the cycle index comes from disk, not from where Shadow
        # currently is).
        self._explicit_target = None if x is None else (
            x,
            y if y is not None else 0.0,
            z if z is not None else 2.0,
        )
        self.target = None
        self._nose = None

    def requirements(self):
        # Same fresh-process issue HoverSkill had: is_armed()/
        # current_mode()/current_position() all read instantly, before
        # this node's subscriptions have had a chance to receive
        # anything - wait for a real connection and a real position fix
        # first, then trust the values.
        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().warn('go_to: requirements failed - no MAVROS link within 10 s.')
            return False
        # Widened 10.0 -> 25.0 Oct 24, 2026 - see takeoff.py's matching
        # comment for why: a live run on shadow_runway saw this still
        # timing out well after the FCU's own log reported GPS in use.
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            self.body.get_logger().warn('go_to: requirements failed - no position estimate within 25 s.')
            return False
        # Nov 8, 2026 - intermittent "requirements() failed" right after a
        # real arm+takeoff (skill-training-button-bugs-nov7.md, Bug 2).
        # Armed/mode used to be read exactly once, the instant the link
        # came up. Now polled for up to 3 s while spinning, so one early
        # or stale state message can't fail the run - and the log says
        # WHICH check failed, so if this recurs it's diagnosable instead
        # of a bare "requirements() failed".
        import rclpy
        deadline = self.body.get_clock().now().nanoseconds + int(3.0 * 1e9)
        while rclpy.ok():
            armed, mode = self.body.is_armed(), self.body.current_mode()
            if armed and mode == 'GUIDED':
                return True
            if self.body.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self.body, timeout_sec=0.2)
        self.body.get_logger().warn(
            f'go_to: requirements failed - armed={armed}, mode={mode!r} '
            '(needs armed=True and GUIDED; press Arm then Takeoff first).')
        return False

    def start(self):
        if self._explicit_target is not None:
            self.target = self._explicit_target
            return
        name, x, y = WAYPOINTS[_next_waypoint_index()]
        self.body.get_logger().info(f'go_to: heading to {name} at ({x}, {y}).')
        self.target = (x, y, PAYLOAD_HOVER_ALTITUDE_M)

    def update(self):
        x, y, z = self.target
        # Nov 8, 2026 - nose-first: face the target (turning on the spot
        # first on a big heading change) instead of sliding sideways with
        # the spawn heading. Arrival is still judged on the real target,
        # never on the temporary hold point used while turning.
        if self._nose is None:
            self._nose = NoseFirst(self.body, turn_first=True)
        sx, sy, sz, yaw = self._nose.setpoint((x, y, z))
        self.body.send_position_setpoint(sx, sy, sz, yaw=yaw)
        pos = self.body.current_position()
        if pos is None:
            return False
        dx, dy, dz = pos[0] - x, pos[1] - y, pos[2] - z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance <= POSITION_TOLERANCE_M
