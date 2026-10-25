#!/usr/bin/env python3
"""
DeliverPayloadSkill - Tier 3. The first real mission: take off, search
the open area with NO pre-known payload coordinates (true visual
search, not known-waypoints - Zer0's explicit call over AskUserQuestion
on Oct 24, 2026), lock onto the one orange payload the live camera
feed actually finds, "pick it up" (despawn it in place, same trick
Zer0 picked for the pickup showcase - no gripper exists yet), fly it
to a fixed delivery pad, and "drop it off" (respawn an orange box at
the pad). One payload per run, by explicit choice over "both oranges".

This talks to THREE different transports, and that's deliberate, not
sloppy layering:

  - MAVROS (via self.body, an ArduPilotBody) for actual flight - arm,
    takeoff, position setpoints. Same as every other skill.
  - ROS2 topics (via PayloadSensor, subscribed through self.body like
    every other Sense) for object_detector.py's live detections.
  - Gazebo's native gz-transport (via a SEPARATE gz.transport13.Node
    this file owns itself) for the world/create and world/remove
    services that despawn/respawn a payload model. Nothing else in
    shadow/ needed gz-transport directly until now - sim_camera_stream.py
    and object_detector.py both subscribe to gz-transport topics for
    the same reason (Gazebo's camera sensor was never bridged into
    ROS2), and this mission reuses that same confirmed pattern plus
    the two world-editing services, which have no ROS2/MAVROS
    equivalent at all.

PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION must be set before the first
gz.msgs10 import, for the exact same reason it's set in
sim_camera_stream.py/object_detector.py: apt's gz-msgs10 Python
bindings were generated against an older protobuf than the one that
resolves at runtime from Zer0's own ~/.local site-packages, and without
this the import chain raises "Descriptors cannot be created directly."
Scoped to this process only (os.environ.setdefault), not a system-wide
pip downgrade - see those two files' own comments for the fuller
reasoning (the numpy<2 system-wide downgrade earlier in this project
caused exactly the kind of collateral breakage this avoids).

ASSUMPTIONS THIS FILE MAKES, not yet independently confirmed live -
check these against telemetry the first time this actually runs,
and see the two constants below (FORWARD_AXIS_SIGN / RIGHT_AXIS_SIGN)
for how to flip them if wrong:

  1. Shadow's yaw never changes in flight. send_position_setpoint()
     always sets IGNORE_YAW | IGNORE_YAW_RATE, so nothing in this
     codebase has ever commanded a heading change - Shadow should sit
     at its spawn yaw (90 degrees, i.e. facing world +Y) for the whole
     mission. If that's wrong, the whole search/approach geometry
     below needs rethinking, not just a sign flip.
  2. MAVROS's local ENU frame and the Gazebo world frame are the same
     frame (same origin, same axes) - shadow_runway.sdf's spherical
     coordinates have heading_deg=0 and this SITL setup has no GPS
     drift modelled, so the payload/pad coordinates read directly out
     of shadow_runway.sdf should be usable as-is for send_position_setpoint
     calls, with no transform in between.
  3. front_camera has no pitch/gimbal control (confirmed - see
     model.sdf's own sensor comment) and an 80-degree horizontal FOV,
     giving roughly a 60-degree vertical FOV. For a 0.3m box resting on
     the ground to appear in frame at all from a level camera at
     height H, the drone needs to be roughly within H / tan(30 deg)
     of the box horizontally - which is why SEARCH_ALTITUDE_M is kept
     low (~1.1m) rather than at the normal 2.0m hover altitude. Never
     independently measured against the real rendered frame - if the
     search sweep flies right past a payload without detecting it,
     lowering SEARCH_ALTITUDE_M further is the first thing to try.
"""

import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean

from .base import Skill
from shadow.senses.payload_sensor import PayloadSensor

WORLD_NAME = 'shadow_runway'

SEARCH_ALTITUDE_M = 1.1   # low on purpose - see module docstring, assumption 3
TAKEOFF_ALT_TOLERANCE_M = 0.3
WAYPOINT_TOLERANCE_M = 0.5

# Facing is fixed at spawn yaw (assumption 1): +Y is "forward" (the way
# the camera points), +X is "right" from the drone's own point of view
# while facing +Y. If live testing shows the lateral lock-on correction
# below is pushing AWAY from the target instead of toward it, that
# means this sign guess was backwards - flip RIGHT_AXIS_SIGN (not the
# whole approach algorithm) to -1.0 and retest.
RIGHT_AXIS_SIGN = 1.0
FORWARD_AXIS_SIGN = 1.0

# Boustrophedon sweep over the area the payloads actually live in
# (y = 0..9, matching shadow_runway.sdf's placements), built as a
# small list of row-end waypoints rather than every point in between -
# object_detector.py's feed is checked every tick while Shadow is
# TRANSITING between these points too, so the straight-line legs
# themselves are the sweep, not just the endpoints. This list never
# reads a payload's actual coordinates - it's a fixed coverage pattern
# over the general search area, which is the whole point of "true
# visual search" over "known waypoints" (Zer0's explicit pick).
SEARCH_WAYPOINTS = [
    (-7.0, 0.0), (7.0, 0.0),
    (7.0, 3.0), (-7.0, 3.0),
    (-7.0, 6.0), (7.0, 6.0),
    (7.0, 9.0), (-7.0, 9.0),
]

# How close (in normalised image area, object_detector.py's z field) a
# target needs to appear before this calls it "locked on" and moves to
# pickup. There's no real calibrated distance behind this number - it
# was picked as a plausible "looks close in frame" threshold, not
# measured against a known stand-off distance. Tune this from the
# first live run's logs (deliver_payload logs target_area() every
# tick during APPROACH) rather than trusting it blind.
LOCK_AREA_PX = 4000.0

LATERAL_GAIN_M = 0.05     # per-tick lateral nudge toward center, at 10Hz -> ~0.5 m/s max
APPROACH_STEP_M = 0.03    # per-tick forward nudge AT FULL CENTERING, at 10Hz -> ~0.3 m/s max closing speed
LOST_LOCK_TICK_LIMIT = 20  # ~2s at 10Hz before giving up and resuming the sweep

# Oct 25, 2026 - Run 9's real fix. With the sensor-freshness bug (Run 8)
# actually fixed, the first live run with a genuinely up-to-date offset_x
# every tick showed it: offset_x drifted between 0.18 and 0.42 for the
# full 62-second APPROACH phase (never converging toward 0) while area
# stayed flat around 80-130 the entire time (never approached
# LOCK_AREA_PX=4000) - i.e. the drone moved roughly 18+ meters "forward"
# (APPROACH_STEP_M every single tick, unconditionally) without ever
# actually closing distance on the target. That's the "flies toward it
# but goes past it into a corner" behaviour Zer0 described, now with a
# real log to explain it: this control law always marched forward in
# the ASSUMED camera-forward axis (FORWARD_AXIS_SIGN*APPROACH_STEP_M,
# every tick, no matter how far off-centre the target is) while only
# gently nudging sideways to recentre. Forward is only actually "toward
# the target" once the target is centred in frame - marching ahead at
# full speed before that just flies past/beside it, a classic
# pursuit-curve failure mode in naive visual servoing.
#
# Checked against how this is normally done (image-based visual
# servoing / IBVS with area-as-depth feedback - see e.g. "Image-based
# Visual Servo Control for Aerial Manipulation Using a Fully-Actuated
# UAV", arXiv:2306.16530): the standard fix is "look-then-move" gating
# - scale the closing-rate axis by how well the steering axis is
# already converged, so the controller centres FIRST and only commits
# to closing distance once it's actually looking at the target. Applied
# below as CENTERING_FORWARD_GATE: forward speed is scaled down the
# further offset_x is from 0, with a floor (MIN_FORWARD_SCALE) so it
# never fully stalls if lateral gain alone can't perfectly zero it out.
CENTERING_SOFTEN = 0.4     # |offset_x| at/above this scales forward speed to the floor
MIN_FORWARD_SCALE = 0.15   # forward speed never drops below this fraction of APPROACH_STEP_M

# Drone's own approach/land point near the delivery pad - deliberately
# offset 0.8m east of the pad's centre (where the payload actually
# gets respawned) so the final LAND doesn't come down on top of the
# box this skill just placed there.
DELIVERY_APPROACH_XY = (0.8, -4.0)
DELIVERY_DROP_POINT = (0.0, -4.0, 0.19)  # pad top (z=0.04) + half the 0.3m box height

# Oct 25, 2026, later still - TRANSIT ("fly from wherever Shadow is to
# the delivery pad") is just a go_to call with no perception involved -
# no unknown target, no camera needed - which makes it the lowest-risk
# place in this whole mission to hand off from hand-coded flight to a
# TRAINED policy. See learned_flight.py's own module docstring for the
# full bridge design this reuses verbatim (same observation contract:
# goal-minus-position; same action contract: a clipped position
# delta). Best-effort, not a hard requirement - deliver_payload is the
# real mission here, and TRANSIT flying well is one leg of it, not the
# point of it, so a missing checkpoint or missing stable_baselines3
# install falls back to the ORIGINAL hand-coded straight-line flight
# below rather than ever failing the mission over it. Same reasoning
# if the policy genuinely can't converge in time: TRANSIT_LEARNED_TICK_BUDGET
# gives it a fair shot, then falls back rather than risking the whole
# run's timeout on one still-early-stage leg.
TRANSIT_CHECKPOINT = os.path.expanduser('~/shadow_ws/ml/models/go_to/go_to_ppo_final.zip')
TRANSIT_MAX_STEP_DELTA_M = 0.3   # same per-tick action scale go_to_env.py trained with
TRANSIT_LEARNED_TICK_BUDGET = 150  # ~15s at 10Hz before giving the hand-coded fallback a turn

# Known world layout used ONLY for bookkeeping - identifying which
# already-placed model to despawn by name once a lock-on happens near
# it. This is entity management, not navigation: nothing in SEARCH or
# APPROACH ever reads these coordinates to fly anywhere. Flying is
# driven purely by the fixed sweep pattern above and the live camera
# feedback in APPROACH; this dict only answers "which named gz entity
# is nearest where Shadow is hovering right now", which the
# world/remove service needs a name for and has no other way to supply.
KNOWN_ORANGE_PAYLOADS = {
    'payload_orange_1': (6.0, 2.0),
    'payload_orange_2': (-5.0, 7.0),
}

DELIVERED_PAYLOAD_SDF = """<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        <material>
          <ambient>0.9 0.6 0.1 1</ambient>
          <diffuse>0.9 0.6 0.1 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _distance_2d(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


class DeliverPayloadSkill(Skill):
    """Tier 3. requirements()/start()/update()/limits() per the usual
    Skill contract - run_skill.py's tick loop and timeout handling
    apply unchanged. A full search-approach-despawn-transit-respawn-
    land mission needs far more wall-clock time than any Tier 1/2
    skill, hence the much longer timeout_sec."""

    name = 'deliver_payload'
    timeout_sec = 300.0

    def __init__(self, body):
        super().__init__(body)
        self.sensor = None
        self._gz = None
        self._phase = 'TAKEOFF'
        self._takeoff_commanded = False
        self._wp_idx = 0
        self._approach_target = None
        self._lost_lock_ticks = 0
        self._land_commanded = False
        self._tick_count = 0
        self._transit_model = None
        self._transit_model_load_attempted = False
        self._transit_ticks = 0
        self._transit_fallback = False

    # ---- Skill contract ------------------------------------------------

    def requirements(self):
        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().error(
                'deliver_payload: no MAVROS connection to ArduPilot.')
            return False
        # 25.0, not 10.0 - see shadow/skills/takeoff.py's matching
        # comment: a live run on shadow_runway Oct 24, 2026 saw this
        # still timing out well after the FCU's own log reported GPS
        # in use.
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            self.body.get_logger().error(
                'deliver_payload: no position estimate yet.')
            return False
        return True

    def start(self):
        # Senses layer - subscribes through self.body, same pattern
        # every other Sense in this package uses.
        self.sensor = PayloadSensor(self.body)

        # This skill's own gz-transport node, separate from ROS2/MAVROS
        # entirely - see module docstring for why this file needs one
        # of its own rather than reusing anything from shadow.body.
        self._gz = GzNode()

        if not self.body.is_armed():
            self.body.set_mode('GUIDED')
            self.body.arm()
        self._takeoff_commanded = self.body.takeoff(SEARCH_ALTITUDE_M)

    def update(self):
        self._tick_count += 1

        if self._phase == 'TAKEOFF':
            return self._tick_takeoff()
        if self._phase == 'SEARCH':
            return self._tick_search()
        if self._phase == 'APPROACH':
            return self._tick_approach()
        if self._phase == 'DESPAWN':
            return self._tick_despawn()
        if self._phase == 'TRANSIT':
            return self._tick_transit()
        if self._phase == 'RESPAWN':
            return self._tick_respawn()
        if self._phase == 'LAND':
            return self._tick_land()

        # Unknown phase should never happen - fail loudly rather than
        # silently reporting completion.
        self.body.get_logger().error(
            f'deliver_payload: unknown phase {self._phase!r}')
        return False

    # ---- phases ---------------------------------------------------------

    def _tick_takeoff(self):
        if not self._takeoff_commanded:
            return False
        pos = self.body.current_position()
        if pos is None:
            return False
        if abs(pos[2] - SEARCH_ALTITUDE_M) <= TAKEOFF_ALT_TOLERANCE_M:
            self.body.get_logger().info(
                'deliver_payload: at search altitude, starting sweep.')
            self._phase = 'SEARCH'
        return False

    def _tick_search(self):
        if self.sensor.has_target():
            pos = self.body.current_position()
            self._approach_target = list(pos[:2]) if pos else list(
                SEARCH_WAYPOINTS[self._wp_idx % len(SEARCH_WAYPOINTS)])
            self.body.get_logger().info(
                'deliver_payload: target acquired during search - switching to APPROACH.')
            self._phase = 'APPROACH'
            return False

        tx, ty = SEARCH_WAYPOINTS[self._wp_idx % len(SEARCH_WAYPOINTS)]
        self.body.send_position_setpoint(tx, ty, SEARCH_ALTITUDE_M)
        pos = self.body.current_position()
        if pos is not None and _distance_2d(pos, (tx, ty)) <= WAYPOINT_TOLERANCE_M:
            self._wp_idx += 1
            if self._wp_idx % len(SEARCH_WAYPOINTS) == 0:
                self.body.get_logger().warning(
                    'deliver_payload: completed a full sweep pass with no '
                    'target found - restarting the pattern.')
        return False

    def _tick_approach(self):
        offset = self.sensor.target_offset()

        if offset is None:
            self._lost_lock_ticks += 1
            if self._lost_lock_ticks > LOST_LOCK_TICK_LIMIT:
                self.body.get_logger().warning(
                    'deliver_payload: lost lock during approach - resuming sweep.')
                self._lost_lock_ticks = 0
                self._phase = 'SEARCH'
                return False
            # Hold the last commanded point rather than drifting while
            # we wait to see if the target reappears.
            x, y = self._approach_target
            self.body.send_position_setpoint(x, y, SEARCH_ALTITUDE_M)
            return False

        self._lost_lock_ticks = 0
        offset_x, _offset_y = offset
        area = self.sensor.target_area()

        # Look-then-move gating (Run 9 fix, Oct 25 - see CENTERING_SOFTEN's
        # own comment above for the real log evidence and the IBVS
        # reasoning behind this): only commit to full forward speed once
        # the target is actually centred. 1.0 at offset_x=0, linearly down
        # to MIN_FORWARD_SCALE by |offset_x|=CENTERING_SOFTEN and beyond -
        # never fully zero, so a target lateral gain can't perfectly reach
        # (e.g. right at the edge of frame) still gets *some* forward
        # progress rather than deadlocking in place.
        forward_scale = max(
            MIN_FORWARD_SCALE,
            1.0 - min(abs(offset_x), CENTERING_SOFTEN) / CENTERING_SOFTEN,
        )

        if self._tick_count % 10 == 0:
            self.body.get_logger().info(
                f'deliver_payload: approaching - offset_x={offset_x:.2f} '
                f'area={area:.0f} forward_scale={forward_scale:.2f}')

        self._approach_target[0] += RIGHT_AXIS_SIGN * LATERAL_GAIN_M * offset_x
        self._approach_target[1] += FORWARD_AXIS_SIGN * APPROACH_STEP_M * forward_scale
        x, y = self._approach_target
        self.body.send_position_setpoint(x, y, SEARCH_ALTITUDE_M)

        if area >= LOCK_AREA_PX:
            self.body.get_logger().info(
                f'deliver_payload: locked on (area={area:.0f}) - picking up.')
            self._phase = 'DESPAWN'
        return False

    def _tick_despawn(self):
        pos = self.body.current_position()
        name = self._nearest_known_payload(pos)
        ok = self._remove_entity(name)
        self.body.get_logger().info(
            f'deliver_payload: despawn {name} -> {"ok" if ok else "FAILED"}')
        self._phase = 'TRANSIT'
        return False

    def _load_transit_model(self):
        """Lazily loads the trained go_to checkpoint the first time
        TRANSIT actually runs (not in start(), so a mission that never
        reaches TRANSIT - e.g. it never finds a payload - never pays
        this cost). See TRANSIT_CHECKPOINT's own comment above for why
        this is best-effort, not required: any failure here just means
        TRANSIT uses its original hand-coded straight-line flight."""
        self._transit_model_load_attempted = True
        if not os.path.exists(TRANSIT_CHECKPOINT):
            self.body.get_logger().info(
                f'deliver_payload: no trained checkpoint at {TRANSIT_CHECKPOINT} - '
                f'TRANSIT will use its original hand-coded straight-line flight.')
            return
        try:
            from stable_baselines3 import PPO
            self._transit_model = PPO.load(TRANSIT_CHECKPOINT)
            self.body.get_logger().info(
                'deliver_payload: TRANSIT will fly via the trained go_to policy '
                f'({os.path.basename(TRANSIT_CHECKPOINT)}), falling back to '
                'hand-coded straight-line flight only if it doesn\'t converge in time.')
        except Exception as exc:  # noqa: BLE001 - any load failure should fall
            # back cleanly to hand-coded flight, never crash the whole mission
            # over a training-stack problem (missing package, bad checkpoint).
            self.body.get_logger().warning(
                f'deliver_payload: could not load trained checkpoint for TRANSIT '
                f'({exc}) - using hand-coded straight-line flight instead.')
            self._transit_model = None

    def _tick_transit(self):
        tx, ty = DELIVERY_APPROACH_XY
        pos = self.body.current_position()

        if not self._transit_model_load_attempted:
            self._load_transit_model()

        use_learned = self._transit_model is not None and not self._transit_fallback

        if use_learned and pos is not None:
            # Same observation/action contract as learned_flight.py's
            # _tick_fly() - goal-minus-position in, a clipped position
            # delta out. deterministic=True for the same reason that
            # file's own comment gives: exploration noise is a training
            # tool, never something a real mission leg should fly with
            # switched on.
            goal = (tx, ty, SEARCH_ALTITUDE_M)
            obs = np.array(goal, dtype=np.float32) - np.array(pos, dtype=np.float32)
            action, _state = self._transit_model.predict(obs, deterministic=True)
            delta = np.clip(action, -TRANSIT_MAX_STEP_DELTA_M, TRANSIT_MAX_STEP_DELTA_M)
            target = (
                pos[0] + float(delta[0]),
                pos[1] + float(delta[1]),
                pos[2] + float(delta[2]),
            )
            self.body.send_position_setpoint(*target)
            self._transit_ticks += 1
            if (self._transit_ticks > TRANSIT_LEARNED_TICK_BUDGET
                    and _distance_2d(pos, (tx, ty)) > WAYPOINT_TOLERANCE_M):
                self.body.get_logger().warning(
                    f'deliver_payload: TRANSIT\'s learned policy hasn\'t converged '
                    f'in {TRANSIT_LEARNED_TICK_BUDGET} ticks - falling back to '
                    f'hand-coded straight-line flight for the rest of this leg.')
                self._transit_fallback = True
        else:
            # Original behaviour, unchanged - either no trained checkpoint
            # was available, or the learned attempt above gave up.
            self.body.send_position_setpoint(tx, ty, SEARCH_ALTITUDE_M)

        if pos is not None and _distance_2d(pos, (tx, ty)) <= WAYPOINT_TOLERANCE_M:
            self._phase = 'RESPAWN'
        return False

    def _tick_respawn(self):
        dx, dy, dz = DELIVERY_DROP_POINT
        ok = self._create_entity('payload_delivered', dx, dy, dz)
        self.body.get_logger().info(
            f'deliver_payload: respawn payload_delivered at pad -> {"ok" if ok else "FAILED"}')
        self._phase = 'LAND'
        return False

    def _tick_land(self):
        if not self._land_commanded:
            self.body.set_mode('LAND')
            self._land_commanded = True
        return not self.body.is_armed()

    # ---- gz-transport world editing ------------------------------------

    def _nearest_known_payload(self, pos):
        if pos is None:
            return next(iter(KNOWN_ORANGE_PAYLOADS))
        best_name, best_dist = None, float('inf')
        for name, (px, py) in KNOWN_ORANGE_PAYLOADS.items():
            d = _distance_2d(pos, (px, py))
            if d < best_dist:
                best_name, best_dist = name, d
        return best_name

    def _remove_entity(self, name):
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        result, _response = self._gz.request(
            f'/world/{WORLD_NAME}/remove', req, Entity, Boolean, 2000)
        return bool(result)

    def _create_entity(self, name, x, y, z):
        req = EntityFactory()
        req.sdf = DELIVERED_PAYLOAD_SDF.format(name=name, x=x, y=y, z=z)
        req.name = name
        req.allow_renaming = True
        result, _response = self._gz.request(
            f'/world/{WORLD_NAME}/create', req, EntityFactory, Boolean, 2000)
        return bool(result)
