#!/usr/bin/env python3
"""
Body layer - the one place that actually talks to ArduPilot over MAVROS.

Nothing above this file (Senses, Skills, Mind) should ever import
mavros_msgs or mention a flight mode string directly. If a Skill needs
Shadow to arm, climb, move, or land, it calls a method here. That's
the whole point of the Body/Senses/Skills/Mind split: swapping sim for
the real Pi, or ArduPilot for something else entirely, only ever means
rewriting this one file.

Mode names and the arm -> takeoff sequence below are ArduCopter's, not
PX4's - this replaces the old PX4-flavoured OFFBOARD dance in
mission_planner.py (archive/mission_planner.py once this lands) now
that Shadow runs ArduCopter SITL. Confirmed against the real MAVProxy
flight sequence that actually flew in Gate 1/Gate 2:

    mode guided
    arm throttle
    takeoff 5

This class is the MAVROS equivalent of typing those three lines by
hand: set_mode('GUIDED'), arm(), takeoff(5.0) - built from the same
services/topics mission_planner.py already proved work:
/mavros/state, /mavros/local_position/pose, /mavros/cmd/arming,
/mavros/set_mode, /mavros/setpoint_raw/local. The one new one is
/mavros/cmd/takeoff (CommandTOL), since ArduCopter's own takeoff
service is the faithful MAVROS equivalent of MAVProxy's `takeoff 5`
command - climbing out via a raw position setpoint from a standing
start is a different manoeuvre and isn't what Gate 1/2 actually
proved.

Everything here is synchronous and blocking (spin_until_future_complete),
on purpose: a Skill's start()/update() calls need a plain true/false or
a current value back, not a future to juggle. If this ever needs to
run multiple skills concurrently, that's a reason to revisit this
choice, not a reason to pre-build async plumbing nobody's using yet.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class ArduPilotBody(Node):
    """One ROS2 node, one drone. A Skill is handed an instance of this
    and never talks to MAVROS directly."""

    def __init__(self, node_name='shadow_body'):
        super().__init__(node_name)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state = State()
        self.create_subscription(State, '/mavros/state', self._state_cb, qos)

        self.pose = PoseStamped()
        self.have_pose = False
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_cb, qos)

        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', qos)

        self._arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self._set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        self._takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff')

    # ---- readiness ---------------------------------------------------

    def wait_until_connected(self, timeout_sec=30.0):
        """Blocks until MAVROS reports a live link to ArduPilot. Returns
        False on timeout instead of spinning forever, so a Skill's
        requirements() check can fail cleanly rather than hang the
        trainer."""
        self._arming_client.wait_for_service(timeout_sec=timeout_sec)
        self._set_mode_client.wait_for_service(timeout_sec=timeout_sec)
        self._takeoff_client.wait_for_service(timeout_sec=timeout_sec)

        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not self.state.connected:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.5)
        return True

    def wait_for_position_estimate(self, timeout_sec=15.0):
        """Same reasoning mission_planner.py already documented: arming
        before a real position estimate exists is why a drone has spun
        in place without climbing before. Don't skip this."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not self.have_pose:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.5)
        return True

    # ---- commands ------------------------------------------------------

    def set_mode(self, mode):
        """mode is an ArduCopter mode string - 'GUIDED', 'LAND',
        'STABILIZE', etc. Returns True only if ArduPilot actually
        accepted the request (mode_sent), which just means the message
        was received, not that the mode switch necessarily stuck -
        same caveat mission_planner.py already flagged for PX4."""
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        ok = bool(result and result.mode_sent)
        if not ok:
            self.get_logger().error(f'set_mode({mode}) was not accepted.')
        return ok

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self._arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        ok = bool(result and result.success)
        if not ok:
            self.get_logger().error('Arming request failed.')
        return ok

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self._arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        return bool(result and result.success)

    def takeoff(self, altitude_m):
        """The real ArduCopter takeoff command (CommandTOL), the MAVROS
        equivalent of typing `takeoff 5` at the MAVProxy prompt. Must
        already be armed and in GUIDED mode - ArduPilot rejects it
        otherwise."""
        req = CommandTOL.Request()
        req.altitude = float(altitude_m)
        future = self._takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        ok = bool(result and result.success)
        if not ok:
            self.get_logger().error(f'takeoff({altitude_m}m) was rejected.')
        return ok

    def send_position_setpoint(self, x, y, z, yaw=None):
        """Streams one position setpoint in MAVROS's local ENU frame,
        relative to the arm point - same frame and message type
        mission_planner.py used for PX4, which MAVROS abstracts the
        same way for ArduCopter. Must be called repeatedly (ideally
        ~20Hz) while in GUIDED mode - a single setpoint with nothing
        following it is not a reliable way to get or keep ArduPilot's
        attention.

        Oct 25, 2026 fix - the real mavros_msgs/PositionTarget.msg bit
        values are IGNORE_YAW=1024 (bit 10) and IGNORE_YAW_RATE=2048
        (bit 11). This used to hand-roll those as two binary literals -
        0b100000000000 (2048, bit 11) labelled "IGNORE_YAW", and
        0b1000000000000 (4096, bit 12 - not a defined flag at all)
        labelled "IGNORE_YAW_RATE". Net effect: the real IGNORE_YAW bit
        (1024) was never actually present in the outgoing type_mask, so
        every setpoint also carried an explicit yaw target - and since
        `target.yaw` was never set either, that target was always 0.0
        rad (true North), continuously fighting Shadow's actual
        spawn/operating yaw (90 degrees - see deliver_payload.py's
        module docstring, assumption 1, which assumes yaw never changes
        and is exactly why it never should have). This is almost
        certainly both the "doesn't really like pan" behaviour seen
        during go_to RL training (go_to.py calls this same method every
        tick) and a real contributor to deliver_payload's APPROACH
        phase getting stuck on an identical, frozen offset_x/area
        reading for 17+ seconds before aborting - if yaw keeps getting
        yanked toward 0 while the approach math assumes a fixed 90
        degree heading, the camera ends up looking somewhere that math
        doesn't expect. Using the message's own real constants instead
        of hand-rolled literals removes this whole bug class rather
        than just patching two numbers.

        Nov 5, 2026 - added the optional `yaw` parameter. Every
        existing caller (go_to.py, agility_circuit_env.py,
        deliver_payload.py) calls this with no third argument and gets
        EXACTLY the Oct 25 behaviour, unchanged: IGNORE_YAW and
        IGNORE_YAW_RATE both set, target.yaw left at its default (0.0)
        and ignored by ArduPilot either way. Passing an actual
        yaw_rad clears both ignore bits and streams that heading as a
        real commanded setpoint, for skills (multi_target_env.py) where
        the policy needs to control heading, not just position. This
        was never a flag sitting there half-used - before this change
        nothing in the codebase could command yaw through this method
        at all, ignored or otherwise.

        Nov 6, 2026 - found the same bug class a third time, one layer
        deeper: type_mask never ignored velocity or acceleration
        either, and target.velocity/target.acceleration were never
        assigned (so they silently default to exactly zero). Per
        MAVLink's SET_POSITION_TARGET_LOCAL_NED convention, a field
        that isn't marked ignored is a real commanded value, not "I
        don't care" - so every setpoint sent here was telling
        ArduCopter "go to this position" AND "hold exactly zero
        velocity" AND "hold exactly zero acceleration" at the same
        time, fighting itself the same way the yaw/yaw_rate bugs above
        did. Standard pure-position-control convention (every mavros/
        MAVSDK goto example) ignores velocity and acceleration too, so
        the flight controller derives them itself from the position
        error instead of being handed a conflicting zero target. Zer0
        flagged the flight during generate_dataset.py's rapid
        re-targeting as visibly rougher than anything the Skills-layer
        testing (go_to/hover/etc.) showed - far more aggressive
        re-targeting than those exercised, and exactly the usage
        pattern this would bite hardest."""
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        ignore_vel_accel = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        )
        if yaw is None:
            target.type_mask = (
                ignore_vel_accel |
                PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
            )
        else:
            # Nov 5, 2026 fix - this originally left type_mask at its
            # default (0, nothing ignored) whenever a yaw was given,
            # which meant the message claimed to carry BOTH a real yaw
            # target AND a real yaw_rate target - and target.yaw_rate
            # was never set, so that second claim was always "hold
            # exactly 0 rad/s," fighting the first claim any time the
            # commanded yaw differs from the current one (which is most
            # ticks, since callers pick a fresh heading each time, not
            # a smooth ramp). Confirmed live: generate_dataset.py
            # started flipping Shadow over after roughly 100 frames of
            # exactly this - wide random-yaw viewpoints. Only yaw_rate
            # needs to be ignored here, not yaw itself - this is the
            # mirror image of the Oct 25 fix above (that one needed
            # IGNORE_YAW set when yaw wasn't actually being commanded;
            # this one needs IGNORE_YAW_RATE set when yaw_rate isn't).
            target.type_mask = ignore_vel_accel | PositionTarget.IGNORE_YAW_RATE
            target.yaw = float(yaw)
        target.position.x = float(x)
        target.position.y = float(y)
        target.position.z = float(z)
        self.setpoint_pub.publish(target)

    def stream_position_for(self, x, y, z, seconds, rate_hz=20):
        cycles = int(seconds * rate_hz)
        for _ in range(cycles):
            self.send_position_setpoint(x, y, z)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

    # ---- current state, for Skills' update()/limits() checks ----------

    def current_position(self):
        """(x, y, z) in the same local ENU frame as send_position_setpoint,
        or None if no position estimate has arrived yet."""
        if not self.have_pose:
            return None
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def current_yaw(self):
        """Current heading in radians, ENU convention (0 = facing +x,
        positive = counter-clockwise viewed from above), decoded from
        MAVROS's own pose quaternion. Returns None under the same
        condition current_position() does - no pose estimate yet.

        Added Nov 5, 2026 alongside send_position_setpoint's new yaw
        parameter - until now nothing read this back either; Skills
        and environments only ever asked where Shadow was, never which
        way it was facing. Standard quaternion-to-yaw extraction
        (atan2 of the z-rotation terms), same convention ArduPilot/
        MAVROS use for PositionTarget.yaw, so a value read here and a
        value sent to send_position_setpoint's yaw argument are
        directly comparable without any extra conversion."""
        if not self.have_pose:
            return None
        q = self.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def current_orientation_quat(self):
        """(x, y, z, w) body-to-world orientation, straight off MAVROS's
        pose - the SAME quaternion current_yaw() above reads, just not
        thrown away down to one angle. Returns None under the same
        condition current_position() does - no pose estimate yet.

        Added Nov 6, 2026 for generate_dataset.py's box labelling: Shadow
        has no gimbal (deliver_payload.py's own documented assumption -
        the front camera is rigid on the airframe), so the camera's real
        attitude is the full body attitude, pitch/roll included, not
        just its heading. ArduCopter pitches/rolls the whole airframe to
        hold or correct a position, so a hovering/correcting Shadow is
        essentially never exactly level - current_yaw() alone can't
        reconstruct that, it already discards the pitch/roll terms this
        reads out whole."""
        if not self.have_pose:
            return None
        q = self.pose.pose.orientation
        return (q.x, q.y, q.z, q.w)

    def is_armed(self):
        return bool(self.state.armed)

    def current_mode(self):
        return self.state.mode

    # ---- callbacks ------------------------------------------------

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self.have_pose = True
