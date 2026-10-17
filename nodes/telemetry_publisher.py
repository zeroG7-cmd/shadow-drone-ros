#!/usr/bin/env python3
"""
telemetry_publisher.py - continuously writes Shadow's live sim state to
a JSON file, so the Digital Lab's three.js frontend (running inside
Zero Command, a separate PyInstaller-frozen Python process) can show a
real physics mirror without ever importing rclpy itself.

Same trick progress.json already uses, just continuous instead of
once-per-run: ROS2/rclpy is genuinely painful to bundle inside
something like PyInstaller (ament's message/plugin discovery expects a
normal filesystem layout, not a frozen bundle), so the Flask backend
should never try to talk to ROS2 directly. This script is the one
process that actually does - a plain file is the handoff point.

NOT yet reconciled against nodes/telemetry_bridge.py, which already
exists in this repo and is listed in the roadmap as "untouched, stays"
- it may already do some of this. Read that file before wiring this
one into CMakeLists.txt/launch - don't end up with two things doing
the same job.

Started Oct 3, 2026 after Zer0 was clear that "pose and state" wasn't
enough - he wants a genuine physics mirror: real motion (not just
position), the real motor commands driving each rotor, and the real
sensor noise ArduPilot's EKF is filtering out under the hood. All four
topics below are confirmed to exist from MAVROS's own documentation
(see the roadmap doc for links) - this is not a guess at topic names:

  /mavros/local_position/odom  (nav_msgs/Odometry)
      Position, orientation, AND both linear and angular velocity in
      one message. The angular velocity is what lets three.js actually
      bank/pitch the model in real time instead of just sliding a box
      between waypoints.

  /mavros/rc/out  (mavros_msgs/RCOut)
      ArduPilot's real SERVO_OUTPUT_RAW - the actual PWM value being
      sent to each of Shadow's four rotors, live. This is the real
      control signal, not a measured RPM off a physical tachometer
      (SITL doesn't have one) - rotor-spin visuals driven by this are
      honest ("what ArduPilot commanded"), not literally "measured
      blade speed". Worth keeping that distinction in the UI copy
      later, not overselling it as RPM.

  /mavros/imu/data  (sensor_msgs/Imu)
      ArduPilot's EKF-filtered, fused IMU estimate.

  /mavros/imu/data_raw  (sensor_msgs/Imu)
      The actual noisy accelerometer/gyro reading, before filtering.
      The gap between this and imu/data, moment to moment, IS real
      sensor noise being filtered out in front of your eyes - a
      genuine signal to render (jitter, or a numeric readout), not
      something faked for show.

NOT YET VERIFIED: whether ArduPilot/MAVROS is actually streaming all
four of these at a usable rate on this machine's current setup, or
whether some are stuck at a low default rate (common MAVLink gotcha -
a message only arrives as fast as something has asked for it). Check
this live before trusting the output rate; don't assume.
"""

import argparse
import json
import os
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import State, RCOut
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

DEFAULT_OUTPUT_PATH = os.path.expanduser(
    '~/shadow_sim_runtime/telemetry/latest.json')
DEFAULT_RATE_HZ = 20.0


def _vector3_to_dict(v):
    return {'x': v.x, 'y': v.y, 'z': v.z}


def _quaternion_to_dict(q):
    return {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}


def _imu_to_dict(msg):
    return {
        'stamp_sec': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        'linear_acceleration': _vector3_to_dict(msg.linear_acceleration),
        'angular_velocity': _vector3_to_dict(msg.angular_velocity),
    }


class TelemetryPublisher(Node):
    """Subscribes once, caches the latest of each message, writes the
    whole snapshot out on a timer - deliberately decoupled from
    individual message arrival rates, so one slow topic can't stall
    the others or the file write."""

    def __init__(self, output_path, rate_hz):
        super().__init__('shadow_telemetry_publisher')
        self.output_path = output_path
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._state = State()
        self._odom = None
        self._rc_out = None
        self._imu_filtered = None
        self._imu_raw = None

        self.create_subscription(State, '/mavros/state', self._state_cb, qos)
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self._odom_cb, qos)
        self.create_subscription(RCOut, '/mavros/rc/out', self._rc_out_cb, qos)
        self.create_subscription(
            Imu, '/mavros/imu/data', self._imu_filtered_cb, qos)
        self.create_subscription(
            Imu, '/mavros/imu/data_raw', self._imu_raw_cb, qos)

        self.create_timer(1.0 / rate_hz, self._write_snapshot)

        # Added after Zer0 ran this the first time and got a silent
        # terminal with nothing to show whether it was working or
        # hung - matches telemetry_bridge.py's own habit of logging a
        # clear startup line instead of running completely silent.
        self.get_logger().info(
            f'shadow_telemetry_publisher started, writing {self.output_path} '
            f'at {rate_hz}Hz. Waiting for MAVROS...')

    # ---- callbacks - just cache the latest, no computation here ------

    def _state_cb(self, msg):
        was_connected = self._state.connected
        self._state = msg
        if msg.connected and not was_connected:
            # One-time transition log, not every message (State
            # publishes continuously) - this is the actual answer to
            # "is it stuck or just quiet": the moment this line
            # appears, MAVROS has a real link to ArduPilot.
            self.get_logger().info('Connected to ArduPilot via MAVROS.')

    def _odom_cb(self, msg):
        self._odom = msg

    def _rc_out_cb(self, msg):
        self._rc_out = msg

    def _imu_filtered_cb(self, msg):
        self._imu_filtered = msg

    def _imu_raw_cb(self, msg):
        self._imu_raw = msg

    # ---- the one thing this node does on its own clock ----------------

    def _write_snapshot(self):
        snapshot = {
            'connected': bool(self._state.connected),
            'armed': bool(self._state.armed),
            'mode': self._state.mode,
        }

        if self._odom is not None:
            p = self._odom.pose.pose.position
            q = self._odom.pose.pose.orientation
            lin = self._odom.twist.twist.linear
            ang = self._odom.twist.twist.angular
            snapshot['odometry'] = {
                'position': {'x': p.x, 'y': p.y, 'z': p.z},
                'orientation': _quaternion_to_dict(q),
                'linear_velocity': _vector3_to_dict(lin),
                'angular_velocity': _vector3_to_dict(ang),
            }

        if self._rc_out is not None:
            # channels is a plain list of raw PWM ints (typically
            # ~1000-2000) - channel order matches Shadow's motor
            # mapping in the SDF, not yet cross-checked here.
            snapshot['rc_out'] = {'channels': list(self._rc_out.channels)}

        if self._imu_filtered is not None:
            snapshot['imu_filtered'] = _imu_to_dict(self._imu_filtered)

        if self._imu_raw is not None:
            snapshot['imu_raw'] = _imu_to_dict(self._imu_raw)

        # Write to a temp file in the same directory, then atomically
        # rename over the real path - a reader (the Flask route) can
        # never observe a half-written file this way, no matter how
        # badly its read and this timer happen to race.
        directory = os.path.dirname(self.output_path)
        fd, tmp_path = tempfile.mkstemp(dir=directory, suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(snapshot, f)
            os.replace(tmp_path, self.output_path)
        except Exception:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
            raise


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-file', default=DEFAULT_OUTPUT_PATH)
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                         help='Snapshot write rate in Hz (default: %(default)s)')
    parsed = parser.parse_args(args)

    rclpy.init()
    node = TelemetryPublisher(parsed.output_file, parsed.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
