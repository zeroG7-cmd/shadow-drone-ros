#!/usr/bin/env python3
"""
Telemetry bridge - subscribes to MAVROS, writes real simulation flight
data into Zero Command's `simulated_telemetry` table (legacy_shadow.db),
tagged to a tracked `simulation_runs` row.

CORRECTED from an earlier version of this script, which wrote into the
plain `telemetry` table - that table is architecturally reserved for
real physical hardware telemetry, never simulation data. This version
uses the schema as it was actually designed: a `simulation_runs` row
created once at startup, with each telemetry sample tagged to that
run's id via `simulated_telemetry`. Keeps simulated and real flight
data permanently separated, which matters the moment real hardware
telemetry exists and needs to never be mixed with simulated numbers.

Real MAVROS topics used, each confirmed against MAVROS's actual plugin
list from tonight's own logs, not guessed:
  /mavros/state                    -> flight_mode
  /mavros/battery                  -> battery_voltage, battery_percentage
  /mavros/global_position/global   -> latitude, longitude, altitude
  /mavros/local_position/pose      -> roll, pitch, yaw (via quaternion)
  /mavros/local_position/velocity_local -> velocity_x/y/z

One honest gap: `link_quality` (in the real-hardware `telemetry` table
only - simulated_telemetry doesn't have this column at all, correctly,
since link quality is meaningless for a simulated vehicle).
"""

import math
import sqlite3
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped

DB_PATH = Path.home() / "zeroGravity-rnd" / "lab" / "database" / "legacy_shadow.db"

WRITE_INTERVAL_SECONDS = 1.0


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.flight_mode = None
        self.battery_voltage = None
        self.battery_percentage = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.velocity_x = None
        self.velocity_y = None
        self.velocity_z = None

        self.create_subscription(State, '/mavros/state', self._state_cb, qos)
        self.create_subscription(BatteryState, '/mavros/battery', self._battery_cb, qos)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self._gps_cb, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self._velocity_cb, qos)

        if not DB_PATH.parent.exists():
            self.get_logger().warn(f'Database folder does not exist: {DB_PATH.parent} '
                                    '- check DB_PATH matches config.py')

        self.simulation_run_id = self._start_simulation_run()

        self.timer = self.create_timer(WRITE_INTERVAL_SECONDS, self._write_row)
        self.get_logger().info(f'Telemetry bridge started, run id '
                                f'{self.simulation_run_id}, writing to '
                                f'{DB_PATH} every {WRITE_INTERVAL_SECONDS}s')

    def _start_simulation_run(self):
        """Create one simulation_runs row for this session - every
        telemetry sample below gets tagged to this run's id, so runs
        stay distinguishable from each other later."""
        conn = sqlite3.connect(str(DB_PATH))
        cursor = conn.execute(
            """
            INSERT INTO simulation_runs (
                simulator, scenario_name, vehicle_model, status, started_at
            ) VALUES (?, ?, ?, ?, ?)
            """,
            (
                'gazebo-classic',
                'living_room',
                'shadow',
                'running',
                datetime.now(timezone.utc).isoformat(),
            ),
        )
        conn.commit()
        run_id = cursor.lastrowid
        conn.close()
        return run_id

    def _state_cb(self, msg):
        self.flight_mode = msg.mode

    def _battery_cb(self, msg):
        self.battery_voltage = msg.voltage
        if msg.percentage is not None and msg.percentage >= 0:
            self.battery_percentage = msg.percentage * 100.0

    def _gps_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def _pose_cb(self, msg):
        q = msg.pose.orientation
        self.roll, self.pitch, self.yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

    def _velocity_cb(self, msg):
        self.velocity_x = msg.twist.linear.x
        self.velocity_y = msg.twist.linear.y
        self.velocity_z = msg.twist.linear.z

    def _write_row(self):
        try:
            conn = sqlite3.connect(str(DB_PATH))
            conn.execute(
                """
                INSERT INTO simulated_telemetry (
                    simulation_run_id, timestamp,
                    battery_voltage, battery_percentage,
                    latitude, longitude, altitude,
                    velocity_x, velocity_y, velocity_z,
                    roll, pitch, yaw, flight_mode
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    self.simulation_run_id,
                    datetime.now(timezone.utc).isoformat(),
                    self.battery_voltage, self.battery_percentage,
                    self.latitude, self.longitude, self.altitude,
                    self.velocity_x, self.velocity_y, self.velocity_z,
                    self.roll, self.pitch, self.yaw,
                    self.flight_mode,
                ),
            )
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            self.get_logger().error(f'Failed to write telemetry row: {e}')

    def destroy_node(self):
        try:
            conn = sqlite3.connect(str(DB_PATH))
            conn.execute(
                "UPDATE simulation_runs SET status = ?, completed_at = ? WHERE id = ?",
                ('completed', datetime.now(timezone.utc).isoformat(), self.simulation_run_id),
            )
            conn.commit()
            conn.close()
        except sqlite3.Error:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

