#!/usr/bin/env python3
"""
Telemetry bridge - subscribes to MAVROS, writes real simulation flight
data into the R&D database's `simulated_telemetry` table, tagged to a
tracked `simulation_runs` row.

CORRECTED AGAIN: the previous version of this file wrote into
legacy_shadow.db - that was wrong. Per the real, authoritative schema
(zero-command-system/scripts/init_databases.py), simulated_telemetry
and simulation_runs belong in ZERO_GRAVITY_RND_DB (zerogravity_rnd.db).
legacy_shadow.db is reserved exclusively for real physical hardware
tables (vehicle, telemetry, missions, hardware_health, faults,
maintenance, battery_cycles) - simulation data should never go there.

Real MAVROS topics used, each confirmed against MAVROS's actual plugin
list from tonight's own logs, not guessed:
  /mavros/state                    -> flight_mode
  /mavros/battery                  -> battery_voltage, battery_percentage
  /mavros/global_position/global   -> latitude, longitude
  /mavros/local_position/pose      -> altitude (relative, local z), roll, pitch, yaw (via quaternion)
  /mavros/local_position/velocity_local -> velocity_x/y/z
"""

import math
import signal
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped

DB_PATH = Path.home() / "zeroGravity-rnd" / "lab" / "database" / "zerogravity_rnd.db"

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

        self._reconcile_stale_runs()

        self.simulation_run_id = self._start_simulation_run()

        self.timer = self.create_timer(WRITE_INTERVAL_SECONDS, self._write_row)
        self.get_logger().info(f'Telemetry bridge started, run id '
                                f'{self.simulation_run_id}, writing to '
                                f'{DB_PATH} every {WRITE_INTERVAL_SECONDS}s')

    def _reconcile_stale_runs(self):
        """Close out any simulation_runs rows still marked 'running' from a
        previous session that never shut down cleanly (killed terminal,
        closed Gazebo, SIGKILL, machine sleep, etc). A stale run's real end
        time is approximated from the last telemetry sample actually logged
        for it - the best available evidence of when it really stopped -
        falling back to its own started_at (i.e. zero duration) if it never
        got that far. Marked 'interrupted', not 'completed', since
        destroy_node() never actually confirmed a clean shutdown for these.
        Runs on every startup, so this never lets stale rows pile back up.
        """
        try:
            conn = sqlite3.connect(str(DB_PATH))
            try:
                stale = conn.execute(
                    "SELECT id, started_at FROM simulation_runs WHERE status = 'running'"
                ).fetchall()
                for run_id, started_at in stale:
                    last_sample = conn.execute(
                        "SELECT MAX(timestamp) FROM simulated_telemetry WHERE simulation_run_id = ?",
                        (run_id,),
                    ).fetchone()[0]
                    conn.execute(
                        "UPDATE simulation_runs SET status = ?, completed_at = ? WHERE id = ?",
                        ('interrupted', last_sample or started_at, run_id),
                    )
                conn.commit()
                if stale:
                    self.get_logger().info(
                        f'Reconciled {len(stale)} stale "running" simulation_runs row(s) '
                        f'from previous session(s) into "interrupted".'
                    )
            finally:
                conn.close()
        except sqlite3.Error as e:
            self.get_logger().error(f'Failed to reconcile stale simulation_runs rows: {e}')

    def _start_simulation_run(self):
        conn = sqlite3.connect(str(DB_PATH))
        cursor = conn.execute(
            """
            INSERT INTO simulation_runs (
                simulator, scenario_name, vehicle_model, status, started_at
            ) VALUES (?, ?, ?, ?, ?)
            """,
            (
                # Oct 26, 2026 (even later) - 'gazebo-classic'/'living_room'
                # were stale from the old PX4 Gazebo-Classic stack, never
                # updated when the project moved to ArduPilot + Gazebo
                # Harmonic (see simulation_control.py's own ALLOWED_WORLDS
                # comment - shadow_runway is the only world that actually
                # exists now). Every report in the Training Log was
                # correctly logging real telemetry, just mislabeling which
                # simulator/world it came from - caught from Zer0's own
                # "it's not in my living room... that's a bug right there."
                'ardupilot-gazebo',
                'shadow_runway',
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
        # altitude no longer comes from here (see _pose_cb below) - this
        # message's altitude is global/MSL, dominated by whatever the
        # SITL home location's elevation happens to be (logged here as
        # ~603-606m for the living_room scenario). A real climb/hover/
        # descent only ever moved that number by a few metres on top of
        # a 600+ metre offset, which is why the Training Log's altitude
        # chart looked flat and meaningless - it was technically correct,
        # just plotting the wrong field for "how high is Shadow right
        # now."

    def _pose_cb(self, msg):
        # local_position/pose's z is relative to wherever Shadow's EKF
        # origin/home was set this run (ENU local frame) - actually "how
        # high above the floor," which is what a hover/landing profile
        # in the Training Log should show. Oct 26, 2026 (even later) fix.
        self.altitude = msg.pose.position.z
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

    def _handle_sigterm(signum, frame):
        # Translate a SIGTERM (e.g. a process manager or `kill` stopping this
        # node) into the same clean-shutdown path a Ctrl+C already takes, so
        # destroy_node() gets a chance to mark this run 'completed' instead
        # of leaving it stuck as 'running' for the next startup to reconcile.
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
