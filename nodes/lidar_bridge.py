#!/usr/bin/env python3
"""
LiDAR bridge - Shadow's 2D LiDAR (RPLIDAR A2M12 in the real build,
the lidar_2d gpu_lidar sensor in model.sdf in the sim).

Nov 9, 2026 - new. Same pattern as object_detector.py / the camera:
subscribe straight to Gazebo's gz-transport topic (no ros_gz_bridge
process), then fan the scan out three ways:

  1. /shadow/scan  (sensor_msgs/LaserScan, frame lidar_link)
     The full-resolution scan in normal ROS convention (0 = nose,
     counter-clockwise positive). This is what slam_toolbox will read
     in Round 2 (mapping), and what any Senses-layer skill can read.

  2. /mavros/obstacle/send  (sensor_msgs/LaserScan, 72 x 5 deg)
     Fed to MAVROS's obstacle_distance plugin, which turns it into a
     MAVLink OBSTACLE_DISTANCE message for ArduPilot (PRX1_TYPE 2).
     That plugin does NO frame conversion: it copies ranges straight
     into the message and assumes they are already body FRD, i.e.
     clockwise from the nose. So this bridge re-bins the scan into 72
     clockwise 5 degree sectors (minimum range per sector), and sets
     the plugin's mav_frame parameter to BODY_FRD at startup. Without
     that parameter the plugin defaults to GLOBAL and ArduPilot would
     treat bearings as compass bearings: obstacles would appear
     rotated by the drone's heading.

  3. ~/shadow_sim_runtime/telemetry/scan.json  (~4 Hz)
     A small summary for the Zero Command dashboard radar: 360 one
     degree bins (clockwise from the nose, metres, null = no return),
     the nearest hit and its bearing, and 8 sector minimums. The
     dashboard never talks ROS, same as the other telemetry files.

Self filter: a 2D LiDAR on top of a quad can see its own props/arms.
Anything closer than SELF_FILTER_M is dropped as "no return".
Check the startup log line "nearest raw return" with nothing around
the drone: if it is below ~0.5 m, that is the airframe, and
SHADOW_LIDAR_SELF_FILTER_M should be raised above it.

Ground filter (Nov 9, after the first live run): a flat 2D scan plane
tilts with the drone. At 2 m altitude a 15 deg tilt puts the beam on the
ground ~7.5 m away, which showed up as phantom arcs on the radar and as
fake obstacles for ArduPilot. Using MAVROS's pose (height + attitude),
every ray that points downward is checked against where it would hit
flat ground; returns at or beyond that distance are dropped. Tilt and
the number of dropped rays go into scan.json so the radar can show them.

TF for mapping (Nov 9, map_area skill): publishes odom -> base_footprint
(x, y, yaw from MAVROS local position, stamped with each scan so
slam_toolbox gets an exact match) and a static base_footprint ->
lidar_link. See config/slam_toolbox_shadow.yaml for the frame layout.

Launched with plain python3 by run_full_sim.sh (no colcon needed).
"""

import json
import math
import os
import sys
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402

from gz.transport13 import Node as GzNode  # noqa: E402
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan  # noqa: E402

GZ_TOPIC = os.environ.get("SHADOW_LIDAR_GZ_TOPIC", "shadow/lidar/scan")
FRAME_ID = "lidar_link"  # matches model.sdf <gz_frame_id>
SELF_FILTER_M = float(os.environ.get("SHADOW_LIDAR_SELF_FILTER_M", "0.30"))

FCU_SECTORS = 72                 # OBSTACLE_DISTANCE holds 72 distances
FCU_STEP_DEG = 360.0 / FCU_SECTORS
MAVROS_OBSTACLE_NODE = "/mavros/obstacle"   # MAVROS ROS2 plugin sub-node
MAVROS_FRAME = "BODY_FRD"

LIDAR_HEIGHT_M = 0.096   # lidar above base_link (model.sdf pose)
GROUND_FRAC = 0.90       # return >= 90% of predicted ground distance = ground
POSE_STALE_S = 1.0

SUMMARY_PATH = os.path.expanduser("~/shadow_sim_runtime/telemetry/scan.json")
SUMMARY_PERIOD_S = 0.25
SECTOR_NAMES = ["front", "front_right", "right", "back_right",
                "back", "back_left", "left", "front_left"]


def cw_bearing_deg(ccw_rad):
    """ROS angle (CCW from nose, radians) -> clockwise bearing 0..360."""
    return (-math.degrees(ccw_rad)) % 360.0


class LidarBridge(Node):
    def __init__(self):
        super().__init__("lidar_bridge")
        self.scan_pub = self.create_publisher(LaserScan, "/shadow/scan", qos_profile_sensor_data)
        self.fcu_pub = self.create_publisher(LaserScan, "/mavros/obstacle/send", 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self.pose_cb, qos_profile_sensor_data)
        self._pose = None  # (t, x, y, z, qx, qy, qz, qw)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "base_footprint"
        st.child_frame_id = FRAME_ID
        st.transform.rotation.w = 1.0
        self.static_tf.sendTransform(st)
        self.param_client = self.create_client(
            SetParameters, MAVROS_OBSTACLE_NODE + "/set_parameters")

        self._lock = threading.Lock()
        self._pending = None
        self._new = threading.Event()
        self.frame_set = False
        self._frame_future = None
        self._last_summary = 0.0
        self._count = 0
        self._t0 = time.time()
        self._first_logged = False
        self._ground_dropped, self._tilt = 0, None

        os.makedirs(os.path.dirname(SUMMARY_PATH), exist_ok=True)
        self.create_timer(2.0, self.ensure_mavros_frame)
        self.create_timer(10.0, self.log_rate)
        threading.Thread(target=self.worker_loop, daemon=True).start()
        self.get_logger().info(
            f'LiDAR bridge up: gz "{GZ_TOPIC}" -> /shadow/scan, '
            f'/mavros/obstacle/send ({FCU_SECTORS} x {FCU_STEP_DEG:g} deg FRD), {SUMMARY_PATH}. '
            f'Self filter {SELF_FILTER_M} m.')

    # ---- MAVROS obstacle plugin frame -------------------------------
    def ensure_mavros_frame(self):
        if self.frame_set:
            return
        if self._frame_future is not None and not self._frame_future.done():
            return
        if not self.param_client.service_is_ready():
            return  # MAVROS not up yet; try again in 2 s
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="mav_frame",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=MAVROS_FRAME))]
        self._frame_future = self.param_client.call_async(req)
        self._frame_future.add_done_callback(self._frame_done)

    def _frame_done(self, fut):
        try:
            res = fut.result()
            ok = bool(res.results) and res.results[0].successful
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Setting {MAVROS_OBSTACLE_NODE} mav_frame failed: {exc}")
            return
        if ok:
            self.frame_set = True
            self.get_logger().info(f"{MAVROS_OBSTACLE_NODE} mav_frame = {MAVROS_FRAME}")
        else:
            reason = res.results[0].reason if res.results else "no result"
            self.get_logger().warn(f"mav_frame rejected: {reason} (will retry)")

    def pose_cb(self, msg):
        o, p = msg.pose.orientation, msg.pose.position
        self._pose = (time.time(), p.x, p.y, p.z, o.x, o.y, o.z, o.w)

    def ground_model(self):
        """Returns (height of lidar, rotation rows, tilt deg) or None."""
        pose = self._pose
        if pose is None or time.time() - pose[0] > POSE_STALE_S:
            return None
        _, _px, _py, z, x, y, zq, w = pose
        # third row of the body->world rotation matrix (only world z needed)
        r20 = 2 * (x * zq - w * y)
        r21 = 2 * (y * zq + w * x)
        r22 = 1 - 2 * (x * x + y * y)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, r22))))
        return z + LIDAR_HEIGHT_M, r20, r21, tilt

    # ---- gz input ----------------------------------------------------
    def gz_scan_cb(self, msg: GzLaserScan):
        with self._lock:
            self._pending = (list(msg.ranges), msg.angle_min, msg.angle_step,
                             msg.range_min, msg.range_max)
        self._new.set()

    def worker_loop(self):
        while rclpy.ok():
            if not self._new.wait(timeout=1.0):
                continue
            self._new.clear()
            with self._lock:
                item, self._pending = self._pending, None
            if item is None:
                continue
            try:
                self.process(*item)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"scan processing failed: {exc}")

    def log_rate(self):
        dt = time.time() - self._t0
        self.get_logger().info(
            f"scans: {self._count / dt:.1f} Hz, mavros frame set: {self.frame_set}")
        self._count, self._t0 = 0, time.time()

    # ---- processing --------------------------------------------------
    def process(self, ranges, angle_min, angle_step, range_min, range_max):
        n = len(ranges)
        if n == 0 or angle_step == 0:
            return
        self._count += 1
        stamp = self.get_clock().now().to_msg()
        rmin = max(range_min, SELF_FILTER_M)

        if not self._first_logged:
            raw = [r for r in ranges if math.isfinite(r) and r > 0]
            self.get_logger().info(
                f"first scan: {n} rays, nearest raw return "
                f"{min(raw) if raw else float('nan'):.2f} m (self filter {SELF_FILTER_M} m)")
            self._first_logged = True

        clean = [r if (math.isfinite(r) and rmin <= r <= range_max) else math.inf for r in ranges]

        # drop rays that are just the tilted scan plane hitting the ground
        gm = self.ground_model()
        self._ground_dropped, self._tilt = 0, (None if gm is None else gm[3])
        if gm is not None and gm[0] > 0.15:
            h, r20, r21, _ = gm
            for k, r in enumerate(clean):
                if r == math.inf:
                    continue
                a = angle_min + k * angle_step
                dz = r20 * math.cos(a) + r21 * math.sin(a)  # world-z of the ray
                if dz < -1e-3 and r >= GROUND_FRAC * (h / -dz):
                    clean[k] = math.inf
                    self._ground_dropped += 1

        # TF odom -> base_footprint, same stamp as the scan (for slam_toolbox)
        pose = self._pose
        if pose is not None and time.time() - pose[0] <= POSE_STALE_S:
            _, px, py, _pz, qx, qy, qz, qw = pose
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_footprint"
            tf.transform.translation.x = float(px)
            tf.transform.translation.y = float(py)
            tf.transform.rotation.z = math.sin(yaw / 2)
            tf.transform.rotation.w = math.cos(yaw / 2)
            self.tf_pub.sendTransform(tf)

        # 1. full scan, ROS convention
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = FRAME_ID
        scan.angle_min = float(angle_min)
        scan.angle_increment = float(angle_step)
        scan.angle_max = float(angle_min + angle_step * (n - 1))
        scan.scan_time = 0.1
        scan.time_increment = 0.1 / n
        scan.range_min = float(rmin)
        scan.range_max = float(range_max)
        scan.ranges = [float(r) for r in clean]
        self.scan_pub.publish(scan)

        # 2. 72 clockwise sectors for ArduPilot. Sector i is centred on
        #    bearing i*5 deg clockwise from the nose.
        sectors = [math.inf] * FCU_SECTORS
        deg_bins = [math.inf] * 360
        for k, r in enumerate(clean):
            if r == math.inf:
                continue
            b = cw_bearing_deg(angle_min + k * angle_step)
            s = int((b + FCU_STEP_DEG / 2) // FCU_STEP_DEG) % FCU_SECTORS
            if r < sectors[s]:
                sectors[s] = r
            d = int(round(b)) % 360
            if r < deg_bins[d]:
                deg_bins[d] = r
        no_obstacle = range_max + 0.01   # MAVLink: max_distance + 1 cm = clear
        fcu = LaserScan()
        fcu.header.stamp = stamp
        fcu.header.frame_id = "base_link_frd"
        fcu.angle_min = 0.0
        fcu.angle_increment = math.radians(FCU_STEP_DEG)
        fcu.angle_max = math.radians(FCU_STEP_DEG * (FCU_SECTORS - 1))
        fcu.range_min = float(rmin)
        fcu.range_max = float(range_max)
        fcu.ranges = [float(r if r != math.inf else no_obstacle) for r in sectors]
        self.fcu_pub.publish(fcu)

        # 3. dashboard summary
        now = time.time()
        if now - self._last_summary >= SUMMARY_PERIOD_S:
            self._last_summary = now
            self.write_summary(now, deg_bins, range_max)

    def write_summary(self, now, deg_bins, range_max):
        nearest_d, nearest_b = None, None
        for b, r in enumerate(deg_bins):
            if r != math.inf and (nearest_d is None or r < nearest_d):
                nearest_d, nearest_b = r, b
        sector_min = {}
        for i, name in enumerate(SECTOR_NAMES):
            centre = i * 45
            vals = [deg_bins[(centre + o) % 360] for o in range(-22, 23)]
            m = min(vals)
            sector_min[name] = None if m == math.inf else round(m, 2)
        data = {
            "t": now,
            "frame": "cw_from_nose_deg",
            "range_max": range_max,
            "self_filter_m": SELF_FILTER_M,
            "ranges": [None if r == math.inf else round(r, 2) for r in deg_bins],
            "nearest": None if nearest_d is None else {
                "dist": round(nearest_d, 2), "bearing": nearest_b},
            "sectors": sector_min,
            "fcu_frame_set": self.frame_set,
            "tilt_deg": None if self._tilt is None else round(self._tilt, 1),
            "ground_dropped": self._ground_dropped,
        }
        tmp = SUMMARY_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, separators=(",", ":"))
        os.replace(tmp, SUMMARY_PATH)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()
    gz_node = GzNode()
    if not gz_node.subscribe(GzLaserScan, GZ_TOPIC, node.gz_scan_cb):
        node.get_logger().error(f'Failed to subscribe to gz topic "{GZ_TOPIC}"')
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
