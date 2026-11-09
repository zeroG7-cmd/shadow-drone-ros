#!/usr/bin/env python3
"""
Map exporter - turns slam_toolbox's live /map into files the Zero Command
dashboard can show (Missions room, LIVE MAP panel). Nov 9, 2026.

  ~/shadow_sim_runtime/telemetry/map.png   the occupancy grid as an image
                                           (dark = unknown, dim green = free,
                                           bright green = wall/obstacle).
                                           North (+y) is up.
  ~/shadow_sim_runtime/telemetry/map.json  size, resolution, origin, how much
                                           is mapped, Shadow's position on
                                           the map (via TF map -> base_footprint)
                                           and a short trail.

Passive: if slam_toolbox isn't running there is simply no /map and nothing
is written. Launched with plain python3 by run_full_sim.sh.
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

OUT_DIR = os.path.expanduser("~/shadow_sim_runtime/telemetry")
PNG_PATH = os.path.join(OUT_DIR, "map.png")
JSON_PATH = os.path.join(OUT_DIR, "map.json")
OCCUPIED = 65            # slam_toolbox publishes 0 free, 100 occupied, -1 unknown
TRAIL_MAX = 400
TRAIL_STEP_M = 0.2

# BGR for cv2
C_UNKNOWN = (18, 23, 11)
C_FREE = (46, 58, 31)
C_OCC = (180, 255, 112)


class MapExporter(Node):
    def __init__(self):
        super().__init__("map_exporter")
        os.makedirs(OUT_DIR, exist_ok=True)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, qos)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.info = None
        self.stats = {}
        self.map_t = None
        self.trail = []
        self.create_timer(0.5, self.write_json)
        self.get_logger().info(f"Map exporter up: /map -> {PNG_PATH}, {JSON_PATH}")

    def map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        img = np.empty((h, w, 3), dtype=np.uint8)
        img[:] = C_UNKNOWN
        img[(grid >= 0) & (grid < OCCUPIED)] = C_FREE
        img[grid >= OCCUPIED] = C_OCC
        img = np.flipud(img)  # row 0 of the grid is the south edge; images start at the top
        tmp = PNG_PATH + ".tmp.png"
        cv2.imwrite(tmp, img)
        os.replace(tmp, PNG_PATH)
        res = msg.info.resolution
        self.info = {
            "width": w, "height": h, "resolution": res,
            "origin_x": msg.info.origin.position.x, "origin_y": msg.info.origin.position.y,
        }
        cell = res * res
        self.stats = {
            "known_m2": round(float(np.count_nonzero(grid >= 0)) * cell, 1),
            "occupied_m2": round(float(np.count_nonzero(grid >= OCCUPIED)) * cell, 2),
        }
        self.map_t = time.time()

    def drone_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except Exception:  # noqa: BLE001 - no TF yet is normal before takeoff/SLAM
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return {"x": round(t.x, 2), "y": round(t.y, 2), "yaw": round(yaw, 3)}

    def write_json(self):
        if self.info is None:
            return
        pose = self.drone_pose()
        if pose is not None:
            if not self.trail or math.hypot(pose["x"] - self.trail[-1][0],
                                            pose["y"] - self.trail[-1][1]) >= TRAIL_STEP_M:
                self.trail.append((pose["x"], pose["y"]))
                self.trail = self.trail[-TRAIL_MAX:]
        data = {"t": time.time(), "map_t": self.map_t, **self.info, **self.stats,
                "drone": pose, "trail": self.trail}
        tmp = JSON_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, separators=(",", ":"))
        os.replace(tmp, JSON_PATH)


def main(args=None):
    rclpy.init(args=args)
    node = MapExporter()
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
