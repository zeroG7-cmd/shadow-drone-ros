#!/usr/bin/env python3
"""
MapAreaSkill - fly a path while slam_toolbox builds a 2D map from the
LiDAR, then check the map against the real world and save it. Tier 3,
Mapping branch. Added Nov 9, 2026.

Built on FollowPathSkill: same paths (zigzag by default), same 2.5 m
altitude, same nose-first flying. Mapping itself is NOT done here -
slam_toolbox runs the whole time the sim is up (run_full_sim.sh), fed by
lidar_bridge.py's /shadow/scan + TF. This skill is the exam:

  1. requirements(): the normal follow_path checks, plus slam_toolbox is
     actually publishing /map (otherwise there is nothing to test).
  2. Fly the path.
  3. Wait for a map update newer than the end of the path.
  4. Check the map against shadow_runway.sdf's real LiDAR targets:
       - south wall (y = -8, x -5..5): at least WALL_MIN_CELLS wall cells
         within TOL_M of where it really is
       - 4 pillars (r 0.25 m): at least 3 of 4 found within TOL_M
     Map coordinates equal world coordinates because mapping starts at
     the spawn point (MAVROS local origin). A map that is shifted or
     rotated by more than TOL_M fails, which is the point: "a map" is
     not enough, it has to be a right map.
  5. Save it either way to ~/shadow_sim_runtime/maps/ as a standard
     PGM + YAML (nav2 map_server format), plus slam_toolbox's own pose
     graph (needed later by the localize skill) when that service works.

Fails with a specific reason (wall not found, 2 of 4 pillars, no map...)
through self.failed, which run_skill.py reports.
"""

import math
import os
import time

import numpy as np

from .follow_path import FollowPathSkill

TOL_M = 0.35
OCCUPIED = 65
WALL = {"y": -8.0, "x_min": -5.0, "x_max": 5.0}
WALL_MIN_CELLS = 40                 # >= 2 m of wall at 0.05 m cells
PILLARS = [(-10.0, 4.5), (10.0, 4.5), (-9.0, -6.0), (9.0, -6.0)]
PILLAR_R = 0.25
PILLAR_MIN_CELLS = 4
PILLARS_NEEDED = 3
MAP_WAIT_S = 15.0
MAPS_DIR = os.path.expanduser("~/shadow_sim_runtime/maps")


class MapAreaSkill(FollowPathSkill):
    name = 'map_area'
    timeout_sec = 180.0   # follow_path's 150 s + waiting for the final map

    def __init__(self, body, path='zigzag'):
        super().__init__(body, path=path)
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._map = None
        self._map_t = None
        self._sub = body.create_subscription(OccupancyGrid, '/map', self._map_cb, qos)
        self.failed = None
        self._path_done_t = None

    def _map_cb(self, msg):
        self._map = msg
        self._map_t = time.time()

    # ---- contract ----------------------------------------------------
    def requirements(self):
        if not super().requirements():
            return False
        import rclpy
        deadline = time.time() + 5.0
        while self._map is None and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self.body, timeout_sec=0.2)
        if self._map is None:
            self.body.get_logger().warn(
                'map_area: requirements failed - no /map from slam_toolbox. Install it '
                '(sudo apt install ros-humble-slam-toolbox) and restart the simulation; '
                'check logs/slam_toolbox.log.')
            return False
        return True

    def start(self):
        super().start()
        self.body.get_logger().info('map_area: mapping while flying; slam_toolbox is building /map.')

    def update(self):
        if self._path_done_t is None:
            if super().update():
                self._path_done_t = time.time()
                self.body.get_logger().info('map_area: path complete, waiting for the final map update.')
            return False
        # hold the last waypoint while waiting
        x, y, z = self.waypoints[-1]
        self.body.send_position_setpoint(x, y, z)
        if self._map_t is None or self._map_t < self._path_done_t + 2.5:
            if time.time() - self._path_done_t > MAP_WAIT_S:
                self.failed = f'no map update within {MAP_WAIT_S:.0f}s of finishing the path'
            return False
        ok, summary = self.evaluate(self._map)
        saved = self.save(self._map)
        self.body.get_logger().info(f'map_area: {summary}. Saved {saved}.')
        if not ok:
            self.failed = summary
            return False
        return True

    # ---- checking ----------------------------------------------------
    def evaluate(self, msg):
        info = msg.info
        res, w, h = info.resolution, info.width, info.height
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        rows, cols = np.nonzero(grid >= OCCUPIED)
        xs = info.origin.position.x + (cols + 0.5) * res
        ys = info.origin.position.y + (rows + 0.5) * res
        known_m2 = float(np.count_nonzero(grid >= 0)) * res * res

        wall = np.count_nonzero((np.abs(ys - WALL['y']) <= TOL_M + 0.1) &
                                (xs >= WALL['x_min'] - TOL_M) & (xs <= WALL['x_max'] + TOL_M))
        found = []
        for px, py in PILLARS:
            d = np.hypot(xs - px, ys - py)
            n = int(np.count_nonzero(np.abs(d - PILLAR_R) <= TOL_M))
            found.append(n >= PILLAR_MIN_CELLS)
        n_pillars = sum(found)
        ok = wall >= WALL_MIN_CELLS and n_pillars >= PILLARS_NEEDED
        summary = (f'wall {"found" if wall >= WALL_MIN_CELLS else "NOT found"} ({wall} cells), '
                   f'{n_pillars} of 4 pillars within {TOL_M} m, {known_m2:.0f} m2 mapped')
        return ok, summary

    # ---- saving --------------------------------------------------------
    def save(self, msg):
        os.makedirs(MAPS_DIR, exist_ok=True)
        stem = os.path.join(MAPS_DIR, time.strftime('map_area_%Y%m%d_%H%M%S'))
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        img = np.full(grid.shape, 205, dtype=np.uint8)          # unknown
        img[(grid >= 0) & (grid < OCCUPIED)] = 254              # free
        img[grid >= OCCUPIED] = 0                               # occupied
        img = np.flipud(img)
        with open(stem + '.pgm', 'wb') as f:
            f.write(f'P5\n{info.width} {info.height}\n255\n'.encode())
            f.write(img.tobytes())
        with open(stem + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(stem)}.pgm\n'
                    f'mode: trinary\nresolution: {info.resolution}\n'
                    f'origin: [{info.origin.position.x}, {info.origin.position.y}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        self._serialize(stem)
        return stem + '.pgm/.yaml'

    def _serialize(self, stem):
        """slam_toolbox's own pose graph, for localization later. Best effort."""
        try:
            import rclpy
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError:
            return
        cli = self.body.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not cli.wait_for_service(timeout_sec=1.0):
            return
        req = SerializePoseGraph.Request()
        req.filename = stem
        fut = cli.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(self.body, timeout_sec=0.1)
