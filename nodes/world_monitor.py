#!/usr/bin/env python3
"""
World monitor - the live half of the Sim Bay WORLD panel. Nov 9, 2026.

Writes ~/shadow_sim_runtime/telemetry/world_live.json about once a second:
  - real_time_factor, sim_time, paused   (Gazebo /world/<w>/stats)
  - wind                                  (last command seen on
                                           /world/<w>/wind, i.e. whatever a
                                           future wind tool / training env
                                           sets; None = never changed)

The static half (gravity, physics step, limits, flight altitudes, speed
caps, avoidance state) is read straight from the world SDF and the config
files by simulation_control.py's /simulation/world, so the panel still
shows something useful with the sim stopped.

Talks to gz-transport directly like the camera and LiDAR nodes (no ROS
needed). Launched with plain python3 by run_full_sim.sh.
"""

import json
import os
import sys
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from gz.transport13 import Node as GzNode  # noqa: E402
from gz.msgs10.world_stats_pb2 import WorldStatistics  # noqa: E402
from gz.msgs10.wind_pb2 import Wind  # noqa: E402

WORLD = os.environ.get("SHADOW_WORLD", sys.argv[1] if len(sys.argv) > 1 else "shadow_runway")
OUT = os.path.expanduser("~/shadow_sim_runtime/telemetry/world_live.json")

state = {"world": WORLD, "real_time_factor": None, "sim_time": None,
         "paused": None, "wind": None}


def stats_cb(msg: WorldStatistics):
    state["real_time_factor"] = round(msg.real_time_factor, 3)
    state["sim_time"] = round(msg.sim_time.sec + msg.sim_time.nsec / 1e9, 1)
    state["paused"] = bool(msg.paused)


def wind_cb(msg: Wind):
    v = msg.linear_velocity
    state["wind"] = {"x": round(v.x, 2), "y": round(v.y, 2), "z": round(v.z, 2),
                     "enabled": bool(msg.enable_wind)}


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    node = GzNode()
    node.subscribe(WorldStatistics, f"/world/{WORLD}/stats", stats_cb)
    node.subscribe(Wind, f"/world/{WORLD}/wind", wind_cb)
    print(f"world_monitor: /world/{WORLD}/stats + /wind -> {OUT}", flush=True)
    while True:
        data = dict(state, t=time.time())
        tmp = OUT + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, OUT)
        time.sleep(1.0)


if __name__ == "__main__":
    main()
