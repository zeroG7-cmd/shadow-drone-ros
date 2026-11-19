#!/usr/bin/env python3
"""
Object detector - watches Shadow's front camera for the orange payload
cube and publishes where it is in the image.

Detection technique: HSV colour thresholding. Chosen deliberately over
a general object detector because the payload is a solid, distinct
colour against a room that isn't - the simplest tool that genuinely
fits this specific case, not a shortcut. Hue range verified against
the payload's real defined colour in living_room.world (0.9, 0.6, 0.1
RGB -> hue ~19 on OpenCV's 0-179 scale), not guessed - but Gazebo's
actual rendering/lighting may shift this slightly, so it's worth
tuning against the real live feed if detection looks unreliable.

Publishes geometry_msgs/Point on /shadow/target_position:
  x, y - normalised image position, -1 to 1, independent of resolution
         (0,0 = dead centre of the frame)
  z    - contour area in pixels; 0 means nothing detected, larger
         roughly means the object is closer/bigger in view

This is Stage 1 only - detection. Nothing here flies the drone. A
mission script would subscribe to this topic and decide what to do
with it, same relationship mission_planner.py already has with MAVROS.

Oct 24, 2026 rewrite - same root cause as sim_camera_stream.py's fix
the same day: this used to subscribe to a ROS2 topic,
/shadow/front_camera/image_raw, that nothing in the stack has ever
published to (Gazebo's camera sensor only publishes on its own
gz-transport topic of the same name, sans leading slash - never
bridged into ROS2 after the Oct 2-4 ArduPilot/Harmonic migration; see
areas/fpv-camera-telemetry-harmonic-fix.md for the full root-cause
writeup). The "SensorDataQoS" comment below about why this "sat
silently receiving nothing" was a real, correct observation about
*something* - it just wasn't the actual cause this time: the topic
had zero publishers of ANY QoS, so nothing could have connected
regardless.

INPUT side only is rewritten: subscribes directly to Gazebo's own
gz-transport topic via gz.transport13/gz.msgs10 (matching
sim_camera_stream.py's approach), decoding the raw RGB_INT8 frame
into a numpy array by hand - no cv_bridge needed for this direction
either. OUTPUT side (the three ROS2 publishers below:
/shadow/target_position, /shadow/detection_debug,
/shadow/detection_mask) is UNCHANGED in spirit - still real rclpy
publishers, since a future mission script genuinely needs to
subscribe to /shadow/target_position over ROS2 - but now builds
sensor_msgs/Image messages by hand too (see numpy_to_image_msg below)
instead of via cv_bridge.cv2_to_imgmsg, so this script no longer
depends on cv_bridge or its numpy-ABI fragility at all.
"""

"""
Nov 8, 2026 - added the colour-blind object tracker (object_tracker.py,
same folder) alongside the original orange detection. Two outputs now,
deliberately separate:

  /shadow/target_position  (geometry_msgs/Point) - UNCHANGED. Still the
      original orange-only, largest-blob logic. deliver_payload.py and
      PayloadSensor read this, and must keep ignoring the blue/green/
      purple distractors until a real classifier (YOLO) exists.
  /shadow/detection/track  (std_msgs/Float32MultiArray) - NEW. The
      tracker's output: [state, cx, cy, w, h, hits, misses], positions
      and sizes normalised 0..1 to the frame. state 0 = nothing,
      1 = tracking (yellow), 2 = locked (green). The FPV overlay draws
      THIS, so the box on screen is now the real tracked box.

Tracker mode comes from the SHADOW_TRACK_MODE env var: "any" (default,
any strongly coloured object) or "orange" (match what the mission
targets). /shadow/detection_mask and /shadow/detection_debug now show
the tracker's mask and overlay.
"""

import os
import sys
# Must be set before gz.msgs10 is ever imported - see the identical
# comment in sim_camera_stream.py for why (old-style generated
# protobuf code vs. a newer protobuf runtime on this machine).
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32MultiArray
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import object_tracker as ot  # noqa: E402  (same folder; launched with plain python3)

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

GZ_TOPIC = "shadow/front_camera/image_raw"  # gz-transport, no leading slash
GZ_FRAME_ID = "front_camera_link"  # matches model.sdf's <gz_frame_id>


def numpy_to_image_msg(array, encoding, header):
    """Hand-rolled equivalent of cv_bridge.cv2_to_imgmsg - avoids
    cv_bridge's numpy-ABI fragility entirely for this (small, simple)
    conversion. array is HxWx3 (encoding='bgr8') or HxW (encoding='mono8')."""
    msg = Image()
    msg.header = header
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = array.shape[2] if array.ndim == 3 else 1
    msg.step = array.shape[1] * channels
    msg.data = array.tobytes()
    return msg


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.detection_pub = self.create_publisher(
            Point, '/shadow/target_position', 10)

        # Separate debug topic with a visible box drawn on it, purely
        # for humans to check detection is working - nothing else
        # subscribes to this or needs it.
        self.debug_image_pub = self.create_publisher(
            Image, '/shadow/detection_debug', 10)

        # Raw black/white mask - shows exactly which pixels pass the
        # HSV threshold, independent of the box/contour logic. This is
        # the real diagnostic for "why isn't it detecting" - if the
        # object is visually there but the mask is black over it, the
        # threshold itself is the problem, not the detection logic.
        self.mask_debug_pub = self.create_publisher(
            Image, '/shadow/detection_mask', 10)

        self.track_pub = self.create_publisher(
            Float32MultiArray, '/shadow/detection/track', 10)

        self.lower_orange = np.array([5, 100, 100])
        self.upper_orange = np.array([25, 255, 255])

        self.track_mode = os.environ.get('SHADOW_TRACK_MODE', ot.MODE_ANY).strip().lower()
        if self.track_mode not in (ot.MODE_ANY, ot.MODE_ORANGE):
            self.track_mode = ot.MODE_ANY
        self.tracker = ot.Tracker()

        self._frame_lock = threading.Lock()
        self._pending = None       # newest unprocessed frame
        self._received = 0         # frames arrived from Gazebo
        self._dropped = 0          # frames skipped because a newer one arrived first
        self._proc_ms = 0.0        # time to process the last frame
        self._last_mask_pub = 0.0
        self._last_stats = (time.monotonic(), 0, 0)

        self._frame_count = 0

        self.get_logger().info('Object detector started, watching '
                                f'gz-transport topic "{GZ_TOPIC}". Mission output: '
                                f'orange only. Tracker mode: {self.track_mode}.')

    # Nov 8, 2026 (second pass) - LATEST-FRAME-ONLY processing.
    # Live test showed the FPV box sitting where the target appeared
    # from ground level while Shadow hovered at 2 m: the detector ran at
    # ~10.7 fps (30 frames per ~2.8 s in the log) against a faster
    # camera, so frames queued up and the box was drawn from frames
    # that were increasingly old. Now the gz callback only stores the
    # newest frame (cheap) and a worker thread always processes the
    # most recent one, dropping any it couldn't get to. The box can lag
    # by at most one processing step, never by a growing backlog.
    def gz_image_cb(self, gz_msg: GzImage):
        with self._frame_lock:
            if self._pending is not None:
                self._dropped += 1
            self._pending = (bytes(gz_msg.data), gz_msg.height, gz_msg.width, gz_msg.step)
            self._received += 1

    def worker_loop(self):
        while rclpy.ok():
            with self._frame_lock:
                item, self._pending = self._pending, None
            if item is None:
                time.sleep(0.003)
                continue
            t0 = time.monotonic()
            try:
                self.process_frame(*item)
            except Exception as exc:  # never let one bad frame kill the detector
                self.get_logger().warn(f'frame processing failed: {exc}')
            self._proc_ms = (time.monotonic() - t0) * 1000.0

    def process_frame(self, data, height_px, width_px, step):
        # Same decode as sim_camera_stream.py's image_callback - see
        # that file's comments for why msg.step (not width*3) is the
        # right row stride to use.
        raw = np.frombuffer(data, dtype=np.uint8).reshape(height_px, step)
        rgb = raw[:, : width_px * 3].reshape(height_px, width_px, 3)
        frame = np.ascontiguousarray(rgb[:, :, ::-1])  # Gazebo sends RGB; cv2 wants BGR

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = GZ_FRAME_ID

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # Log the actual HSV value at the centre of the frame every
        # ~30 frames (roughly once a second) - point the camera at the
        # stationary payload and read these real numbers to see
        # exactly where they fall relative to the threshold below.
        self._frame_count += 1
        if self._frame_count % 30 == 0:
            h, w = hsv.shape[:2]
            center_hsv = hsv[h // 2, w // 2]
            self.get_logger().info(
                f'Centre pixel HSV: {center_hsv.tolist()} | '
                f'threshold: {self.lower_orange.tolist()} to '
                f'{self.upper_orange.tolist()}')

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        point = Point()  # defaults to (0,0,0) - z=0 means "not detected"

        # --- Tracker (colour-blind by default) - drives the FPV box ---
        height, width = frame.shape[:2]
        track_mask = ot.make_mask(frame, self.track_mode)
        status = self.tracker.update(ot.find_candidates(track_mask), width, height)
        track_msg = Float32MultiArray()
        track_msg.data = status['flat']
        self.track_pub.publish(track_msg)
        if self._frame_count % 30 == 0:
            now = time.monotonic()
            t_prev, rx_prev, done_prev = self._last_stats
            dt = max(1e-6, now - t_prev)
            self._last_stats = (now, self._received, self._frame_count)
            self.get_logger().info(
                f'Tracker: state={status["state"]} hits={status["hits"]} '
                f'misses={status["misses"]} mode={self.track_mode} | '
                f'camera {(self._received - rx_prev) / dt:.1f} fps, '
                f'processed {(self._frame_count - done_prev) / dt:.1f} fps, '
                f'{self._proc_ms:.0f} ms/frame, dropped {self._dropped} total')
        debug_frame = ot.draw_overlay(frame, status) if self.debug_image_pub.get_subscription_count() > 0 else frame

        # --- Mission output (orange only, unchanged logic) ---
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 50:  # ignore tiny noise blobs
                moments = cv2.moments(largest)
                cx = moments['m10'] / moments['m00']
                cy = moments['m01'] / moments['m00']

                height, width = frame.shape[:2]
                point.x = (cx - width / 2) / (width / 2)
                point.y = (cy - height / 2) / (height / 2)
                point.z = area

        self.detection_pub.publish(point)
        # Building a ROS Image from Python is slow (it copies ~900 KB per
        # colour frame), which is a big part of why this ran at ~11 fps.
        # Debug image: only when something is actually subscribed. Mask:
        # throttled to ~3 Hz for the /mask view in sim_camera_stream.py.
        if self.debug_image_pub.get_subscription_count() > 0:
            self.debug_image_pub.publish(numpy_to_image_msg(debug_frame, 'bgr8', header))
        now = time.monotonic()
        if now - self._last_mask_pub >= 0.33 and self.mask_debug_pub.get_subscription_count() > 0:
            self._last_mask_pub = now
            self.mask_debug_pub.publish(numpy_to_image_msg(track_mask, 'mono8', header))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    gz_node = GzNode()
    subscribed = gz_node.subscribe(GzImage, GZ_TOPIC, node.gz_image_cb)
    if not subscribed:
        node.get_logger().error(
            f'Failed to subscribe to gz-transport topic "{GZ_TOPIC}" - '
            f'node.subscribe() returned false.')

    worker = threading.Thread(target=node.worker_loop, daemon=True)
    worker.start()

    try:
        # gz-transport delivers frames on its own background thread
        # (see gz_image_cb above); rclpy.spin() here just keeps the
        # ROS2 side (the three publishers, logging, process lifetime)
        # alive - no ROS2 subscription callbacks to dispatch anymore.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
