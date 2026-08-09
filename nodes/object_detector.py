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
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, '/shadow/front_camera/image_raw', self._image_cb, 10)

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

        self.lower_orange = np.array([5, 100, 100])
        self.upper_orange = np.array([25, 255, 255])

        self._frame_count = 0

        self.get_logger().info('Object detector started, watching '
                                '/shadow/front_camera/image_raw for '
                                'the orange payload...')

    def _image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
        debug_frame = frame.copy()

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

                # The actual box, drawn for the debug view only.
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h),
                              (0, 255, 0), 2)
                cv2.circle(debug_frame, (int(cx), int(cy)), 4,
                          (0, 0, 255), -1)

        self.detection_pub.publish(point)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_debug_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
