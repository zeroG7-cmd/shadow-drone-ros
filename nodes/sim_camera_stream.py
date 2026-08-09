#!/usr/bin/env python3
"""
Shadow mission planner - Stage A.

Flies a real offboard mission through MAVROS: arm, takeoff, fly to the
pickup pad, fly to the delivery pad, return to the start point, land.

No attach/detach logic yet - that's Stage B, once this flight path is
proven working on its own. This stage exists to verify the actual
flight sequencing (arming, OFFBOARD mode, setpoint streaming) works
correctly before adding payload complexity on top of it.

Coordinates are in MAVROS's local ENU frame (East-North-Up, z positive
= up), relative to wherever Shadow was armed - not raw Gazebo world
coordinates. Since Shadow spawns at the world origin, these line up
with the pickup_pad / delivery_pad positions in living_room.world.

Real, load-bearing rule confirmed from PX4's own docs before writing
this: OFFBOARD mode will be REJECTED if requested before setpoints are
already streaming. This is why setpoints are published for ~2 seconds
before the mode switch is ever requested, not the other way around.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time


PICKUP = (-1.0, 0.5, 1.0)      # x, y, z (height) in metres
DELIVERY = (0.2, 0.5, 1.0)
HOME = (0.0, 0.0, 1.0)
HOVER_SECONDS = 4.0


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, qos)

        self.have_local_position = False
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._local_pos_cb, qos)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos)

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.get_logger().info('MAVROS services available.')

    def _state_cb(self, msg):
        self.current_state = msg

    def _local_pos_cb(self, msg):
        # First message means MAVROS is actually receiving a real position
        # estimate from PX4's EKF - not just that MAVLink is connected.
        self.have_local_position = True

    def _publish_setpoint(self, xyz):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    def _stream_setpoint_for(self, xyz, seconds):
        """Publish a setpoint at 20Hz for the given duration - this is
        both how PX4 tracks a target position AND, before arming, how
        it satisfies the 'setpoints already streaming' requirement for
        OFFBOARD mode."""
        rate_hz = 20
        cycles = int(seconds * rate_hz)
        for _ in range(cycles):
            self._publish_setpoint(xyz)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

    def _set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info('OFFBOARD mode set.')
            return True
        self.get_logger().error('Failed to set OFFBOARD mode.')
        return False

    def _arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Armed.')
            return True
        self.get_logger().error('Arming failed.')
        return False

    def run_mission(self):
        self.get_logger().info('Waiting for MAVROS connection to PX4...')
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Connected. Waiting for a valid position '
                                'estimate before arming (this is the real '
                                'check that was missing before - arming '
                                'without one is why it spun without '
                                'climbing last time)...')
        while rclpy.ok() and not self.have_local_position:
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Position estimate confirmed. Streaming '
                                'initial setpoints...')

        # Required: stream setpoints before requesting OFFBOARD, or PX4
        # rejects the mode switch outright.
        self._stream_setpoint_for(HOME, 2.0)

        if not self._set_offboard_mode():
            return
        if not self._arm():
            return

        self.get_logger().info('Taking off...')
        self._stream_setpoint_for(HOME, HOVER_SECONDS)

        self.get_logger().info('Flying to pickup pad...')
        self._stream_setpoint_for(PICKUP, HOVER_SECONDS)

        self.get_logger().info('At pickup pad. Hovering (Stage B will '
                                'attach the payload here).')
        self._stream_setpoint_for(PICKUP, HOVER_SECONDS)

        self.get_logger().info('Flying to delivery pad...')
        self._stream_setpoint_for(DELIVERY, HOVER_SECONDS)

        self.get_logger().info('At delivery pad. Hovering (Stage B will '
                                'release the payload here).')
        self._stream_setpoint_for(DELIVERY, HOVER_SECONDS)

        self.get_logger().info('Returning home...')
        self._stream_setpoint_for(HOME, HOVER_SECONDS)

        self.get_logger().info('Landing...')
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Mission complete.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
