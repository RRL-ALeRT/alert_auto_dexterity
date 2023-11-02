#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tf2_ros
from octomap_msgs.srv import BoundingBoxQuery
from alert_auto_dexterity.action import ManipulatorAction

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading


class OctomapClearer(Node):
    def __init__(self):
        super().__init__("octomap_clearer")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.clear_octomap_service = self.create_client(
            BoundingBoxQuery, "/kinova/octomap_server/clear_bbox"
        )

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            ManipulatorAction,
            "clear_octomap_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""

        self.location = goal_request.location
        self.goal_done = False

        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        while rclpy.ok() and not self.goal_done:
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return ManipulatorAction.Result()

            self.clear_octomap_near_object()

            self.create_rate(2).sleep()

        goal_handle.succeed()

        # Populate result message
        result = ManipulatorAction.Result()

        return result

    def clear_octomap_near_object(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", self.location, rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            size = 0.8
            # if self.location == "rs_front_color_optical_frame":
                # size = 0.6

            min_point = Point()
            min_point.x = x - size / 2
            min_point.y = y - size / 2
            min_point.z = z - size / 2
            max_point = Point()
            max_point.x = x + size / 2
            max_point.y = y + size / 2
            max_point.z = z + size / 2

            req = BoundingBoxQuery.Request()
            req.min = min_point
            req.max = max_point

            while not self.clear_octomap_service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    "Clear Box service not available, waiting again..."
                )

            future = self.clear_octomap_service.call_async(req)
            while rclpy.ok():
                if future.done():
                    self.goal_done = True
                    break

        except Exception as e:
            self.get_logger().error(f"{e}")


def main(args=None):
    rclpy.init(args=args)
    clearer = OctomapClearer()
    executor = MultiThreadedExecutor()
    rclpy.spin(clearer, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
