import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from yasmin_ros import ActionState
from nav2_msgs.action import NavigateToPose
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import math

class AlignToMapOrientation(ActionState, Node):
    def __init__(self):
        Node.__init__(self, "align_to_map_orientation_node")
        ActionState.__init__(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,
            None,
            self.response_handler,
            None,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("AlignToMapOrientation Node Initialized")


    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        timeout = Duration(seconds=10.0)
        start_time = self.get_clock().now()
        while not self.tf_buffer.can_transform(
            "odom", "base_link", Time(), Duration(seconds=0.05)
        ):
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error("Timed out waiting for transform from map to base_link")
                raise RuntimeError("Transform timeout")
            self.get_logger().info("Waiting for transform from map to base_link...")
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            t = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time(seconds=0), timeout=Duration(seconds=0.05)
            )
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = t.transform.translation.x
            goal.pose.pose.position.y = t.transform.translation.y
            goal.pose.pose.position.z = 0.0

            #face along map x-axis (yaw=0)
            goal.pose.pose.orientation.x = 0.0
            goal.pose.pose.orientation.y = 0.0
            goal.pose.pose.orientation.z = 0.0
            goal.pose.pose.orientation.w = 1.0
            return goal
        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            raise

    def response_handler(self, blackboard: Blackboard, response: NavigateToPose.Result) -> str:
        return SUCCEED
