#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from webots_spot_msgs.srv import SpotMotion
import yasmin
from yasmin import  Blackboard
from yasmin_ros import ActionState, ServiceState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from nav2_msgs.action import NavigateToPose


class Nav2Dex(ActionState, Node):
    def __init__(self) -> None:
        Node.__init__(self, "nav2dex_node")
        ActionState.__init__(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,
            None,
            self.response_handler,
            self.print_feedback,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Nav2Dex Node Initialized")

        # Declare parameter for goal offset
        self.declare_parameter("goal_offset_x", -0.8)
        offset_x = self.get_parameter("goal_offset_x").value

        # Wait for transform with timeout
        timeout = Duration(seconds=5.0)
        start_time = self.get_clock().now()
        while not self.tf_buffer.can_transform(
            "base_link", "linear_front", Time(), Duration(seconds=0.05)
        ):
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error("Timed out waiting for transform from linear_front to base_link")
                raise RuntimeError("Transform timeout")
            self.get_logger().info("Waiting for transform from linear_front to base_link...")
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", "linear_front", Time(), Duration(seconds=0.05)
            )
            self.goal_msg = NavigateToPose.Goal()
            self.goal_msg.pose = PoseStamped()
            self.goal_msg.pose.header.frame_id = "map"
            self.goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_msg.pose.pose.position.x = t.transform.translation.x + offset_x
            self.goal_msg.pose.pose.position.y = t.transform.translation.y
            self.goal_msg.pose.pose.position.z = 0.0
            self.goal_msg.pose.pose.orientation.w = 1.0

        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            raise

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_msg.pose.pose.position.x
        goal.pose.pose.position.y = self.goal_msg.pose.pose.position.y
        goal.pose.pose.position.z = self.goal_msg.pose.pose.position.z
        goal.pose.pose.orientation.w = self.goal_msg.pose.pose.orientation.w
        return goal

    def response_handler(self, blackboard: Blackboard, response: NavigateToPose.Result) -> str:
        blackboard["nav_result"] = response.result
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: NavigateToPose.Feedback
    ) -> None:
        yasmin.YASMIN_LOG_INFO(f"Distance remaining: {feedback.distance_remaining:.2f} m")

class SitDown(ServiceState, Node):
    def __init__(self):
        Node.__init__(self, "sit_down_node")
        ServiceState.__init__(
            self,
            SpotMotion,
            "/Spot/lie_down",  
            self.create_request_handler,
            [SUCCEED, ABORT], 
            self.response_handler,
        )
        self.get_logger().info("Waiting for /Spot/lie_down service...")
        self.service_client = self.create_client(SpotMotion, "/Spot/lie_down")
        while not self.service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service /Spot/lie_down not available, waiting...")

    def create_request_handler(self, blackboard: Blackboard) -> SpotMotion.Request:
        req = SpotMotion.Request()
        req.override = True
        return req

    def response_handler(self, blackboard: Blackboard, response) -> str:
        yasmin.YASMIN_LOG_INFO(f"SpotMotion response: {response}")
        if hasattr(response, "answer") and response.answer == "lying down":
            yasmin.YASMIN_LOG_INFO("Sit down service called successfully.")
            return SUCCEED
        else:
            yasmin.YASMIN_LOG_ERROR("Sit down service failed.")
            return ABORT

def print_result(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['nav_result']}")
    return SUCCEED

def wait_period(blackboard: Blackboard) -> str:
    wait_seconds = 5  
    yasmin.YASMIN_LOG_INFO(f"Waiting for {wait_seconds} seconds before sitting down...")
    time.sleep(wait_seconds)
    return SUCCEED
