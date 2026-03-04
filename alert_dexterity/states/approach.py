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
from mbf_msgs.action import MoveBase

class MBFDexBase(ActionState, Node):
    def __init__(self, target_frame: str, offset_x: float, outcomes=None) -> None:
        Node.__init__(self, f"MBFdex_{target_frame}_node")
        ActionState.__init__(
            self,
            MoveBase,
            "/move_base_flex/move_base",
            self.create_goal_handler,
            outcomes or [SUCCEED, ABORT, CANCEL],  # Default outcomes if not provided
            self.response_handler,
            self.print_feedback,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f"MBFDex Node Initialized for frame: {target_frame}")

        base_frame = "odom"

        # parameter for goal offset
        self.declare_parameter("goal_offset_x", offset_x) 
        offset_x = self.get_parameter("goal_offset_x").value

        timeout = Duration(seconds=10.0)
        start_time = self.get_clock().now()
        while not self.tf_buffer.can_transform(
            base_frame, target_frame, Time(), Duration(seconds=0.05)
        ):
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error(f"Timed out waiting for transform from {target_frame} to {base_frame}")
                raise RuntimeError("Transform timeout")
            self.get_logger().info(f"Waiting for transform from {target_frame} to {base_frame}...")
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            t = self.tf_buffer.lookup_transform(
                base_frame, target_frame, Time(), Duration(seconds=0.05)
            )
            self.goal_msg = MoveBase.Goal()
            self.goal_msg.target_pose.header.frame_id = "map"
            self.goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_msg.target_pose.pose.position.x = t.transform.translation.x + offset_x
            self.goal_msg.target_pose.pose.position.y = t.transform.translation.y
            self.goal_msg.target_pose.pose.position.z = 0.0
            self.goal_msg.target_pose.pose.orientation.w = 1.0

        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            raise

    def create_goal_handler(self, blackboard: Blackboard) -> MoveBase.Goal:
        goal = MoveBase.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = self.goal_msg.target_pose.pose.position.x
        goal.target_pose.pose.position.y = self.goal_msg.target_pose.pose.position.y
        goal.target_pose.pose.position.z = self.goal_msg.target_pose.pose.position.z
        goal.target_pose.pose.orientation.w = self.goal_msg.target_pose.pose.orientation.w
        return goal

    def response_handler(self, blackboard: Blackboard, response: MoveBase.Result) -> str:
        blackboard["nav_result"] = response.outcome
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: MoveBase.Feedback
    ) -> None:
        if hasattr(feedback, 'dist_to_goal'):
            yasmin.YASMIN_LOG_INFO(f"Distance remaining: {feedback.dist_to_goal:.2f} m")
        elif hasattr(feedback, 'current_pose'):
            yasmin.YASMIN_LOG_INFO(f"Current pose: {feedback.current_pose.pose.position}")
        else:
            yasmin.YASMIN_LOG_INFO(f"Feedback received: {feedback}")


class MBFDexLinearFront(MBFDexBase):
    def __init__(self):
        super().__init__("linear_board", -0.8, outcomes=[SUCCEED, ABORT, CANCEL])


class MBFDexOmniFront(MBFDexBase):
    def __init__(self):
        super().__init__("omni_board", -0.5, outcomes=[SUCCEED, ABORT, CANCEL])


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

class StandUp(ServiceState, Node):
    def __init__(self):
        Node.__init__(self, "stand_up_node")
        ServiceState.__init__(
            self,
            SpotMotion,
            "/Spot/stand_up", 
            self.create_request_handler,
            [SUCCEED, ABORT],
            self.response_handler,
        )
        self.get_logger().info("Waiting for /Spot/stand_up service...")
        self.service_client = self.create_client(SpotMotion, "/Spot/stand_up")
        while not self.service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service /Spot/stand_up not available, waiting...")

    def create_request_handler(self, blackboard: Blackboard) -> SpotMotion.Request:
        req = SpotMotion.Request()
        req.override = True
        return req

    def response_handler(self, blackboard: Blackboard, response) -> str:
        yasmin.YASMIN_LOG_INFO(f"SpotMotion response: {response}")
        if hasattr(response, "answer") and response.answer in ["standing", "standing up"]:
            yasmin.YASMIN_LOG_INFO("Stand up service called successfully.")
            return SUCCEED
        else:
            yasmin.YASMIN_LOG_ERROR("Stand up service failed.")
            return ABORT

def print_result(blackboard: Blackboard) -> str:
    try:
        yasmin.YASMIN_LOG_INFO(f"Navigation outcome: {blackboard['nav_result']}")
    except KeyError:
        yasmin.YASMIN_LOG_INFO("Navigation outcome: unknown")
    return SUCCEED

def wait_period(blackboard: Blackboard) -> str:
    wait_seconds = 5  
    yasmin.YASMIN_LOG_INFO(f"Waiting for {wait_seconds} seconds before sitting down...")
    time.sleep(wait_seconds)
    return SUCCEED
