#!/usr/bin/env python3

import math
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from alert_auto_dexterity.action import TrajectoryToFrame

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from spot_msgs.action import Trajectory

from rclpy.duration import Duration


class TravelToFrame(Node):
    def __init__(self):
        super().__init__("spot_trajectory")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.timer = self.create_timer(0.1, self.on_timer)
        self.target_frame = ""
        self.rel_frame = "body"
        self.tf_available = False
        self.t = TransformStamped()

        self._action_server = ActionServer(
            self,
            TrajectoryToFrame,
            "trajectory_to_frame",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.trajectory_action_client = ActionClient(self, Trajectory, "trajectory")

        self.goal_done = False
        self.publish_feedback_once_flag = False

    def send_trajectory_goal(self, target_pose, duration, precise_positioning=False):
        # PoseStamped, Duration, bool
        goal_msg = Trajectory.Goal()
        goal_msg.target_pose = target_pose
        goal_msg.duration = duration
        goal_msg.precise_positioning = precise_positioning

        self.trajectory_action_client.wait_for_server()

        self.trajectory_future = self.trajectory_action_client.send_goal_async(
            goal_msg, feedback_callback=self.trajectory_feedback_callback
        )

        self.trajectory_future.add_done_callback(self.trajectory_goal_response_callback)

    def trajectory_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self.trajectory_get_result_future = goal_handle.get_result_async()
        self.trajectory_get_result_future.add_done_callback(
            self.trajectory_get_result_callback
        )

    def trajectory_get_result_callback(self, future):
        self.goal_done = True
        result = future.result().result
        self.get_logger().info(
            f"Trajectory Result: {result.success} - {result.message}"
        )

    def trajectory_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if self.publish_feedback_once_flag:
            self.publish_feedback_once_flag = False
            self.get_logger().info(f"Received feedback: {feedback.feedback}")

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.goal_done = False
        self.publish_feedback_once_flag = True
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.goal_done = True
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def calc_tf(self):
        # Store frame names in variables that will be used to
        # compute transformations
        try:
            self.t = self.tf_buffer.lookup_transform(
                self.rel_frame, self.target_frame, rclpy.time.Time()
            )
            self.tf_available = True
        except TransformException as ex:
            self.get_logger().info(f"{ex}")
            return

    def is_near_target(self):
        # Seems like spot's trajectory server checks for near, only in 2D plane.
        try:
            t = self.tf_buffer.lookup_transform(
                self.rel_frame, self.target_frame, rclpy.time.Time()
            )
            yaw = math.atan2(
                2.0
                * (
                    t.transform.rotation.y * t.transform.rotation.w
                    + t.transform.rotation.x * t.transform.rotation.z
                ),
                1.0
                - 2.0
                * (
                    t.transform.rotation.y * t.transform.rotation.y
                    + t.transform.rotation.x * t.transform.rotation.x
                ),
            )

            if (
                abs(t.transform.translation.x) < 0.05
                and abs(t.transform.translation.y) < 0.05
                and abs(yaw) < math.radians(4)
            ):
                return True
            return False
        except TransformException:
            return False

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info("Executing goal...")

        self.goal_done = False

        self.target_frame = goal_handle.request.frame_id

        while not self.tf_available:
            self.calc_tf()
        self.tf_available = False

        feedback_msg = TrajectoryToFrame.Feedback()

        pose = PoseStamped()

        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "body"
        pose.pose.position.x = self.t.transform.translation.x
        pose.pose.position.y = self.t.transform.translation.y

        pose.pose.orientation.z = self.t.transform.rotation.z
        pose.pose.orientation.w = self.t.transform.rotation.w

        duration = Duration(seconds=10, nanoseconds=0).to_msg()

        self.send_trajectory_goal(pose, duration)

        # self.get_logger().info(f"{self.trajectory_future.result()}")

        while not self.goal_done and rclpy.ok():
            if not goal_handle.is_active:
                self.trajectory_future.result().cancel_goal_async()
                self.get_logger().info("Goal aborted")
                return TrajectoryToFrame.Result()

            if goal_handle.is_cancel_requested:
                self.trajectory_future.result().cancel_goal_async()
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return TrajectoryToFrame.Result()

            goal_handle.publish_feedback(feedback_msg)
            if self.is_near_target():
                self.trajectory_future.result().cancel_goal_async()
                break

        self.create_rate(1).sleep()
        goal_handle.succeed()
        result = TrajectoryToFrame.Result()

        return result


def main():
    rclpy.init()

    action_server = TravelToFrame()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
