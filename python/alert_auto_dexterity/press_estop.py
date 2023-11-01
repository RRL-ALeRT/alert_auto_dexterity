#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
import numpy as np
import math
import threading

from alert_auto_dexterity.action import ManipulatorAction

from moveit_ik import MoveitIKClientAsync as IK
from moveit_action_client import MoveGroupActionClient as Moveit
from gripper_action_client import GripperActionClient as Gripper

from utils import Quaternion


def moveit_motion(x, y, z, qx, qy, qz, qw):
    ik = IK()
    moveit = Moveit()
    target_angles = ik.send_request(x, y, z, qx, qy, qz, qw)
    if target_angles != None:
        ik.destroy_node()
        moveit.send_goal(target_angles)
        while not moveit.goal_done:
            rclpy.spin_once(moveit)
        return target_angles
    return target_angles


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class SeeObject(Node):
    def __init__(self):
        super().__init__("see_object")

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            ManipulatorAction,
            "single_object_task",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
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

        self.tf_buffer.clear()
        self.position1 = None
        self.position2 = None

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        gripper = Gripper()
        self.get_logger().info("Executing goal...")

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return ManipulatorAction.Result()

            if self.position1 is None or self.position2 is None:
                self.get_tf("base_link", "tool_estop_target_0_1")
                self.get_tf("base_link", "tool_estop_target_0_035")
                self.create_rate(2).sleep()
                continue

            x = self.position1.translation.x
            y = self.position1.translation.y
            z = self.position1.translation.z
            xa = self.position1.rotation.x
            ya = self.position1.rotation.y
            za = self.position1.rotation.z
            wa = self.position1.rotation.w

            feedback_msg = ManipulatorAction.Feedback()
            feedback_msg.end_effector_target.translation.x = x
            feedback_msg.end_effector_target.translation.y = y
            feedback_msg.end_effector_target.translation.z = z
            feedback_msg.end_effector_target.rotation.x = xa
            feedback_msg.end_effector_target.rotation.y = ya
            feedback_msg.end_effector_target.rotation.z = za
            feedback_msg.end_effector_target.rotation.w = wa
            goal_handle.publish_feedback(feedback_msg)

            joint_angles = moveit_motion(x, y, z, xa, ya, za, wa)
            if joint_angles is None:
                gripper.send_goal("open")
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            break

        gripper.send_goal("close")
        while (not gripper.goal_done) and rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return ManipulatorAction.Result()

            rclpy.spin_once(gripper)

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return ManipulatorAction.Result()

            x = self.position2.translation.x
            y = self.position2.translation.y
            z = self.position2.translation.z
            xa = self.position2.rotation.x
            ya = self.position2.rotation.y
            za = self.position2.rotation.z
            wa = self.position2.rotation.w

            feedback_msg = ManipulatorAction.Feedback()
            feedback_msg.end_effector_target.translation.x = x
            feedback_msg.end_effector_target.translation.y = y
            feedback_msg.end_effector_target.translation.z = z
            feedback_msg.end_effector_target.rotation.x = xa
            feedback_msg.end_effector_target.rotation.y = ya
            feedback_msg.end_effector_target.rotation.z = za
            feedback_msg.end_effector_target.rotation.w = wa
            goal_handle.publish_feedback(feedback_msg)

            joint_angles = moveit_motion(x, y, z, xa, ya, za, wa)
            if joint_angles is None:
                gripper.send_goal("open")
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            break

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return ManipulatorAction.Result()

            x = self.position1.translation.x
            y = self.position1.translation.y
            z = self.position1.translation.z
            xa = self.position1.rotation.x
            ya = self.position1.rotation.y
            za = self.position1.rotation.z
            wa = self.position1.rotation.w

            feedback_msg = ManipulatorAction.Feedback()
            feedback_msg.end_effector_target.translation.x = x
            feedback_msg.end_effector_target.translation.y = y
            feedback_msg.end_effector_target.translation.z = z
            feedback_msg.end_effector_target.rotation.x = xa
            feedback_msg.end_effector_target.rotation.y = ya
            feedback_msg.end_effector_target.rotation.z = za
            feedback_msg.end_effector_target.rotation.w = wa
            goal_handle.publish_feedback(feedback_msg)

            joint_angles = moveit_motion(x, y, z, xa, ya, za, wa)
            if joint_angles is None:
                gripper.send_goal("open")
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            break

        gripper.send_goal("open")
        while (not gripper.goal_done) and rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return ManipulatorAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return ManipulatorAction.Result()

            rclpy.spin_once(gripper)

        goal_handle.succeed()

        # Populate result message
        result = ManipulatorAction.Result()

        return result

    def get_tf(self, header, child):
        try:
            transform = self.tf_buffer.lookup_transform(
                header, child, rclpy.time.Time()
            )
            if child == "tool_estop_target_0_1":
                self.position1 = transform.transform
            else:
                self.position2 = transform.transform

        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {str(e)}")
            self.create_rate(2).sleep()


def main(args=None):
    rclpy.init(args=args)
    node = SeeObject()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
