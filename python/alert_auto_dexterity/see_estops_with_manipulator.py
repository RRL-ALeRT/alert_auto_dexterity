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

from alert_auto_dexterity.action import ManipulatorManipulation

from moveit_ik import MoveitIKClientAsync as IK
from moveit_action_client import MoveGroupActionClient as Moveit


class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __mul__(self, other):
        result = Quaternion(0, 0, 0, 1)
        result.x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        result.y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        result.z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        result.w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        return result


def moveit_motion(x,y,z,qx,qy,qz,qw):
    ik = IK()
    moveit = Moveit()
    target_angles = ik.send_request(x,y,z,qx,qy,qz,qw)
    if target_angles != None:
        ik.destroy_node()
        moveit.send_goal(target_angles)
        while not moveit.goal_done:
            rclpy.spin_once(moveit)
        return True, target_angles
    return False, target_angles


def moveit_set_joint_angles(target_angles):
    moveit = Moveit()
    moveit.send_goal(target_angles)
    while not moveit.goal_done:
        rclpy.spin_once(moveit)


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
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
    

class SeeObject(Node):
    def __init__(self):
        super().__init__('see_object')

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            ManipulatorManipulation,
            'see_with_manipulator',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joint_angles = None

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        # if goal_request.location != "previous_angles":
        #     self.joint_angles = None

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        self.position = None
        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return ManipulatorManipulation.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return ManipulatorManipulation.Result()
            
            # Set joint angles to past joint angles if location == "same" in goal_request
            if self.joint_angles is not None:
                moveit_set_joint_angles(self.joint_angles)
                goal_handle.succeed()

                # Populate result message
                result = ManipulatorManipulation.Result()

                return result
                
            if self.position is None:
                self.get_tf("base_link", "tool_frame")
                self.create_rate(1).sleep()
                continue

            x = self.position.translation.x
            y = self.position.translation.y
            z = self.position.translation.z + 0.1
            xa = self.position.rotation.x
            ya = self.position.rotation.y
            za = self.position.rotation.z
            wa = self.position.rotation.w

            feedback_msg = ManipulatorManipulation.Feedback()
            feedback_msg.end_effector_target.translation.x = x
            feedback_msg.end_effector_target.translation.y = y
            feedback_msg.end_effector_target.translation.z = z
            feedback_msg.end_effector_target.rotation.x = xa
            feedback_msg.end_effector_target.rotation.y = ya
            feedback_msg.end_effector_target.rotation.z = za
            feedback_msg.end_effector_target.rotation.w = wa
            goal_handle.publish_feedback(feedback_msg)

            manipulator_actuated, _ = moveit_motion(x, y, z, xa, ya, za, wa)

            if not manipulator_actuated:
                self.get_logger().info('Goal aborted')
                return ManipulatorManipulation.Result()
            
            break

        self.position = None
        self.orientation = None
        self.tf_buffer.clear()

        self.create_rate(1).sleep()

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return ManipulatorManipulation.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return ManipulatorManipulation.Result()

            while self.position is None or self.orientation is None:
                self.get_tf("base_link", "tool_frame")
                self.get_tf("camera_link", "estop_set")
                self.create_rate(1).sleep()
                continue

            x = self.position.translation.x
            y = self.position.translation.y
            z = self.position.translation.z
            xa = self.position.rotation.x
            ya = self.position.rotation.y
            za = self.position.rotation.z
            wa = self.position.rotation.w

            dx = self.orientation.translation.x
            dy = self.orientation.translation.y
            dz = self.orientation.translation.z

            # Calculate the quaternion to point in that direction
            roll = math.atan2(dy, dz)
            pitch = -math.atan2(dx, dz)
            yaw = 0  # Assuming no roll is needed

            quaternion = quaternion_from_euler(roll, pitch, yaw)

            # print(quaternion)
            q1 = Quaternion(xa, ya, za, wa)
            q2 = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            result = q1 * q2 # Adding them

            feedback_msg = ManipulatorManipulation.Feedback()
            feedback_msg.end_effector_target.translation.x = x
            feedback_msg.end_effector_target.translation.y = y
            feedback_msg.end_effector_target.translation.z = z
            feedback_msg.end_effector_target.rotation.x = result.x
            feedback_msg.end_effector_target.rotation.y = result.y
            feedback_msg.end_effector_target.rotation.z = result.z
            feedback_msg.end_effector_target.rotation.w = result.w
            goal_handle.publish_feedback(feedback_msg)

            manipulator_actuated, joint_angles = moveit_motion(x, y, z, result.x, result.y, result.z, result.w)

            if not manipulator_actuated:
                self.get_logger().info('Goal aborted')
                return ManipulatorManipulation.Result()

            break

        self.joint_angles = joint_angles

        # Sleep time to detect objects_set again
        self.create_rate(1).sleep()

        goal_handle.succeed()

        # Populate result message
        result = ManipulatorManipulation.Result()

        return result


    def get_tf(self, header, child):
        try:
            transform = self.tf_buffer.lookup_transform(
                header,
                child,
                rclpy.time.Time()
            )
            if header == "base_link":
                self.position = transform.transform
            else:
                self.orientation = transform.transform

        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SeeObject()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
