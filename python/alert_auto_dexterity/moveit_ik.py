import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Twist

import numpy as np
import time

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

IS_SITTING = True


def quaternion_from_euler(roll, pitch, yaw):
    ai = roll / 2.0
    aj = pitch / 2.0
    ak = yaw / 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = Quaternion()
    q.x = cj * sc - sj * cs
    q.y = cj * ss + sj * cc
    q.z = cj * cs - sj * sc
    q.w = cj * cc + sj * ss

    return q


class MoveitIKClientAsync(Node):
    def __init__(self):
        super().__init__("moveit_ik")

        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, "/compute_ik")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            rclpy.spin_once(self)

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def send_request(self, x, y, z, qx, qy, qz, qw):
        self.joint_state = None

        while self.joint_state is None:
            rclpy.spin_once(self)

        req = GetPositionIK.Request()
        
        req.ik_request.group_name = "manipulator"
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True

        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.header.frame_id = "base_link"

        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z

        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw

        req.ik_request.timeout.sec = 10

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

        target_angles = list(self.future.result().solution.joint_state.position)[:6]

        if len(target_angles) > 0:
            return target_angles

        self.get_logger().warn("No ik soln found")
        return None


class SpotPitch(Node):
    def __init__(self):
        super().__init__("spot_pitch_control")

        self.pose_pub = self.create_publisher(Pose, "/body_pose", 1)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)

    def change_pitch(self, pitch_angle):
        if IS_SITTING:
            return
        roll = 0
        pitch = -pitch_angle
        yaw = 0

        pose = Pose()
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = quat

        for i in range(100):
            self.pose_pub.publish(pose)
            self.twist_pub.publish(Twist())
            time.sleep(0.02)


class MoveitIKPitch(Node):
    def __init__(self):
        super().__init__("moveit_ik")

        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, "/compute_ik")

        self.spot_pitch = SpotPitch()
        self.spot_pitch.change_pitch(0.0)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            rclpy.spin_once(self)
        
        self.current_pitch = 0.0

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def send_request(self, x, y, z, qx, qy, qz, qw):
        self.joint_state = None

        while self.joint_state is None:
            rclpy.spin_once(self)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        request = GetPositionIK.Request()

        request.ik_request.group_name = "manipulator"
        request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.avoid_collisions = False

        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.header.frame_id = "body"

        request.ik_request.pose_stamped.pose = pose

        request.ik_request.timeout.nanosec = 500000000

        joint_angles = []
        pitch_angles = [0.0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3, 0.35, -0.35]
        for pitch_angle in pitch_angles:
            transform = TransformStamped()
            transform.transform.rotation = quaternion_from_euler(0, pitch_angle, 0)
            pose = do_transform_pose(pose, transform)

            request.ik_request.pose_stamped.pose = pose

            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            joint_angles = list(future.result().solution.joint_state.position)[:6]
            print(pitch_angle, joint_angles)
            if len(joint_angles) > 0:
                break

        if pitch_angle > 0:
            pitch_angle += 0.05
        if pitch_angle < 0:
            pitch_angle -= 0.05
        
        if len(joint_angles) > 0:
            self.current_pitch += pitch_angle
            self.spot_pitch.change_pitch(self.current_pitch)
