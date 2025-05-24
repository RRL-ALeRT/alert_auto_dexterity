import time
import rclpy
from rclpy.node import Node
from yasmin import Blackboard, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from sensor_msgs.msg import JointState
from tf2_ros import TransformException, Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class MoveToPoseState(State, Node):
    def __init__(self):
        Node.__init__(self, "move_to_pose_state")
        State.__init__(self, outcomes=[SUCCEED, CANCEL, ABORT])  # <-- Add CANCEL here
        set_ros_loggers()
        self.get_logger().info("MoveToPoseState initialized.")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state = None
        self.tf_base_link_pipe = None

        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/kinova_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.arm_client.wait_for_server()
        self.arm_joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ]
        self.sequence = [
            "linear_front", "front_right", "angled_right", "front_left", "angled_left"
        ]

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def execute(self, blackboard: Blackboard) -> str:
        # Wait for initial joint states and at least one transform
        start_time = time.time()
        while self.joint_state is None or self.tf_base_link_pipe is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    "base_link", "linear_front", rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                self.tf_base_link_pipe = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                ]
            except TransformException:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 10:
                self.get_logger().error("Timeout waiting for joint states or transform.")
                return ABORT

        # 1. Collect and store all transforms for the sequence
        transforms = {}
        for frame in self.sequence:
            tf_found = False
            start_time = time.time()
            while not tf_found and (time.time() - start_time < 5.0):
                try:
                    t = self.tf_buffer.lookup_transform(
                        "base_link",
                        frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                    transforms[frame] = [
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    ]
                    self.get_logger().info(
                        f"Stored base_link to {frame} transform: {transforms[frame]}"
                    )
                    tf_found = True
                except TransformException as ex:
                    rclpy.spin_once(self, timeout_sec=0.1)
            if not tf_found:
                self.get_logger().info(f"Could not transform base_link to {frame} after waiting.")
                transforms[frame] = None

        # 2. Compute IK and send trajectories using stored transforms
        for frame in self.sequence:
            tf_data = transforms.get(frame)
            if tf_data is None:
                self.get_logger().info(f"Skipping {frame}: no stored transform")
                continue

            self.tf_base_link_pipe = tf_data

            # Prepare IK request
            req = GetPositionIK.Request()
            req.ik_request.group_name = "manipulator"
            req.ik_request.robot_state.joint_state = self.joint_state
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            req.ik_request.pose_stamped.header.frame_id = "base_link"
            req.ik_request.pose_stamped.pose.position.x = tf_data[0] - 0.3
            req.ik_request.pose_stamped.pose.position.y = tf_data[1]
            req.ik_request.pose_stamped.pose.position.z = tf_data[2]
            req.ik_request.pose_stamped.pose.orientation.x = tf_data[3]
            req.ik_request.pose_stamped.pose.orientation.y = tf_data[4]
            req.ik_request.pose_stamped.pose.orientation.z = -tf_data[5]
            req.ik_request.pose_stamped.pose.orientation.w = -tf_data[6]
            req.ik_request.timeout.sec = 5

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if not response or not response.solution.joint_state.position:
                self.get_logger().info(f"No IK solution for {frame}")
                continue

            target_angles = list(response.solution.joint_state.position)[:6]

            # Send trajectory
            arm_goal_msg = FollowJointTrajectory.Goal()
            arm_goal_msg.trajectory.joint_names = self.arm_joint_names
            arm_point = JointTrajectoryPoint()
            arm_point.positions = target_angles
            arm_point.velocities = [0.0] * 6
            arm_point.time_from_start.sec = 3
            arm_goal_msg.trajectory.points.append(arm_point)
            send_goal_future = self.arm_client.send_goal_async(arm_goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().info("Trajectory goal rejected")
                continue
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info(f"Trajectory completed for {frame}")

            # Wait 3 seconds before next move
            time.sleep(3)

        return SUCCEED
