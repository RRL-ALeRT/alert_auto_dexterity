#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from copy import deepcopy
import time

import yasmin
from yasmin import Blackboard, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL


class MoveToPoseState(State, Node):

    def __init__(self, sequence=None):
        Node.__init__(self, "move_to_pose_state")
        State.__init__(self, outcomes=[SUCCEED, CANCEL, ABORT])
        set_ros_loggers()
        self.get_logger().info("MoveToPoseState initialized.")

        self.declare_parameter("timeout_duration", 10.0)
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.timeout_duration = self.get_parameter("timeout_duration").value
        self.expected_joint_names = self.get_parameter("joint_names").value

        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/kinova_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.filtered_joint_state = None
        self.current_pose = None
        self.is_moving = False
        self.goal_reached = True
        self.last_command = None
        self.goal_handle = None

        # home pose
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = "base_link"
        self.home_pose.pose.position.x = 0.67
        self.home_pose.pose.position.y = 0.0
        self.home_pose.pose.position.z = 0.42
        self.home_pose.pose.orientation.x = 0.51
        self.home_pose.pose.orientation.y = -0.51
        self.home_pose.pose.orientation.z = -0.5
        self.home_pose.pose.orientation.w = 0.5

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Sequence of TF frames
        self.sequence = sequence or [
            "linear_front",
            "front_right",
            "angled_right",
            "front_left",
            "angled_left",
        ]

    def _wait_for_services_and_state(self):
        """Wait for services and initial state with timeout."""

        def wait_with_timeout(condition, log_msg, timeout_sec=10):
            start_time = self.get_clock().now()
            while not condition():
                self.get_logger().info(f"Waiting for {log_msg}...")
                rclpy.spin_once(self, timeout_sec=0.1)
                if (self.get_clock().now() - start_time).nanoseconds > timeout_sec * 1e9:
                    self.get_logger().warn(f"Timeout waiting for {log_msg}")
                    return False
            return True

        wait_with_timeout(
            lambda: self.cli.wait_for_service(timeout_sec=1.0), "IK service"
        )
        wait_with_timeout(
            lambda: self.arm_client.wait_for_server(timeout_sec=1.0),
            "FollowJointTrajectory action",
        )
        wait_with_timeout(
            lambda: self.filtered_joint_state is not None, "filtered joint state"
        )

        # Initial pose check
        if not wait_with_timeout(
            lambda: self.current_pose is not None, "initial pose", timeout_sec=2.0
        ):
            self.get_logger().warn(
                "Initial pose not available via TF, assuming default home pose."
            )
            self.current_pose = deepcopy(self.home_pose)

    def lookup_current_pose(self):
        try:
            if self.tf_buffer.can_transform(
                "base_link", "bracelet_link", rclpy.time.Time()
            ):
                trans = self.tf_buffer.lookup_transform(
                    "base_link", "bracelet_link", rclpy.time.Time()
                )
                self.current_pose = PoseStamped()
                self.current_pose.header.frame_id = "base_link"
                self.current_pose.pose.position.x = trans.transform.translation.x
                self.current_pose.pose.position.y = trans.transform.translation.y
                self.current_pose.pose.position.z = trans.transform.translation.z
                self.current_pose.pose.orientation.x = trans.transform.rotation.x
                self.current_pose.pose.orientation.y = trans.transform.rotation.y
                self.current_pose.pose.orientation.z = trans.transform.rotation.z
                self.current_pose.pose.orientation.w = trans.transform.rotation.w
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def joint_state_callback(self, msg: JointState):
        filtered_positions = [None] * len(self.expected_joint_names)
        for i, joint in enumerate(self.expected_joint_names):
            if joint in msg.name:
                filtered_positions[i] = msg.position[msg.name.index(joint)]

        if None in filtered_positions:
            return

        self.filtered_joint_state = JointState()
        self.filtered_joint_state.name = list(self.expected_joint_names)
        self.filtered_joint_state.position = filtered_positions

    def move_to_frame(self, target_frame: str):
        self.get_logger().info(f"Attempting to move to frame: {target_frame}")
        try:
            if not self.tf_buffer.can_transform(
                "base_link",
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ):
                self.get_logger().error(f"Could not find TF frame: {target_frame}")
                self.goal_reached = True
                return

            trans = self.tf_buffer.lookup_transform(
                "base_link", target_frame, rclpy.time.Time()
            )

            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = trans.transform.translation.x
            target_pose.pose.position.y = trans.transform.translation.y
            target_pose.pose.position.z = trans.transform.translation.z
            target_pose.pose.orientation.x = trans.transform.rotation.x
            target_pose.pose.orientation.y = trans.transform.rotation.y
            target_pose.pose.orientation.z = trans.transform.rotation.z
            target_pose.pose.orientation.w = trans.transform.rotation.w

            self.get_logger().info(
                f"Target pose for '{target_frame}' in base_link: "
                f"x={target_pose.pose.position.x:.3f}, "
                f"y={target_pose.pose.position.y:.3f}, "
                f"z={target_pose.pose.position.z:.3f}"
            )

            self.send_ik_request(target_pose, self.send_trajectory)
            self.last_command = f"move_to_{target_frame}"

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(
                f"Failed to lookup transform for {target_frame}: {e}"
            )
            self.goal_reached = True

    def home_cmd(self):
        if self.tf_buffer.can_transform("base_link", "home", rclpy.time.Time()):
            self.move_to_frame("home")
        else:
            self.get_logger().info("No 'home' TF found, using default home pose.")
            if not self.filtered_joint_state:
                self.get_logger().warn("Cannot move to home: joint state unavailable")
                self.goal_reached = True
                return
            self.send_ik_request(deepcopy(self.home_pose), self.send_trajectory)
            self.last_command = "home"

    def send_ik_request(self, target_pose: PoseStamped, callback):
        if not self.filtered_joint_state:
            self.get_logger().warn("No filtered joint state available")
            callback(None)
            return

        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = "manipulator"
        ik_req.ik_link_name = "bracelet_link"
        ik_req.avoid_collisions = True
        target_pose.header.stamp = self.get_clock().now().to_msg()
        ik_req.pose_stamped = target_pose
        ik_req.robot_state = RobotState()
        ik_req.robot_state.joint_state = self.filtered_joint_state
        ik_req.timeout.sec = 5
        req.ik_request = ik_req

        future = self.cli.call_async(req)
        future.add_done_callback(lambda fut: self._ik_callback(fut, callback))

    def _ik_callback(self, future, callback):
        try:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().debug("IK solution found")
                callback(response.solution.joint_state)
            else:
                self.get_logger().warn(
                    f"IK computation failed with error code: {response.error_code.val}"
                )
                callback(None)
        except Exception as e:
            self.get_logger().error(f"Failed to process IK response: {e}")
            callback(None)

    def send_trajectory(self, joint_state):
        if not joint_state:
            self.get_logger().warn("No valid joint state for trajectory")
            self.goal_reached = True
            return

        filtered_positions = [
            joint_state.position[joint_state.name.index(joint)]
            for joint in self.expected_joint_names
            if joint in joint_state.name
        ]

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.expected_joint_names)
        point = JointTrajectoryPoint()
        point.positions = filtered_positions
        point.velocities = [0.0] * len(filtered_positions)
        point.time_from_start.sec = 3
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.is_moving = True
        self.goal_reached = False

        self.get_logger().info("Sending trajectory goal...")
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Trajectory goal rejected")
            self.is_moving = False
            self.goal_reached = True
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:  # SUCCESS
            self.get_logger().info(
                f"Command {self.last_command} executed successfully"
            )
        else:
            self.get_logger().warn(
                f"Trajectory execution failed with code: {result.error_code}"
            )

        self.goal_reached = True
        self.is_moving = False
        self.goal_handle = None

    def execute(self, blackboard: Blackboard) -> str:
        self._wait_for_services_and_state()

        commands = [self.home_cmd]
        commands.extend([lambda f=frame: self.move_to_frame(f) for frame in self.sequence])
        # commands.append(self.home_cmd)
        # commands = [lambda f=frame: self.move_to_frame(f) for frame in self.sequence]
        # commands.append(self.home_cmd)

        self.get_logger().info(f"Starting sequence with frames: {self.sequence}")
        for i, cmd in enumerate(commands):
            self.get_logger().info(f"--- Step {i + 1}/{len(commands)} ---")

            self.goal_reached = False
            cmd()

            start_time = self.get_clock().now()
            while not self.goal_reached:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.lookup_current_pose()
                elapsed = (self.get_clock().now() - start_time).nanoseconds
                if elapsed > self.timeout_duration * 4 * 1e9:
                    self.get_logger().error(
                        f"Major timeout waiting for step {i + 1}"
                    )
                    if self.goal_handle:
                        self.goal_handle.cancel_goal_async()
                    self.goal_reached = True
                    break

            time.sleep(1.0)  # Pause between moves

        self.get_logger().info("Sequence completed.")
        return SUCCEED


def retract_manipulator(blackboard: Blackboard) -> str:
    arm = MoveToPoseState()
    arm._wait_for_services_and_state()

    goal = FollowJointTrajectory.Goal()
    trajectory = JointTrajectory()
    trajectory.joint_names = list(arm.expected_joint_names)
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.9, -2.54, 0.0, -1.0, 1.57]
    point.velocities = [0.0] * 6
    point.time_from_start.sec = 3
    trajectory.points.append(point)
    goal.trajectory = trajectory

    arm.goal_reached = False
    future = arm.arm_client.send_goal_async(goal)

    start_time = time.time()
    while not arm.goal_reached and (time.time() - start_time < 15.0):
        rclpy.spin_once(arm, timeout_sec=0.1)
        if future.done() and not hasattr(future, "_handled"):
            future._handled = True
            gh = future.result()
            if gh and gh.accepted:
                result_future = gh.get_result_async()
                result_future.add_done_callback(
                    lambda f: setattr(arm, "goal_reached", True)
                )
            else:
                arm.goal_reached = True

    yasmin.YASMIN_LOG_INFO("Manipulator retracted.")
    return SUCCEED
