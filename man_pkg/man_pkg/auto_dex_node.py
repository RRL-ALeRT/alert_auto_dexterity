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
from typing import Callable, List, Optional
import time

class LinearCommander(Node):
    def __init__(self):
        super().__init__('linear_commander')
        self._initialize_parameters()
        self._initialize_clients()
        self._initialize_state()
        self._initialize_subscribers_and_timers()
        self._wait_for_services_and_state()

    def _initialize_parameters(self):
        """Initialize node parameters."""
        self.declare_parameter('timeout_duration', 5.0)
        self.declare_parameter('joint_names', ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        
        self.timeout_duration = self.get_parameter('timeout_duration').value
        self.expected_joint_names = self.get_parameter('joint_names').value

    def _initialize_clients(self):
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/kinova_joint_trajectory_controller/follow_joint_trajectory')

    def _initialize_state(self):
        self.filtered_joint_state = None
        self.current_pose = None
        self.is_moving = False
        self.goal_reached = True
        self.last_command = None
        self.goal_handle = None

        # Fallback home pose if 'home' TF is missing
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = 'map'
        self.home_pose.pose.position.x = 0.668
        self.home_pose.pose.position.y = 0.001
        self.home_pose.pose.position.z = 0.4225
        self.home_pose.pose.orientation.x = 0.49
        self.home_pose.pose.orientation.y = -0.49
        self.home_pose.pose.orientation.z = -0.5
        self.home_pose.pose.orientation.w = 0.5

    def _initialize_subscribers_and_timers(self):
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Timer to keep current pose updated (useful for debugging)
        self.tf_timer = self.create_timer(0.1, self.lookup_current_pose)
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)

    def _wait_for_services_and_state(self):
        """Wait for services and initial state with timeout."""
        def wait_with_timeout(condition, log_msg, timeout_sec=10):
            start_time = self.get_clock().now()
            while not condition():
                self.get_logger().info(f"Waiting for {log_msg}...")
                rclpy.spin_once(self)
                if (self.get_clock().now() - start_time).nanoseconds > timeout_sec * 1e9:
                    self.get_logger().warn(f"Timeout waiting for {log_msg}")
                    return False
            return True

        wait_with_timeout(lambda: self.cli.wait_for_service(timeout_sec=1.0), "IK service")
        wait_with_timeout(lambda: self.action_client.wait_for_server(timeout_sec=1.0), "FollowJointTrajectory action")
        wait_with_timeout(lambda: self.filtered_joint_state is not None, "filtered joint state")
        
        # Initial pose check
        if not wait_with_timeout(lambda: self.current_pose is not None, "initial pose", timeout_sec=2.0):
            self.get_logger().warn("Initial pose not available via TF, assuming default home pose internally.")
            self.current_pose = deepcopy(self.home_pose)

    def lookup_current_pose(self):
        """Updates internal state with the current end effector pose via TF."""
        try:
            if self.tf_buffer.can_transform('map', 'end_effector_link', rclpy.time.Time()):
                trans = self.tf_buffer.lookup_transform('map', 'end_effector_link', rclpy.time.Time())
                self.current_pose = PoseStamped()
                self.current_pose.header.frame_id = 'map'
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
        """Filter joint state for expected joints."""
        filtered_positions = [None] * len(self.expected_joint_names)
        found_any = False
        for i, joint in enumerate(self.expected_joint_names):
            if joint in msg.name:
                filtered_positions[i] = msg.position[msg.name.index(joint)]
                found_any = True
        
        if None in filtered_positions:
            return

        self.filtered_joint_state = JointState()
        self.filtered_joint_state.name = self.expected_joint_names
        self.filtered_joint_state.position = filtered_positions

    def move_to_frame(self, target_frame: str):
        """
        Look up the transform for target_frame relative to map
        and move the end-effector directly to that pose.
        All offsets are baked into the TF tree.
        """
        self.get_logger().info(f"Attempting to move to frame: {target_frame}")
        
        # 1. Wait for the TF frame to become available (Startup Race Condition Fix)
        timeout_sec = 3.0
        start_time = self.get_clock().now()
        
        while not self.tf_buffer.can_transform('map', target_frame, rclpy.time.Time()):
            if (self.get_clock().now() - start_time).nanoseconds > timeout_sec * 1e9:
                self.get_logger().error(f"Timeout: Could not find valid TF path to '{target_frame}'")
                self.goal_reached = True
                return
            # Actively spin the node so the tf_buffer can receive incoming messages
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2. Proceed with the lookup now that we know the buffer has it
        try:
            trans = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())

            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            target_pose.pose.position.x = trans.transform.translation.x
            target_pose.pose.position.y = trans.transform.translation.y
            target_pose.pose.position.z = trans.transform.translation.z
            target_pose.pose.orientation.x = trans.transform.rotation.x
            target_pose.pose.orientation.y = trans.transform.rotation.y
            target_pose.pose.orientation.z = trans.transform.rotation.z
            target_pose.pose.orientation.w = trans.transform.rotation.w

            self.get_logger().info(
                f"Target pose for '{target_frame}' in map: "
                f"x={target_pose.pose.position.x:.3f}, "
                f"y={target_pose.pose.position.y:.3f}, "
                f"z={target_pose.pose.position.z:.3f}"
            )

            self.send_ik_request(target_pose, self.send_trajectory)
            self.last_command = f"move_to_{target_frame}"
            self.reset_timeout_timer()

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform for {target_frame}: {e}")
            self.goal_reached = True

    def home_cmd(self):
        """Move to home position."""
        # Ideally, you have a 'home' TF frame. If not, this falls back to hardcoded.
        if self.tf_buffer.can_transform('map', 'home', rclpy.time.Time()):
             self.move_to_frame('home')
        else:
            self.get_logger().info("No 'home' TF found, using default home pose.")
            if not self.filtered_joint_state:
                self.get_logger().warn("Cannot move to home: joint state unavailable")
                self.goal_reached = True
                return
            self.send_ik_request(deepcopy(self.home_pose), self.send_trajectory)
            self.last_command = 'home'
            self.reset_timeout_timer()

    def send_ik_request(self, target_pose: PoseStamped, callback: Callable):
        if not self.filtered_joint_state:
            self.get_logger().warn("No filtered joint state available")
            callback(None)
            return
        
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = "manipulator"
        ik_req.ik_link_name = "end_effector_link"
        ik_req.avoid_collisions = True
        target_pose.header.stamp = self.get_clock().now().to_msg()
        ik_req.pose_stamped = target_pose
        ik_req.robot_state = RobotState()
        ik_req.robot_state.joint_state = self.filtered_joint_state
        ik_req.timeout.sec = 5
        req.ik_request = ik_req
        
        future = self.cli.call_async(req)
        future.add_done_callback(lambda fut: self.ik_callback(fut, callback))

    def ik_callback(self, future, callback: Callable):
        try:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().debug("IK solution found")
                callback(response.solution.joint_state)
            else:
                self.get_logger().warn(f"IK computation failed with error code: {response.error_code.val}")
                callback(None)
        except Exception as e:
            self.get_logger().error(f"Failed to process IK response: {e}")
            callback(None)

    def send_trajectory(self, joint_state: Optional[JointState]):
        if not joint_state:
            self.get_logger().warn("No valid joint state for trajectory")
            self.goal_reached = True
            return

        filtered_positions = [
            joint_state.position[joint_state.name.index(joint)]
            for joint in self.expected_joint_names if joint in joint_state.name
        ]

        trajectory = JointTrajectory()
        trajectory.joint_names = self.expected_joint_names
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
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Trajectory goal rejected")
            self.is_moving = False
            self.goal_reached = True
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0: # SUCCESS
            self.get_logger().info(f"Command {self.last_command} executed successfully")
        else:
            self.get_logger().warn(f"Trajectory execution failed with code: {result.error_code}")
        
        self.goal_reached = True
        self.is_moving = False
        self.goal_handle = None

    def stop_on_timeout(self):
        if not self.goal_reached and self.is_moving:
            self.get_logger().warn("Timeout reached. Cancelling current movement.")
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.goal_reached = True

    def reset_timeout_timer(self):
        self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)

    def execute_sequence(self, sequence_name: str, commands: List[Callable]):
        self.get_logger().info(f"Starting sequence: {sequence_name}")
        for i, cmd in enumerate(commands):
            self.get_logger().info(f"--- Step {i+1}/{len(commands)} ---")
            
            self.goal_reached = False
            cmd()
            
            # Blocking wait for completion
            start_time = self.get_clock().now()
            while not self.goal_reached:
                rclpy.spin_once(self, timeout_sec=0.1)
                if (self.get_clock().now() - start_time).nanoseconds > 20 * 1e9:
                    self.get_logger().error(f"Major Timeout waiting for step {i+1}")
                    break
            
            time.sleep(1.0) # Pause between moves

    def linear_sequence(self):
        commands = [
            lambda: self.move_to_frame("linear_front"),
            lambda: self.move_to_frame("front_left"),
            lambda: self.move_to_frame("angled_left"),
            lambda: self.move_to_frame("front_right"),
            lambda: self.move_to_frame("angled_right"),
            self.home_cmd
        ]
        self.execute_sequence("linear", commands)

    def omni_sequence(self):
        # IMPORTANT: Ensure these TF frames exist in your scene
        commands = [
            lambda: self.move_to_frame("omni_front"),
            lambda: self.move_to_frame("top_right"),
            lambda: self.move_to_frame("top_left"),
            lambda: self.move_to_frame("bottom_left"),
            lambda: self.move_to_frame("bottom_right"),
            self.home_cmd
        ]
        self.execute_sequence("omni", commands)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    commander = LinearCommander()
    try:
        # Choose which sequence to run
        # commander.linear_sequence()
        commander.omni_sequence()
        
    except KeyboardInterrupt:
        commander.get_logger().info('Keyboard interrupt, shutting down...')
    except Exception as e:
        commander.get_logger().error(f"Unexpected error: {e}")
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()