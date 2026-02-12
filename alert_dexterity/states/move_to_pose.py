import time
import rclpy
from rclpy.node import Node
import yasmin
from yasmin import Blackboard, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from sensor_msgs.msg import JointState
from tf2_ros import TransformException, Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import JointConstraint, Constraints, PlanningOptions, MotionPlanRequest
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
from copy import deepcopy

target_angles = None

def skip_extra_rotations(j):
    k = []
    for f in j:
        if f > np.pi:
            f -= 2 * np.pi
        elif f < -np.pi:
            f += 2 * np.pi
        k.append(f)
    return k
class MoveGroupActionClient(Node):
    def __init__(self, node, target_angles, sequence=None):
        
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = "base_link"
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        j= skip_extra_rotations(target_angles)

        joints = {}
        joints["joint_1"] = j[0]
        joints["joint_2"] = j[1]
        joints["joint_3"] = j[2]
        joints["joint_4"] = j[3]
        joints["joint_5"] = j[4]
        joints["joint_6"] = 0.0

        constraints = Constraints()
        for joint, angle in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))
        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.planner_id = "move_group"
        self.motion_plan_request.group_name = "manipulator"
        self.motion_plan_request.num_planning_attempts = 5
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.5
        self.motion_plan_request.max_acceleration_scaling_factor = 0.5
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = True
        self.planning_options.look_around_attempts = 5
        self.planning_options.max_safe_execution_cost = 0.0
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 5
        self.planning_options.replan_delay = rclpy.duration.Duration(seconds=1.0)

        self._action_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )
    def send_goal(self, blackboard: Blackboard):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.request.planning_options = self.planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal = future.result()
        if not goal.accepted:
            self.get_logger().error("Goal was rejected by the action server")
            return
        self.get_logger().info("Goal accepted by the action server")
        self._get_result_future = goal.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self._logger.info(str(future))

    def feedback_callback(self, feedback):
        self._logger.info(str(feedback))
        
class MoveToPoseState(State, Node):
    def __init__(self, sequence=None):
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
        self.sequence = sequence or [
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
                    "base_link", self.sequence[0], rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                self.tf_base_link_pipe = [
                    t.transform.translation.x - 0.3,  # Adjusted x position
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

        # 2. Compute IK and send trajectories using stored transforms
        transforms = blackboard["saved_poses"] if "saved_poses" in blackboard else {}
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
            req.ik_request.timeout.sec = 7

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
    
def retract_manipulator(blackboard: Blackboard) -> str:
    arm = MoveToPoseState()
    arm.arm_client.wait_for_server()
    
    arm_goal_msg = FollowJointTrajectory.Goal()
    arm_goal_msg.trajectory.joint_names = arm.arm_joint_names
    arm_point = JointTrajectoryPoint()
    arm_point.positions = [
        0.0,
        -1.9,
        -2.54,
        0.0,
        -1.0,
        1.57]  # Retract to home position
    arm_point.velocities = [0.0] * 6
    arm_point.time_from_start.sec = 3
    arm_goal_msg.trajectory.points.append(arm_point)
    arm.arm_client.send_goal_async(arm_goal_msg)
    yasmin.YASMIN_LOG_INFO("Manipulator retracted.")
    return SUCCEED
