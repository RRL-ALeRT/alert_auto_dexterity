import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, TransformStamped, Twist, TwistStamped, Quaternion
import tf2_ros
import tf2_py
import transforms3d as t3d
from transforms3d.euler import quat2euler
from geometry_msgs.msg import Vector3
from tf2_geometry_msgs import do_transform_pose
import tf2_geometry_msgs
from tf2_ros import StaticTransformBroadcaster

from moveit_msgs.msg import ServoStatus

import time
import math
from scipy.spatial.transform import Rotation as R

from world_info_msgs.msg import BoundingPolygon, BoundingPolygonArray
from world_info_msgs.msg import BoundingBox, BoundingBoxArray, Keypoints, KeypointsArray
from geometry_msgs.msg import Point32

from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from copy import deepcopy

from tf2_ros import StaticTransformBroadcaster

ARUCO_ID = 25
ARUCO_SIZE = 0.059
DISTANCE_TO_ARUCO = 0.35
label_map = ["estop"]

relative_points = np.array(
    [
        [-ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
        [ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
        [ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0],
        [-ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0],
    ],
    dtype=np.float32,
)


class Moveit(Node):
    def __init__(self):
        super().__init__("moveit_plan_execute_python")

        self._action_client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self, target_angles):
        self.joint_state = None
        self.goal_done = False

        motion_plan_request = MotionPlanRequest()

        motion_plan_request.workspace_parameters.header.stamp = (
            self.get_clock().now().to_msg()
        )
        motion_plan_request.workspace_parameters.header.frame_id = "gen3_base_link"
        motion_plan_request.workspace_parameters.min_corner.x = -1.0
        motion_plan_request.workspace_parameters.min_corner.y = -1.0
        motion_plan_request.workspace_parameters.min_corner.z = -1.0
        motion_plan_request.workspace_parameters.max_corner.x = 1.0
        motion_plan_request.workspace_parameters.max_corner.y = 1.0
        motion_plan_request.workspace_parameters.max_corner.z = 1.0
        motion_plan_request.start_state.is_diff = True

        jc = JointConstraint()
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        joints = {}
        joints["joint_1"] = target_angles[0]
        joints["joint_2"] = target_angles[1]
        joints["joint_3"] = target_angles[2]
        joints["joint_4"] = target_angles[3]
        joints["joint_5"] = target_angles[4]
        joints["joint_6"] = target_angles[5]

        constraints = Constraints()
        for joint, angle in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        motion_plan_request.goal_constraints.append(constraints)

        motion_plan_request.pipeline_id = "ompl"
        # motion_plan_request.planner_id = "STOMP"
        motion_plan_request.group_name = "manipulator"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 10.0
        motion_plan_request.max_velocity_scaling_factor = 0.5
        motion_plan_request.max_acceleration_scaling_factor = 0.5
        motion_plan_request.max_cartesian_speed = 0.0

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.look_around = False
        planning_options.look_around_attempts = 0
        planning_options.max_safe_execution_cost = 0.0
        planning_options.replan = True
        planning_options.replan_attempts = 10
        planning_options.replan_delay = 0.1

        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_plan_request
        goal_msg.planning_options = planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.goal_done = True
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # self.get_logger().info(str(future))
        self.goal_done = True

    def feedback_callback(self, feedback_msg):
        # self.get_logger().info(str(feedback_msg))
        pass


class IK(Node):
    def __init__(self):
        super().__init__("moveit_ik")

        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, "/compute_ik")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            rclpy.spin_once(self)

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def send_request(self, pose):
        self.joint_state = None

        while self.joint_state is None:
            rclpy.spin_once(self)

        req = GetPositionIK.Request()
        
        req.ik_request.group_name = "manipulator"
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True

        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.header.frame_id = "gen3_base_link"

        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout.sec = 10

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

        target_angles = list(self.future.result().solution.joint_state.position)[:6]

        if len(target_angles) > 0:
            return target_angles

        self.get_logger().warn("No ik soln found")
        return None


def moveit_motion(pose):
    ik = IK()
    moveit = Moveit()
    target_angles = ik.send_request(pose)
    ik.destroy_node()
    if target_angles != None:
        moveit.send_goal(target_angles)
        while not moveit.goal_done:
            rclpy.spin_once(moveit)
        moveit.destroy_node()
        return target_angles
    moveit.destroy_node()
    return target_angles


def compute_end_effector_twist(body_twist, body_to_ee_transform, ee_to_body_transform):
    # Convert Quaternion to Rotation Matrix for both transformations
    quat_ab = [body_to_ee_transform.transform.rotation.x,
               body_to_ee_transform.transform.rotation.y,
               body_to_ee_transform.transform.rotation.z,
               body_to_ee_transform.transform.rotation.w]
    r_ab = R.from_quat(quat_ab)
    rotation_matrix_ab = r_ab.as_matrix()

    quat_ba = [ee_to_body_transform.transform.rotation.x,
               ee_to_body_transform.transform.rotation.y,
               ee_to_body_transform.transform.rotation.z,
               ee_to_body_transform.transform.rotation.w]
    r_ba = R.from_quat(quat_ba)
    rotation_matrix_ba = r_ba.as_matrix()

    # Calculate translation for both transformations
    translation_ab = np.array([body_to_ee_transform.transform.translation.x,
                                body_to_ee_transform.transform.translation.y,
                                body_to_ee_transform.transform.translation.z])

    translation_ba = np.array([ee_to_body_transform.transform.translation.x,
                                ee_to_body_transform.transform.translation.y,
                                ee_to_body_transform.transform.translation.z])

    # Construct the twist in the body frame
    body_twist_vector = np.array([body_twist.linear.x, body_twist.linear.y, body_twist.linear.z,
                                  body_twist.angular.x, body_twist.angular.y, body_twist.angular.z])

    # Construct the twist in the end effector frame using the relation Sa = [Ad(Tab)] Sb
    # Adjoint transformation for Tab
    adjoint_ab = np.zeros((6, 6))
    adjoint_ab[:3, :3] = rotation_matrix_ab
    adjoint_ab[3:6, 3:6] = rotation_matrix_ab
    adjoint_ab[:3, 3:6] = np.dot(rotation_matrix_ab.T, np.cross(-translation_ab, rotation_matrix_ab))

    # Adjoint transformation for Tba
    adjoint_ba = np.zeros((6, 6))
    adjoint_ba[:3, :3] = rotation_matrix_ba
    adjoint_ba[3:6, 3:6] = rotation_matrix_ba
    adjoint_ba[:3, 3:6] = np.dot(rotation_matrix_ba.T, np.cross(-translation_ba, rotation_matrix_ba))

    # Apply the relation Sa = [Ad(Tab)] Sb
    end_effector_twist_vector = np.dot(adjoint_ab, np.dot(adjoint_ba, -body_twist_vector))

    # Construct the Twist message for the end effector
    end_effector_twist = Twist()
    end_effector_twist.linear.x = -end_effector_twist_vector[0]
    end_effector_twist.linear.y = -end_effector_twist_vector[1]
    end_effector_twist.linear.z = -end_effector_twist_vector[2]
    end_effector_twist.angular.x = -end_effector_twist_vector[3]
    end_effector_twist.angular.y = -end_effector_twist_vector[4]
    end_effector_twist.angular.z = -end_effector_twist_vector[5]

    return end_effector_twist


class CameraLinkPose:
    def __init__(self, tf_buffer, logger):
        self.logger = logger
        self.tf_buffer = tf_buffer

        self.nominal_x_position = 0.617
        self.min_x_position = 0.61
        self.max_x_position = 0.8

        self.nominal_y_position = 0.0
        self.min_y_position = self.nominal_y_position - 0.15
        self.max_y_position = self.nominal_y_position + 0.15

        self.nominal_z_position = 0.6
        self.min_z_position = self.nominal_z_position - 0.1
        self.max_z_position = self.nominal_z_position + 0.1

        self.source = "camera_link"
        self.target = "body"

        self.height_source = "camera_link"
        self.height_target = "gpe"

        self.current_spot_height = 0.0
        self.new_spot_height = 0.0

    def is_ee_within_xyz_limit(self):
        SPOT_SPEED_TR = 0.16
        SPOT_SPEED_ROT = 0.16

        change_height = False

        try:
            t = self.tf_buffer.lookup_transform(
                self.target, self.source, rclpy.time.Time()
            )

            x = 0.0
            if t.transform.translation.x < self.min_x_position:
                x = -SPOT_SPEED_TR
            elif t.transform.translation.x > self.max_x_position:
                x = SPOT_SPEED_TR

            y = 0.0
            if t.transform.translation.y < self.min_y_position:
                y = -SPOT_SPEED_ROT
            elif t.transform.translation.y > self.max_y_position:
                y = SPOT_SPEED_ROT

            DELTA_HEIGHT = 0.005
            if t.transform.translation.z < self.min_z_position:
                self.new_spot_height -= DELTA_HEIGHT
            elif t.transform.translation.z > self.max_z_position:
                self.new_spot_height += DELTA_HEIGHT
            
            # self.logger.info(f"{self.new_spot_height}")

            MIN_SPOT_HEIGHT = -0.04
            MAX_SPOT_HEIGHT = 0.04
            self.new_spot_height = np.clip(self.new_spot_height, MIN_SPOT_HEIGHT, MAX_SPOT_HEIGHT)

            t1 = self.tf_buffer.lookup_transform(
                self.source, self.target, rclpy.time.Time()
            )

            if x != 0 or y != 0 or self.current_spot_height != self.new_spot_height:
                if self.current_spot_height != self.new_spot_height:
                    self.current_spot_height = self.new_spot_height
                    change_height = True
                return False, change_height, x, y, self.current_spot_height, [t1, t]

            return True, change_height, 0.0, 0.0, self.current_spot_height, [t1, t]

        except tf2_ros.TransformException as ex:
            # self.get_logger().info(f"Could not transform {self.source} to {self.target}: {ex}")
            pass

        return False, change_height, 0.0, 0.0, self.current_spot_height, None



def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    
class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        static_broadcaster = StaticTransformBroadcaster(self)

        transform1 = TransformStamped()
        transform1.header.frame_id = f'aruco_{ARUCO_ID}'
        transform1.child_frame_id = 'camera_link_target'
        transform1.transform.translation.z = DISTANCE_TO_ARUCO
        transform1.transform.rotation.y = 1.0
        transform1.transform.rotation.w = 0.0

        transform2 = TransformStamped()
        transform2.header.frame_id = 'camera_link_target'
        transform2.child_frame_id = 'tool_frame_target'
        transform2.transform.translation.y = 0.056
        transform2.transform.translation.z = 0.203
        transform2.transform.rotation.z = 1.0
        transform2.transform.rotation.w = 0.0

        static_broadcaster.sendTransform([transform1, transform2])

        pose_model_path = os.path.join(
            get_package_share_directory("alert_auto_dexterity"), "models", "estop_openvino_model"# "estop.onnx"
        )
        
        # Load the YOLOv8 model
        self.yolo_model = YOLO(pose_model_path, task="pose")

        self.bridge = CvBridge()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image, '/kinova_color', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/kinova_color/camera_info', self.camera_info_callback, 1)

        self.servo_status_sub = self.create_subscription(ServoStatus, '/servo_node/status', self.servo_status_cb, 1)

        # self.create_timer(1/50, self.step)
        self.ee_twist_pub = self.create_publisher(TwistStamped, "/twist_controller/commands", 1)
        self.spot_twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.spot_body_pose_pub = self.create_publisher(Pose, "/body_pose", 1)

        self.bounding_polygon_pub = self.create_publisher(BoundingPolygonArray, "/kinova_color/bp", 1)

        IMAGE_TOPIC = "/kinova_color"

        self.bounding_box_pub = self.create_publisher(
            BoundingBoxArray, IMAGE_TOPIC + "/bb", 1
        )
        self.keypoints_pub = self.create_publisher(
            KeypointsArray, IMAGE_TOPIC + "/kp", 1
        )

        ar_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_detector = cv2.aruco.ArucoDetector(ar_dict)

        self.source = 'tool_frame_target'
        self.target = 'gen3_base_link'
        
        # Initialize previous values for smoothing
        self.prev_linear = np.zeros(3)
        self.prev_angular = np.zeros(3)
        self.alpha = 0.2  # Smoothing factor (tune this value)

        self.target_goal_reached = False

        self.camera_pose = CameraLinkPose(self.tf_buffer, self.get_logger())

        self.pause = False

    def servo_status_cb(self, msg):
        status = msg.code
        if status == 0:
            self.pause = False
            return

        self.pause = True
        self.get_logger().warn(f"{status}")
        time.sleep(0.1)

    def image_callback(self, msg):
        if not hasattr(self, 'dist_coeff'):
            return

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.yolo_model.track(image, persist=True, verbose=False)

        boxes = results[0].boxes.xyxy.cpu()

        if results[0].boxes.id is not None:
            detection = {}
            detection["box"] = results[0].boxes.xywh.cpu().tolist()
            detection["kpt"] = results[0].keypoints.xy.cpu().tolist()
            detection["score"] = results[0].boxes.conf.float().cpu().tolist()
            detection["tracker_id"] = results[0].boxes.id.int().cpu().tolist()

            bb_msg = BoundingBoxArray()
            bb_msg.header = msg.header
            bb_msg.type = label_map[0]

            kp_msg = KeypointsArray()
            kp_msg.header = msg.header
            kp_msg.type = label_map[0]

            # detection = results[0]

            center_x = image.shape[1] // 2
            center_y = image.shape[0] // 2

            closest_distance = float("inf")
            closest_estop = None

            idx = 0
            for box, keypoints in zip(detection["box"], detection["kpt"]):
                for keypoint in keypoints:
                    distance = calculate_distance(keypoint, (center_x, center_y))
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_estop = idx
                idx += 1

            idx = closest_estop

            if closest_estop is not None:
                box = detection["box"][idx]
                keypoints = detection["kpt"][idx]
                score = detection["score"][idx]
                label = detection["tracker_id"][idx]

                x1, y1, x2, y2 = box

                u1 = keypoints[0][0]
                v1 = keypoints[0][1]
                u2 = keypoints[1][0]
                v2 = keypoints[1][1]
                u3 = keypoints[2][0]
                v3 = keypoints[2][1]
                u4 = keypoints[3][0]
                v4 = keypoints[3][1]
                u5 = keypoints[4][0]
                v5 = keypoints[4][1]

                image_points = np.array(
                    [[u1, v1], [u2, v2], [u4, v4], [u3, v3]], dtype=np.float32
                )

                label = int(label)
                label_name = label_map[label] if label < len(label_map) else "Unknown"
                label_str = f"{label_name}: {score:.2f}"  # Create a label string

                bb = BoundingBox()
                bb.name = label_map[0] + str(idx)
                bb.confidence = float(score)
                bb.cx = float(x1-x2/2)
                bb.cy = float(y1-y2/2)
                bb.width = float(x2)
                bb.height = float(y2)
                bb_msg.array.append(bb)

                kp = Keypoints()
                kp.name = label_map[0] + str(idx)
                for image_kp in image_points:
                    point = Point32()
                    point.x = float(image_kp[0])
                    point.y = float(image_kp[1])
                    kp.array.append(point)
                kp_msg.array.append(kp)

                _, rvec, tvec = cv2.solvePnP(
                    relative_points, image_points, self.camera_matrix, self.dist_coeff
                )
                transform = self.calculate_transform(rvec, tvec)
                self.publish_transform(transform, msg.header, ARUCO_ID)

            self.bounding_box_pub.publish(bb_msg)
            self.keypoints_pub.publish(kp_msg)

        self.step()

    def camera_info_callback(self, msg):
        if hasattr(self, 'dist_coeff'):
            return
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeff = np.array(msg.d)

    def calculate_transform(self, rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        translation_vector = np.array(tvec).reshape((3, 1))

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = translation_vector.flatten()

        return transform

    def publish_transform(self, transform, header, aruco_id):
        tf_msg = TransformStamped()
        tf_msg.header = header
        tf_msg.child_frame_id = f'aruco_{aruco_id}'
        tf_msg.transform.translation.x = transform[0, 3]
        tf_msg.transform.translation.y = transform[1, 3]
        tf_msg.transform.translation.z = transform[2, 3]

        rotation_matrix = transform[:3, :3]
        rotation_quaternion = t3d.quaternions.mat2quat(rotation_matrix)
        tf_msg.transform.rotation.x = -rotation_quaternion[2]
        tf_msg.transform.rotation.y = rotation_quaternion[1]
        tf_msg.transform.rotation.z = -rotation_quaternion[0]
        tf_msg.transform.rotation.w = rotation_quaternion[3]

        self.tf_broadcaster.sendTransform(tf_msg)
            
    def step(self):
        GOAL_DISTANCE_TOLERANCE = 4.0
        GOAL_ANGULAR_TOLERANCE = 1.5

        ee_twist_msg = None

        ee_within_xyz_limit, change_height, spot_speed_x, spot_yaw, spot_height, t_body_camera_link = self.camera_pose.is_ee_within_xyz_limit()
        # self.get_logger().info(f"{ee_within_xyz_limit}, {x}")
        if not ee_within_xyz_limit:
            if spot_speed_x != 0 or spot_yaw != 0 or change_height:
                spot_twist_msg = Twist()
                spot_twist_msg.linear.x = spot_speed_x
                spot_twist_msg.angular.z = spot_yaw
                # spot_twist_msg.linear.y = spot_yaw

                body_pose_msg = Pose()
                # self.get_logger().info(f"{spot_height}")
                body_pose_msg.position.z = spot_height

                if not self.pause:
                    self.spot_body_pose_pub.publish(body_pose_msg)
                    # self.spot_twist_pub.publish(spot_twist_msg)

                    ee_twist_msg = TwistStamped()
                    ee_twist_msg.header.frame_id = "body"
                    ee_twist_msg.header.stamp = self.get_clock().now().to_msg()

                    ee_twist_msg.twist.linear.x = -2.0*spot_twist_msg.linear.x
                    ee_twist_msg.twist.linear.y = -4.0*spot_twist_msg.angular.z

                    if t_body_camera_link is not None:
                        t_body_camera_link, t_camera_link_body = t_body_camera_link
                        # ee_twist_msg.twist = compute_end_effector_twist(spot_twist_msg, t_body_camera_link, t_camera_link_body)
                        
                        # print(spot_twist_msg, ee_twist_msg)

        try:
            t = self.tf_buffer.lookup_transform(
                self.target, self.source, rclpy.time.Time()
            )

            within_distance_tolerance = False
            if abs(t.transform.translation.x) < GOAL_DISTANCE_TOLERANCE and \
            abs(t.transform.translation.y) < GOAL_DISTANCE_TOLERANCE and \
            abs(t.transform.translation.z) < GOAL_DISTANCE_TOLERANCE:
                within_distance_tolerance = True

            # Apply low-pass filter to angular components
            euler_angles = quat2euler([
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z
            ])

            within_angular_tolerance = False
            if abs(euler_angles[0]) < GOAL_ANGULAR_TOLERANCE and \
            abs(euler_angles[1]) < GOAL_ANGULAR_TOLERANCE and \
            abs(euler_angles[2]) < GOAL_ANGULAR_TOLERANCE:
                within_angular_tolerance = True

            # self.get_logger().info(f"{within_distance_tolerance}, {within_angular_tolerance}")
            if within_distance_tolerance and within_angular_tolerance:
                self.target_goal_reached = True
            else:
                pose_goal = Pose()
                pose_goal.position.x = t.transform.translation.x
                pose_goal.position.y = t.transform.translation.y 
                pose_goal.position.z = t.transform.translation.z 
                pose_goal.orientation.x = t.transform.rotation.x
                pose_goal.orientation.y = t.transform.rotation.y
                pose_goal.orientation.z = t.transform.rotation.z
                pose_goal.orientation.w = t.transform.rotation.w

                self.tf_buffer.clear()

                moveit_motion(pose_goal)

                self.target_goal_reached = False
            self.get_logger().info(f"{self.target_goal_reached}")

        except tf2_ros.TransformException as ex:
            self.get_logger().info(f"Could not transform {self.source} to {self.target}: {ex}")
            pass

        # self.ee_twist_pub.publish(ee_twist_msg)
        # self.tf_buffer.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
