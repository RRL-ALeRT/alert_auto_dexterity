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

        pose_model_path = os.path.join(
            get_package_share_directory("alert_auto_dexterity"), "models", "estop_openvino_model"# "estop.onnx"
        )
        
        # Load the YOLOv8 model
        self.yolo_model = YOLO(pose_model_path, task="pose")

        static_broadcaster = StaticTransformBroadcaster(self)

        transform = TransformStamped()
        transform.header.frame_id = f'aruco_{ARUCO_ID}'
        transform.child_frame_id = 'camera_link_target'
        transform.transform.translation.z = DISTANCE_TO_ARUCO
        transform.transform.rotation.y = 1.0
        transform.transform.rotation.w = 0.0

        static_broadcaster.sendTransform(transform)

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

        self.source = 'camera_link_target'
        self.target = 'camera_link'
        
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
        GOAL_DISTANCE_TOLERANCE = 0.025
        GOAL_ANGULAR_TOLERANCE = 0.12

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
                    self.spot_twist_pub.publish(spot_twist_msg)

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
                
            if ee_twist_msg is None:
                ee_twist_msg = TwistStamped()
                ee_twist_msg.header = t.header

                if not within_distance_tolerance:
                    # Apply low-pass filter to linear components
                    ee_twist_msg.twist.linear.x = 1.5 * self.alpha * t.transform.translation.x + (1 - self.alpha) * self.prev_linear[0]
                    ee_twist_msg.twist.linear.y = 1.5 * self.alpha * t.transform.translation.y + (1 - self.alpha) * self.prev_linear[1]
                    ee_twist_msg.twist.linear.z = 1.5 * self.alpha * t.transform.translation.z + (1 - self.alpha) * self.prev_linear[2]
                    
                    linear_max = 0.2
                    ee_twist_msg.twist.linear.x = np.clip(ee_twist_msg.twist.linear.x, -linear_max, linear_max)
                    ee_twist_msg.twist.linear.y = np.clip(ee_twist_msg.twist.linear.y, -linear_max, linear_max)
                    ee_twist_msg.twist.linear.z = np.clip(ee_twist_msg.twist.linear.z, -linear_max, linear_max)

                if not within_angular_tolerance:
                    ee_twist_msg.twist.angular.x = 1.5 * self.alpha * euler_angles[0] + (1 - self.alpha) * self.prev_angular[0]
                    ee_twist_msg.twist.angular.y = 1.5 * self.alpha * euler_angles[1] + (1 - self.alpha) * self.prev_angular[1]
                    ee_twist_msg.twist.angular.z = 1.5 * self.alpha * euler_angles[2] + (1 - self.alpha) * self.prev_angular[2]

                    angular_max = 0.8

                    ee_twist_msg.twist.angular.x = np.clip(ee_twist_msg.twist.angular.x, -angular_max, angular_max)
                    ee_twist_msg.twist.angular.y = np.clip(ee_twist_msg.twist.angular.y, -angular_max, angular_max)
                    ee_twist_msg.twist.angular.z = np.clip(ee_twist_msg.twist.angular.z, -angular_max, angular_max)

            # self.get_logger().info(f"{within_distance_tolerance}, {within_angular_tolerance}")
            if within_distance_tolerance and within_angular_tolerance:
                self.target_goal_reached = True
            else:
                self.target_goal_reached = False

        except tf2_ros.TransformException as ex:
            # self.get_logger().info(f"Could not transform {self.source} to {self.target}: {ex}")

            if ee_twist_msg is None:
                # Initialize ee_twist_msg with previous decayed values when transform is not available
                ee_twist_msg = TwistStamped()
                ee_twist_msg.header.frame_id = "camera_link"
                ee_twist_msg.header.stamp = self.get_clock().now().to_msg()
                ee_twist_msg.twist.linear.x = self.prev_linear[0] * 0.5  # Adjust the decay rate as needed
                ee_twist_msg.twist.linear.y = self.prev_linear[1] * 0.5  # Adjust the decay rate as needed
                ee_twist_msg.twist.linear.z = self.prev_linear[2] * 0.5  # Adjust the decay rate as needed
                ee_twist_msg.twist.angular.x = self.prev_angular[0] * 0.5  # Adjust the decay rate as needed
                ee_twist_msg.twist.angular.y = self.prev_angular[1] * 0.5  # Adjust the decay rate as needed
                ee_twist_msg.twist.angular.z = self.prev_angular[2] * 0.5  # Adjust the decay rate as needed

        # Publish the result
        if not self.target_goal_reached:
            # Update previous values for the next iteration
            self.prev_linear = np.array([ee_twist_msg.twist.linear.x, ee_twist_msg.twist.linear.y, ee_twist_msg.twist.linear.z])
            self.prev_angular = np.array([ee_twist_msg.twist.angular.x, ee_twist_msg.twist.angular.y, ee_twist_msg.twist.angular.z])

            if not ee_within_xyz_limit:
                pass

                # ee_twist_msg = TwistStamped()
                # ee_twist_msg.header.frame_id = "gen3_base_link"
                # ee_twist_msg.header.stamp = self.get_clock().now().to_msg()

                # if spot_speed_x < 0:
                #     ee_twist_msg.twist.linear.x = 0.16
                # elif spot_speed_x > 0:
                #     ee_twist_msg.twist.linear.x = -0.16

                # if spot_yaw < 0:
                #     ee_twist_msg.twist.linear.y = 0.16
                # elif spot_yaw > 0:
                #     ee_twist_msg.twist.linear.y = -0.16

                # if change_height:
                #     if spot_height < 0:
                #         ee_twist_msg.twist.linear.z = 0.1
                #     elif spot_height > 0:
                #         ee_twist_msg.twist.linear.z = -0.1

        # else:
        #     # Update previous values for the next iteration
        #     self.prev_linear = np.array([0,0,0])
        #     self.prev_angular = np.array([0,0,0])

        #     ee_twist_msg = TwistStamped()

        # self.get_logger().info(f"{ee_twist_msg}")
        self.ee_twist_pub.publish(ee_twist_msg)

        self.tf_buffer.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
