#!/usr/bin/env python3

import rclpy

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

import os
import openvino as ov
import cv2
from yolov8_pose_utils import *

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped, Point32

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv_bridge
from sensor_msgs.msg import Image
from world_info_msgs.msg import BoundingBox, BoundingBoxArray, Keypoints, KeypointsArray

from ament_index_python.packages import get_package_share_directory

DEBUG = False

# Relative 3D coordinates of 5 points with the first point as the origin (0,0,0)
# TL, TR, BL, BR, M
relative_points = np.array([
    [-0.059/2,  0.059/2, 0],
    [ 0.059/2,  0.059/2, 0],
    [-0.059/2, -0.059/2, 0],
    [ 0.059/2, -0.059/2, 0]], dtype=np.float32
)

IMAGE_TOPIC = '/kinova_color'
SCORE_THRESHOLD = 0.8
BOUNDING_BOX_EDGE_LIMIT = 20

fx = 653.68229
fy = 651.855994
cx = 311.753415
cy = 232.400954
# Camera intrinsic parameters
focal_length = (fx, fy)  # Focal length in pixels
principal_point = (cx, cy)  # Principal point (center of the image) in pixels
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

k1 = k2 = p1 = p2 = k3 = 0
# Distortion coefficients (if any)
dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

# Load a model
device = "GPU"
kpt_shape = [5, 2]
label_map = ["estop"]


def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class LifecyclePoseNode(LifecycleNode):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        self.bounding_box_pub = None
        self.keypoints_pub = None
        self.tf2_publish_timer = None
        # self.trigger_configure()

    def on_configure(self, state: LifecycleState):
        self.bounding_box_pub = self.create_lifecycle_publisher(BoundingBoxArray, IMAGE_TOPIC + '/bb', 1)
        self.keypoints_pub = self.create_lifecycle_publisher(KeypointsArray, IMAGE_TOPIC + '/kp', 1)

        self.t = None
        self.closest_estop = None
        self.closest_distance = float('inf')
        self.tf2_publish_timer = self.create_timer(0.1, self.publish_tf2_continuously)

        self.camera_sub = self.create_subscription(Image, IMAGE_TOPIC, self.camera_cb, 1)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        pose_model_path = os.path.join(get_package_share_directory("alert_auto_dexterity"), "models", "estop.onnx")

        core = ov.Core()
        pose_ov_model = core.read_model(pose_model_path)

        if device != "CPU":
            pose_ov_model.reshape({0: [1, 3, 640, 640]})
        self.compiled_model = core.compile_model(pose_ov_model, device)

        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("on_activate() is called.")
        self.t = None
        self.tf_buffer.clear()

        return super().on_activate(state)
  
    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("on_deactivate() is called.")
        
        self.t = None
        self.closest_estop = None
        self.closest_distance = float('inf')

        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState):
        self.destroy_publisher(self.bounding_box_pub)
        self.destroy_publisher(self.keypoints_pub)
        self.destroy_timer(self.tf2_publish_timer)
        self.destroy_subscription(self.camera_sub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.destroy_publisher(self.bounding_box_pub)
        self.destroy_publisher(self.keypoints_pub)
        self.destroy_timer(self.tf2_publish_timer)

        self.destroy_node()

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

    def camera_cb(self, msg):
        if self.bounding_box_pub is None or not self.bounding_box_pub.is_activated:
            return

        if self.t is not None:
            return
        
        image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, 'bgr8')

        input_image = image[:, :, ::-1]
        results = detect(input_image, self.compiled_model, kpt_shape)

        image_width = image.shape[1]
        image_height = image.shape[0]

        bb_msg = BoundingBoxArray()
        bb_msg.header = msg.header
        bb_msg.type = label_map[0]

        kp_msg = KeypointsArray()
        kp_msg.header = msg.header
        kp_msg.type = label_map[0]

        detection = results[0]

        center_x = image.shape[1] // 2
        center_y = image.shape[0] // 2

        idx = 0
        for box, keypoints in zip(detection['box'], detection['kpt']):
            for keypoint in keypoints:
                distance = calculate_distance(keypoint, (center_x, center_y))
                if distance < self.closest_distance:
                    self.closest_distance = distance
                    self.closest_estop = idx
            idx += 1

        idx = self.closest_estop

        if self.closest_estop is not None:
            box = detection['box'][idx]
            keypoints = detection['kpt'][idx]

            x1, y1, x2, y2, score, label = box

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

            image_points = np.array([[u1, v1],
                                     [u2, v2],
                                     [u3, v3],
                                     [u4, v4]], dtype=np.float32)

            label = int(label)
            label_name = label_map[label] if label < len(label_map) else 'Unknown'
            label_str = f'{label_name}: {score:.2f}'  # Create a label string

            if DEBUG:
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                # Draw the bounding box label
                cv2.putText(image, label_str, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)

                for point in image_points:
                    image = cv2.circle(image, (int(point[0]), int(point[1])), 1, (0,0,255), 5)

            bb = BoundingBox()
            bb.name = label_map[0] + str(idx)
            bb.confidence = float(score)
            bb.cx = float(x1)
            bb.cy = float(y1)
            bb.width = float(abs(x2 - x1))
            bb.height = float(abs(y2 - y1))
            bb_msg.array.append(bb)

            kp = Keypoints()
            kp.name = label_map[0] + str(idx)
            for image_kp in image_points:
                point = Point32()
                point.x = float(image_kp[0])
                point.y = float(image_kp[1])
                kp.array.append(point)
            kp_msg.array.append(kp)

            # Use solvePnP to estimate camera pose
            # success, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(relative_points, image_points, camera_matrix, dist_coeffs)
            success, rotation_vector, translation_vector = cv2.solvePnP(relative_points, image_points, camera_matrix, dist_coeffs)

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Convert the rotation matrix to a quaternion
            r = R.from_matrix(rotation_matrix)
            quaternion = r.as_quat()

            self.build_tf2(translation_vector, quaternion, msg.header.frame_id)

        self.bounding_box_pub.publish(bb_msg)
        self.keypoints_pub.publish(kp_msg)

        if DEBUG:
            cv2.imshow("", image)
            cv2.waitKey(1)

    def build_tf2(self, xyz, quat, header_frame_id):
        self.t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        self.t.header.frame_id = 'camera_link'
        self.t.child_frame_id = label_map[0]

        self.t.transform.translation.x = xyz[0][0]
        self.t.transform.translation.y = xyz[1][0]
        self.t.transform.translation.z = xyz[2][0]

        self.t.transform.rotation.x = quat[0]
        self.t.transform.rotation.y = quat[1]
        self.t.transform.rotation.z = quat[2]
        self.t.transform.rotation.w = quat[3]

    def publish_tf2_continuously(self):
        if self.t is not None:
            # Send the transformation
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.t)


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecyclePoseNode('find_object_pose')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()