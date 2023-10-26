import geometry_msgs.msg
import numpy as np

import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster

from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, TransformStamped, Point32


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
    

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
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


def combine_tf_transforms(input_tf2, new_child, old_child_to_new_child_translation, old_child_to_new_child_euler_rotation):
    translation = old_child_to_new_child_translation
    euler = old_child_to_new_child_euler_rotation
    quat = quaternion_from_euler(euler[2], euler[1], euler[0])

    target_pose = Pose()
    target_pose.position.x = translation[0]
    target_pose.position.y = translation[1]
    target_pose.position.z = translation[2]
    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]
    target_pose_wrt_map = do_transform_pose(target_pose, input_tf2)

    transform = geometry_msgs.msg.TransformStamped()
    transform.header = input_tf2.header
    transform.child_frame_id = new_child

    transform.transform.translation.x = target_pose_wrt_map.position.x
    transform.transform.translation.y = target_pose_wrt_map.position.y
    transform.transform.translation.z = target_pose_wrt_map.position.z

    transform.transform.rotation.x = target_pose_wrt_map.orientation.x
    transform.transform.rotation.y = target_pose_wrt_map.orientation.y
    transform.transform.rotation.z = target_pose_wrt_map.orientation.z
    transform.transform.rotation.w = target_pose_wrt_map.orientation.w

    return transform


class DynamicBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_broadcaster')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "estop_set",
                "tool_target",
                rclpy.time.Time())

            tfs = []
            tfs.append(combine_tf_transforms(t, "estop_TL", [ 0.13,  0.13, 0.08], [ 0.785, -0.785, 0.0   ]))
            tfs.append(combine_tf_transforms(t, "estop_TR", [-0.13,  0.13, 0.08], [-0.785,  0.785, 0.0   ]))
            tfs.append(combine_tf_transforms(t, "estop_BL", [ 0.13, -0.13, 0.08], [ 0.785,  0.0,   -0.785]))
            tfs.append(combine_tf_transforms(t, "estop_BR", [-0.13, -0.13, 0.08], [-0.785,  0.0,   -0.785]))
            tfs.append(combine_tf_transforms(t, "estop_M",  [ 0.0,   0.0,  0.0 ], [ 0.785,  0.0,   0.0   ]))

            self.tf_broadcaster.sendTransform(tfs)

        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return



def main():
    rclpy.init()
    rclpy.spin(DynamicBroadcaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
