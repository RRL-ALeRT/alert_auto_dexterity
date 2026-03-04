import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK


class MoveitIKClientAsync(Node):
    
    def __init__(self):
        super().__init__('moveit_ik_client_async')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')

    def send_request(self, target_frame):
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        try:
            transform = self.tf_buffer.lookup_transform('base_link', target_frame, rclpy.time.Time())
            request.ik_request.pose_stamped.pose = transform.transform.translation
            request.ik_request.pose_stamped.pose.orientation = transform.transform.rotation
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {target_frame} to base_link: {ex}')
            return None     
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().error_code.val == future.result().error_code.SUCCESS:
                joint_angles = future.result().solution.joint_state.position
                self.get_logger().info(f'IK solution found: {joint_angles}')
                return joint_angles
            else:
                self.get_logger().error(f'IK service returned error code: {future.result().error_code.val}')
                return None
        else:
            self.get_logger().error('Failed to call IK service')
            return None