import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK


class MoveitIKClientAsync(Node):
    def __init__(self):
        super().__init__('moveit_ik')

        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, '/compute_ik')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            rclpy.spin_once(self)

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def moveit_ik(self, x, y, z, qx, qy, qz, qw):
        self.req.ik_request.group_name = 'manipulator'
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = 'base_link'

        self.req.ik_request.pose_stamped.pose.position.x = x
        self.req.ik_request.pose_stamped.pose.position.y = y
        self.req.ik_request.pose_stamped.pose.position.z = z
        
        self.req.ik_request.pose_stamped.pose.orientation.x = qx
        self.req.ik_request.pose_stamped.pose.orientation.y = qy
        self.req.ik_request.pose_stamped.pose.orientation.z = qz
        self.req.ik_request.pose_stamped.pose.orientation.w = qw

        self.req.ik_request.timeout.sec = 10

        return self.req.ik_request

    def send_request(self, x, y, z, qx, qy, qz, qw):
        self.req = GetPositionIK.Request()
        self.joint_state = None

        while self.joint_state is None:
            rclpy.spin_once(self)

        self.req.ik_request = self.moveit_ik(x, y, z, qx, qy, qz, qw)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        target_angles = list(self.future.result().solution.joint_state.position)[:6]

        if len(target_angles) > 0:
            return target_angles

        self.get_logger().warn("No ik soln found")
        return None
