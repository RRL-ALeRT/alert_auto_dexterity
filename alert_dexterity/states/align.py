import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from yasmin_ros import ActionState
from mbf_msgs.action import MoveBase
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

class AlignToMapOrientation(ActionState, Node):
    def __init__(self):
        Node.__init__(self, "align_to_map_orientation_node")
        ActionState.__init__(
            self,
            MoveBase,
            "/move_base_flex/move_base",
            self.create_goal_handler,
            None,
            self.response_handler,
            None,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = "odom"
        self.target_frame = "base_link"
        self.get_logger().info("AlignToMapOrientation Node Initialized")


    def create_goal_handler(self, blackboard: Blackboard) -> MoveBase.Goal:
        timeout = Duration(seconds=10.0)
        start_time = self.get_clock().now()
        while not self.tf_buffer.can_transform(
            self.base_frame, self.target_frame, Time(), Duration(seconds=0.05)
        ):
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error("Timed out waiting for transform from map to base_link")
                raise RuntimeError("Transform timeout")
            self.get_logger().info("Waiting for transform from map to base_link...")
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_frame, rclpy.time.Time(seconds=0), timeout=Duration(seconds=0.05)
            )
            goal = MoveBase.Goal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal.target_pose.pose.position.x = t.transform.translation.x
            goal.target_pose.pose.position.y = t.transform.translation.y
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
            return goal
        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            raise

    def response_handler(self, blackboard: Blackboard, response: MoveBase.Result) -> str:
        return SUCCEED
