import time
import rclpy
from rclpy.node import Node
from yasmin import Blackboard, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from tf2_ros import TransformException, Buffer, TransformListener

class SavePosesState(State, Node):
    def __init__(self, sequence=None):
        Node.__init__(self, "save_poses_state")
        State.__init__(self, outcomes=[SUCCEED, CANCEL, ABORT])
        set_ros_loggers()
        self.get_logger().info("SavePosesState initialized.")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sequence = sequence or [
            "linear_front", "front_right", "angled_right", "front_left", "angled_left"
        ]

    def execute(self, blackboard: Blackboard) -> str:
        transforms = {}
        for frame in self.sequence:
            tf_found = False
            start_time = time.time()
            while not tf_found and (time.time() - start_time < 5.0):
                try:
                    t = self.tf_buffer.lookup_transform(
                        "base_link",
                        frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                    transforms[frame] = [
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    ]
                    self.get_logger().info(
                        f"Stored base_link to {frame} transform: {transforms[frame]}"
                    )
                    tf_found = True
                except TransformException:
                    rclpy.spin_once(self, timeout_sec=0.1)
            if not tf_found:
                self.get_logger().info(f"Could not transform base_link to {frame} after waiting.")
                transforms[frame] = None

        blackboard["saved_poses"] = transforms
        return SUCCEED
