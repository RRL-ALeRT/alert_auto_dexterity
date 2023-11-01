import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from octomap_msgs.msg import Octomap


class MoveIt2PlanningSceneNode(Node):
    def __init__(self):
        super().__init__("moveit2_planning_scene_node")
        self.apply_planning_scene_service = self.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )
        while not self.apply_planning_scene_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.octomap_subscription = self.create_subscription(
            Octomap, "/octomap_full", self.octomap_callback, 10
        )
        self.latest_octomap = None

    def octomap_callback(self, msg):
        self.latest_octomap = msg
        self.apply_planning_scene()
        rclpy.shutdown()

    def apply_planning_scene(self):
        if self.latest_octomap is not None:
            planning_scene_msg = PlanningScene()
            planning_scene_msg.is_diff = True
            planning_scene_msg.world.octomap.header = self.latest_octomap.header
            planning_scene_msg.world.octomap.octomap = self.latest_octomap

            request = ApplyPlanningScene.Request()
            request.scene = planning_scene_msg

            future = self.apply_planning_scene_service.call_async(request)

            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info("Planning Scene applied successfully")
            else:
                self.get_logger().error("Failed to apply Planning Scene")


def main(args=None):
    rclpy.init(args=args)
    planning_scene_node = MoveIt2PlanningSceneNode()

    rclpy.spin(planning_scene_node)
    planning_scene_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
