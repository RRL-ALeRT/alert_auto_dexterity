import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class AddSceneObjectNode(Node):

    def __init__(self):
        super().__init__('add_scene_object_node')
        self.planning_scene_service = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        # self.wait_for_service(self.planning_scene_service)
        

    def add_scene_object(self):
        scene_object = CollisionObject()
        scene_object.header.frame_id = "estop_M"
        scene_object.id = "object_allowance"
        scene_object.operation = CollisionObject.ADD

        # Define the geometry of the cube
        cube_primitive = SolidPrimitive()
        cube_primitive.type = SolidPrimitive.BOX
        cube_primitive.dimensions = [0.2, 0.2, 0.2]  # Cube size (x, y, z)

        # Define the pose of the cube
        cube_pose = Pose()
        cube_pose.position.x = 0.0  # Adjust the x position as needed
        cube_pose.position.y = 0.0  # Adjust the y position as needed
        cube_pose.position.z = 0.1  # Adjust the z position as needed

        scene_object.primitives.append(cube_primitive)
        scene_object.primitive_poses.append(cube_pose)

        planning_scene_msg = ApplyPlanningScene.Request()
        planning_scene_msg.scene.world.collision_objects.append(scene_object)
        planning_scene_msg.scene.is_diff = True

        if self.planning_scene_service.wait_for_service():
            # Send the request to add the object to the planning scene
            self.call_add_scene_object_service(planning_scene_msg)

    def call_add_scene_object_service(self, request):
        future = self.planning_scene_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Scene object added successfully")
        else:
            self.get_logger().error(f"Failed to add the scene object: {future.exception()}")


def main():
    rclpy.init()
    node = AddSceneObjectNode()
    node.add_scene_object()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
