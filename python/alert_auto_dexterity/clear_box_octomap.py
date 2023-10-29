import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tf2_ros
from octomap_msgs.srv import BoundingBoxQuery


class OctomapClearer(Node):

    def __init__(self):
        super().__init__('octomap_clearer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.done = False
        self.clear_octomap_service = self.create_client(BoundingBoxQuery, '/octomap_server/clear_bbox')

        self.create_timer(0.5, self.clear_octomap_near_object)

    def clear_octomap_near_object(self):
        try:
            while not self.clear_octomap_service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

            transform = self.tf_buffer.lookup_transform('odom', 'P', rclpy.time.Time())

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            size = 1.0

            min_point = Point()
            min_point.x = x-size/2
            min_point.y = y-size/2
            min_point.z = z-size/2
            max_point = Point()
            max_point.x = x+size/2
            max_point.y = y+size/2
            max_point.z = z+size/2

            req = BoundingBoxQuery.Request()
            req.min = min_point
            req.max = max_point

            self.clear_octomap_service.call_async(req)
            exit()

        except Exception as e:
            self.get_logger().error(f'{e}')

def main(args=None):
    rclpy.init(args=args)
    clearer = OctomapClearer()
    rclpy.spin(clearer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
