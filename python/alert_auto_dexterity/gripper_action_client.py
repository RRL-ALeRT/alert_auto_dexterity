from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand

import time


class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_execute_python')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

    def send_goal(self, position):
        if position == "open":
            t = 0.0
        elif position == "close":
            t = 0.8

        self.goal_done = False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = t
        goal_msg.command.max_effort = 10.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.goal_done = True
        time.sleep(1)
