#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
import tf2_ros

from alert_auto_dexterity.action import StandInFront

import threading
import numpy as np

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            StandInFront,
            'stand_in_front',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # PID control constants
        self.kp_linear = 0.8  # Proportional gain for linear velocity
        self.ki_linear = 0.0  # Integral gain for linear velocity (initially set to 0)
        self.kd_linear = 0.0003  # Derivative gain for linear velocity (initially set to 0)

        self.kp_angular = 1.5  # Proportional gain for angular velocity
        self.ki_angular = 0.0001  # Integral gain for angular velocity (initially set to 0)
        self.kd_angular = 0.0003  # Derivative gain for angular velocity (initially set to 0)

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        # Initialize PID controller variables
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0
        
        self.reach_time = self.get_clock().now().to_msg().sec

        self.goal_done = False

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')
        while not self.goal_done:
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return StandInFront.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return StandInFront.Result()
            
            self.move_robot(goal_handle)

            # self.create_rate(100).sleep()

        goal_handle.succeed()

        # Populate result message
        result = StandInFront.Result()

        return result

    def move_robot(self, goal_handle):
        try:
            transform = self.tf_buffer.lookup_transform(
                'body',
                'spot_target',
                rclpy.time.Time()
            )

            # Calculate position errors in X and Y
            error_x = transform.transform.translation.x
            error_y = transform.transform.translation.y

            # Calculate linear velocity for position control using PID controller
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.calculate_pid_linear(error_x)
            cmd_vel_msg.linear.y = self.calculate_pid_linear(error_y)

            transform = self.tf_buffer.lookup_transform(
                'body',
                'estop_set',
                rclpy.time.Time()
            )
            error_angle = np.arctan2(transform.transform.translation.y, transform.transform.translation.x)

            feedback_msg = StandInFront.Feedback()
            feedback_msg.error_x = error_x
            feedback_msg.error_y = error_y
            feedback_msg.error_angle = error_angle
            goal_handle.publish_feedback(feedback_msg)

            # Calculate angular velocity for orientation control using PID controller
            cmd_vel_msg.angular.z = self.calculate_pid_angular(error_angle)

            # Set upper and lower limits
            cmd_vel_msg.linear.x = max(-0.25, min(cmd_vel_msg.linear.x, 0.25))
            cmd_vel_msg.linear.y = max(-0.25, min(cmd_vel_msg.linear.y, 0.25))
            cmd_vel_msg.angular.z = max(-0.4, min(cmd_vel_msg.angular.z, 0.4))

            # Condition to stop reaching to Spot Target
            if abs(error_x) < 0.2 and abs(error_y) < 0.2 and abs(error_angle) < 0.1:
                if self.get_clock().now().to_msg().sec > self.reach_time:
                    self.goal_done = True
            else:
                self.reach_time = self.get_clock().now().to_msg().sec + 2

            # Publish the velocity commands to cmd_vel topic
            self.cmd_vel_publisher.publish(cmd_vel_msg)

        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {str(e)}")

    def calculate_pid_linear(self, error):
        # PID control for linear velocity
        # Calculate proportional term
        proportional = self.kp_linear * error

        # Calculate integral term (approximate integral using the cumulative sum)
        self.integral_linear += error
        integral = self.ki_linear * self.integral_linear

        # Calculate derivative term
        derivative = self.kd_linear * (error - self.prev_error_linear)

        # Calculate the control signal
        control_signal = proportional + integral + derivative

        # Update previous error
        self.prev_error_linear = error

        return control_signal

    def calculate_pid_angular(self, error):
        # PID control for angular velocity
        # Calculate proportional term
        proportional = self.kp_angular * error

        # Calculate integral term (approximate integral using the cumulative sum)
        self.integral_angular += error
        integral = self.ki_angular * self.integral_angular

        # Calculate derivative term
        derivative = self.kd_angular * (error - self.prev_error_angular)

        # Calculate the control signal
        control_signal = proportional + integral + derivative

        # Update previous error
        self.prev_error_angular = error

        return control_signal

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
