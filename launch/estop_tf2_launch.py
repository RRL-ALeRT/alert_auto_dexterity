from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='spot_target_transform',
            output='screen',
            arguments=['0.3', '0', '0.85', '0', '1.57', '-1.57', 'estop_set', 'spot_target']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool_target_transform',
            output='screen',
            arguments=['0', '0', '0', '-0.785', '3.14', '0', 'estop_set', 'tool_target']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool_estop_target_transform',
            output='screen',
            arguments=['0', '0', '0', '0', '3.14', '0', 'estop', 'tool_estop_target']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool_estop_target_0_1_transform',
            output='screen',
            arguments=['0', '0', '-0.1', '0', '0', '0', 'tool_estop_target', 'tool_estop_target_0_1']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool_estop_target_0_05_transform',
            output='screen',
            arguments=['0', '0', '-0.05', '0', '0', '0', 'tool_estop_target', 'tool_estop_target_0_05']
        ),
    ])
