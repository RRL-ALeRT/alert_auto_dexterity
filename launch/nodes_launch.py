from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alert_auto_dexterity',
            executable='find_estops.py',
            name='find_estops',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='stand_in_front.py',
            name='stand_in_front',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='see_estops_with_manipulator.py',
            name='see_estops_with_manipulator',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='goto_estops_with_manipulator.py',
            name='goto_estops_with_manipulator',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='find_single_estop.py',
            name='find_single_estop',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='press_estop.py',
            name='press_estop',
            output='screen',
        ),
    ])
