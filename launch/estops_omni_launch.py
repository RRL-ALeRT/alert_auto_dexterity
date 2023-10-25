import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


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

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gologpp_agent',
                    executable='gologpp_agent',
                    output='screen',
                    parameters=[
                        {'gpp_code': os.path.join(get_package_share_directory("alert_auto_dexterity"), "gpp_agents", "estops_omni.gpp")}
                    ],
                )
            ]
        ),
    ])