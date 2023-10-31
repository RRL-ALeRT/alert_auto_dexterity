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
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='stand_in_front.py',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='see_estops_with_manipulator.py',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='goto_estops_with_manipulator.py',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='find_object_pose.py',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='depth_to_pointcloud2',
            namespace='kinova',
            output='screen',
            remappings=[
                ('/kinova/camera_info','/kinova_depth/camera_info'),
                ('/kinova/image_rect','/kinova_depth'),
            ],
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            namespace='kinova',
            output='screen',
            parameters=[{
                "resolution": 0.05,
                "frame_id": "map",
                "base_frame_id": "camera_depth_frame",
                "sensor_model.max_range": 2.0,
                "latch": False,
                "filter_speckles": True,
            }],
            remappings=[
                ("/kinova/cloud_in", "/kinova/points"),
            ],
        ),
        Node(
            package='alert_auto_dexterity',
            executable='clear_box_octomap.py',
            output='screen',
        ),
        Node(
            package='alert_auto_dexterity',
            executable='press_estop.py',
            output='screen',
        ),

        TimerAction(
            period=3.0,
            actions=[
                # Node(
                #     package='gologpp_agent',
                #     executable='gologpp_agent',
                #     output='screen',
                #     parameters=[
                #         {'gpp_code': os.path.join(get_package_share_directory("alert_auto_dexterity"), "gpp_agents", "estops_omni.gpp")}
                #     ],
                # )
                Node(
                    package='alert_auto_dexterity',
                    executable='execute_tree',
                    output='screen',
                )
            ]
        ),
    ])
