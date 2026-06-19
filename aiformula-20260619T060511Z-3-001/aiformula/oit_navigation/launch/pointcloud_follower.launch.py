import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='oit_navigation',
            executable='pointcloud_follower',
            name='pointcloud_follower_node',
            output='screen',
            parameters=[
                # トピック名のオーバーライド
                {'pointcloud_topic': '/pointcloud'},
                {'cmd_vel_topic': '/cmd_vel'},
                # ゲインの設定
                {'linear_gain': 0.5},
                {'angular_gain': 1.0},
                # 速度上限
                {'max_linear_speed': 0.5},
                {'max_angular_speed': 1.0},
            ],
            remappings=[
                ('/pointcloud', '/aiformula_perception/lane_line_publisher/lane_lines/center'),
                ('/cmd_vel', '/aiformula_control/twist_mux/cmd_vel'),
            ]
        ),
    ])
