from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    multi_line_follower = Node(
        package='oit_navigation',
        executable='multi_line_follower',
        name='multi_line_follower',
        output='screen',
        parameters=[
            {'linear_gain': 0.4},
            {'angular_gain': 0.7},
            {'max_linear_speed': 2.0},
            {'max_angular_speed': 0.4},
            {'lookahead_min_x': 0.0},
            {'lookahead_max_x': 10.0},
            {'far_weight_gain': 1.5},
            {'fallback_speed': 0.5},
            {'data_timeout': 10.0}
        ]
    )
    
    obstacle_avoider = Node(
        package='oit_navigation',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        output='screen',
        parameter=[
            {'max_angular_speed': 0.5}
        ]
    )
    
    return LaunchDescription([
        multi_line_follower,
        obstacle_avoider,
    ])