from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    encoder_node = Node(
        package='encoder_node',
        executable='encoder_driver',
        name='encoder_driver',
        output='screen'
    )

    fusion_node = Node(
        package='encoder_node',
        executable='fusion_node',
        name='fusion_node',
        output='screen'
    )

    return LaunchDescription([
        encoder_node,
        fusion_node
    ])
