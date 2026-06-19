from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oit_navigation',
            executable='feedback_controller',
            name='feedback_controller',
            output='screen',
            parameters=[
                {
                    'raw_cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/sub_odom',
                    'feedback_cmd_vel_topic': '/aiformula_control/twist_mux/cmd_vel',
                    
                    'wheel_radius': 0.13,
                    'tread': 0.7,
                    'omega_wheel_max': 90.0,
                    'mu': 0.7,
                    'v_min':0.05,
                    
                    'Kp_v_PI': 0.6,
                    'Ki_v_PI': 0.0,
                    'Kp_v_PD': 0.6,
                    'Kd_v_PD': 0.0,
                    
                    'Kp_w_PI': 2.5,
                    'Ki_w_PI': 0.1,
                    'Kp_w_PD': 2.5,
                    'Kd_w_PD': 0.0,
                    
                    'omega_low': 0.4,
                    'omega_high': 0.8,
                    'cycle_time': 0.01,
                    'integrator_limit': 1.0,
                    'derivative_alpha': 0.2
                }
            ]
        )
    ])