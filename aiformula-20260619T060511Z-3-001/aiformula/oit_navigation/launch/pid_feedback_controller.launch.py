from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'oit_navigation',
            executable = 'pid_feedback_controller',
            name = 'pid_feedback_controller',
            output = 'screen',
            parameters = [
                {
                    'log_path': os.path.join(
                        os.getenv('HOME'),
                        'workspace/ros2_ws/src/aiformula/oit_navigation/log/feedback_log.csv'
                    ),
                    'Kp_v_st':0.55,
                    'Ki_v_st':0.04,
                    'Kd_v_st':0.05,
                    'Kp_w_st':1.6,
                    'Ki_w_st':0.03,
                    'Kd_w_st':0.45,
                    'Kp_v_turn':0.55,
                    'Ki_v_turn':0.02,
                    'Kd_v_turn':0.06,
                    'Kp_w_turn':1.8,
                    'Ki_w_turn':0.015,
                    'Kd_w_turn':0.6,
                    
                    'integrator_limit':0.1,
                    'derivative_alpha':0.7,
                    
                    'omega_low':0.05,
                    'omega_high':0.12
                    
                }
            ]
        )
    ])