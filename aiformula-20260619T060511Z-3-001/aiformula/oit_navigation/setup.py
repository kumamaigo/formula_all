from setuptools import setup

package_name = 'oit_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pointcloud_follower.launch.py']),
        ('share/' + package_name + '/launch', ['launch/run.launch.py']),
        ('share/' + package_name + '/launch', ['launch/feedback_controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/pid_feedback_controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/oit_nodes.launch.py']),
        ('share/' + package_name + '/launch', ['launch/oit_nodes_sim.launch.py']),
        ('share/' + package_name + '/launch', ['launch/twist_mux.launch.py']),
        ('share/' + package_name + '/config', ['config/twist_mux.yaml']),
        ('share/' + package_name + '/launch', ['launch/object_road_detector_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_follower = oit_navigation.pointcloud_follower:main',
            'run = oit_navigation.run:main',
            'cmd_remap = oit_navigation.cmd_remap:main',
            'encoder_node = oit_navigation.encoder_node:main',
            'pi_feedback_controller = oit_navigation.pi_feedback_controller:main',
            'pd_feedback_controller = oit_navigation.pd_feedback_controller:main',
            'feedback_controller = oit_navigation.feedback_controller:main',
            'pid_feedback_controller = oit_navigation.pid_feedback_controller:main',
            'brake_test = oit_navigation.brake_test:main',
            'straight_test = oit_navigation.straight_test:main',
            'curve_test = oit_navigation.curve_test:main',
            'save_multi_odom = oit_navigation.save_multi_odom:main',
            'save_odom = oit_navigation.save_odom:main',
            'pid_eval_logger = oit_navigation.pid_eval_logger:main',
            'save_images = oit_navigation.save_images:main',
            'multi_line_follower = oit_navigation.multi_line_follower:main',
            'obstacle_avoider = oit_navigation.obstacle_avoider:main',
            'accel_straight = oit_navigation.accel_straight:main'
        ],
    },
)
