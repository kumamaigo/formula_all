from setuptools import setup

package_name = 'light_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aiformula',
    maintainer_email='masima458@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'focal_measure = light_panel.focal_measure:main',
            'distance_estimation_pub_node = light_panel.distance_estimation_pub_node:main',
            'distance_estimation_sub_node = light_panel.distance_estimation_sub_node:main'
        ],
    },
)
