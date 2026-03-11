from setuptools import setup
import os
from glob import glob

package_name = 'od2_node_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyeum',
    maintainer_email='giyeong626@motomind.co.kr',
    description='od2_node_manager',
    license='MIT License',
    entry_points={
        'console_scripts': [
            'od2_node_manager = od2_node_manager.od2_node_manager:main',
            'od2_node_goal_sender = od2_node_manager.od2_node_goal_sender:main',
            'od2_node_current_pose_sender = od2_node_manager.od2_node_current_pose_sender:main',
            'od2_node_editor = od2_node_manager.od2_node_editor:main',
            'od2_pose_publisher = od2_node_manager.od2_pose_publisher:main',
            'od2_edge_follower = od2_node_manager.od2_edge_follower:main',
            'od2_virtual_bumper = od2_node_manager.od2_virtual_bumper:main',
        ],
    },
)
