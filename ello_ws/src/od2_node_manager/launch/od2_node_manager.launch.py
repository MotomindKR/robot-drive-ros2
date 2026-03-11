from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mapname = LaunchConfiguration('mapname')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mapname',
            default_value='test01.json',
            description='Map configuration file name'
        ),

        Node(
            package='od2_node_manager',  
            executable='od2_node_manager', 
            name='od2_node_manager_node',
            output='screen',
            parameters=[{'mapname': mapname}]        
        ),
        Node(
            package='od2_node_manager',  
            executable='od2_pose_publisher', 
            name='od2_pose_publisher_node',
            output='screen'
        ),
        Node(
            package='od2_node_manager',  
            executable='od2_edge_follower', 
            name='od2_edge_follower_node',
            output='screen',
            parameters=[{'mapname': mapname}]        
        ),
        Node(
            package='od2_node_manager',  
            executable='od2_virtual_bumper', 
            name='od2_virtual_bumper_node',
            output='screen'
        )
    ])
