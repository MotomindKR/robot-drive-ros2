from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_params_file = os.path.join(get_package_share_directory('od2_navigation'), 'config', 'amcl.config.yaml')
    map_file_name = LaunchConfiguration('map_file_name')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_map_file_name_argument = DeclareLaunchArgument(
        'map_file_name',
        default_value='home_serialized',
        description='Serialized map file name (without extension)'
    )


    start_sync_slam_toolbox_node = Node(
        parameters=[
            os.path.join(get_package_share_directory("od2_navigation"), 'config', 'mapper_params_localization.yaml'),
            {
                'use_sim_time': use_sim_time,
                'map_file_name': PathJoinSubstitution([
                    get_package_share_directory("od2_navigation"),
                    'maps',
                    map_file_name
                ])
            }
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("od2_navigation"), 'config', 'amcl.config.yaml'),
            {"use_sim_time": use_sim_time}
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_map_file_name_argument)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(amcl_node)

    return ld
