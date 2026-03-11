from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("od2_navigation"), "maps", "c5_2f_pc_room.yaml"]),
        description="Full path to the yaml map file",
    )

    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("od2_navigation"), "config", "amcl.config.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params_file, {"use_sim_time": use_sim_time}],
    )

    nav_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav_manager",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )
    
    sllidar_s1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_s1_launch.py')
        )
    )

    sllidar_scan_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'scan_filter.launch.py')
        )
    )

    od2_controller_node = Node(
        package='od2_controller_ros2',
        executable='od2_controller_ros2',
        name='od2_controller_ros2',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            amcl_params_file_arg,
            map_file_arg,
            map_server_node,
            amcl_node,
            nav_manager,
            sllidar_s1_launch,
            sllidar_scan_filter_launch,
            od2_controller_node
        ]
    )