import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'ekf_slam'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for Nav2 nodes'
    )
    
    # Include Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        nav2_bringup_launch,
    ])
