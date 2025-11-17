import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'ekf_slam'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # EKF SLAM node
    ekf_slam_node = Node(
        package='ekf_slam',
        executable='ekf_slam_node',
        name='ekf_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_resolution': 0.05,
            'map_width': 400,
            'map_height': 400,
            'max_laser_range': 3.5,
            'min_laser_range': 0.12,
            'scan_topic': '/scan',
            'odom_topic': '/odom',
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        ekf_slam_node,
    ])
