import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the clustering-based EKF SLAM node."""
    
    return LaunchDescription([
        Node(
            package='ekf_slam',
            executable='ekf_slam_clustering_node',
            name='ekf_slam_clustering_node',
            output='screen',
            parameters=[{
                'map_resolution': 0.05,
                'map_width': 400,
                'map_height': 400,
                'max_laser_range': 3.5,
                'min_laser_range': 0.12,
            }]
        ),
    ])
