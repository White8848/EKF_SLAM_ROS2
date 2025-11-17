import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'ekf_slam'
    pkg_share = get_package_share_directory(pkg_name)
    
    # RViz configuration file path
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    
    # Use default config if custom doesn't exist
    use_rviz_config = LaunchConfiguration('rviz_config')
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='Full path to the RViz config file to use'
    )

    # RViz2 node with simulation time
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', use_rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_rviz_config_cmd,
        rviz_node,
    ])
