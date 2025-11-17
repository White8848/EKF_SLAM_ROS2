import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'ekf_slam'
    pkg_share = get_package_share_directory(pkg_name)

    # Set the path to the default world file
    default_world = os.path.join(
        pkg_share, 'worlds', 'bookstore', 'bookstore.world'
    )

    # Ensure Gazebo Classic can find models in our package
    bookstore_dir = os.path.join(pkg_share, 'worlds', 'bookstore')
    models_dir = os.path.join(bookstore_dir, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')

    combined_model_path = (
        f"{models_dir}:{existing_model_path}"
        if existing_model_path else models_dir
    )
    # Use the bookstore folder as a resource root so 'file://models/...' resolves
    combined_resource_path = (
        f"{bookstore_dir}:{existing_resource_path}"
        if existing_resource_path else bookstore_dir
    )

    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the world file'
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=combined_model_path
    )
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', value=combined_resource_path
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={'world': world}.items(),
    )

    # Static TF publishers for robot frames
    # TF tree: odom -> base_footprint -> base_link -> base_scan/imu_link/camera_*
    
    # Static transform: base_footprint -> base_link (from base_joint in SDF)
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_broadcaster',
        arguments=['0', '0', '0.010', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    # Static transform: base_link -> base_scan (from lidar_joint in SDF)
    base_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_scan_broadcaster',
        arguments=['-0.064', '0', '0.121', '0', '0', '0', 'base_link', 'base_scan']
    )
    
    # Static transform: base_link -> imu_link
    imu_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    # Static transform: base_link -> camera_link
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_broadcaster',
        arguments=['0.073', '-0.011', '0.084', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # Static transform: camera_link -> camera_rgb_frame
    camera_rgb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_rgb_broadcaster',
        arguments=['0.003', '0.011', '0.009', '0', '0', '0', 'camera_link', 'camera_rgb_frame']
    )

    return LaunchDescription([
        declare_world_cmd,
        set_gazebo_model_path,
        set_gazebo_resource_path,
        gazebo_launch,
        base_link_tf,
        base_scan_tf,
        imu_link_tf,
        camera_tf,
        camera_rgb_tf,
    ])
