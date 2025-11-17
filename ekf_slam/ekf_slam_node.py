#!/usr/bin/env python3
"""
EKF SLAM Node for ROS 2
Implements Extended Kalman Filter based SLAM using laser scan data
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    return q


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler angles"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw


class EKFSLAMNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 200)
        self.declare_parameter('map_height', 200)
        self.declare_parameter('max_laser_range', 3.5)
        self.declare_parameter('min_laser_range', 0.12)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.max_range = self.get_parameter('max_laser_range').value
        self.min_range = self.get_parameter('min_laser_range').value
        
        # Initialize state: [x, y, theta, landmark1_x, landmark1_y, ...]
        self.state = np.array([0.0, 0.0, 0.0])  # Robot pose in map frame [x, y, theta]
        self.covariance = np.eye(3) * 0.1  # Initial uncertainty
        
        # Process and measurement noise
        self.Q = np.diag([0.1, 0.1, 0.05])  # Process noise for motion model
        self.R_landmark = np.diag([0.1, 0.1])  # Measurement noise for landmarks
        
        # Map data
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0
        
        # Previous odometry for motion model
        self.prev_odom = None
        
        # Map frame is aligned with odom frame at startup (no drift correction yet)
        # In a simple implementation without loop closure, map = odom
        # So map->odom transform is identity (0, 0, 0)
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_theta = 0.0
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        self.landmarks_pub = self.create_publisher(PoseArray, '/landmarks', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing map
        self.create_timer(1.0, self.publish_map)
        
        # Debug: log initial state
        self.first_scan = True
        
        self.get_logger().info('EKF SLAM Node initialized')
    
    def odom_callback(self, msg):
        """Process odometry data for motion prediction"""
        current_odom = msg
        
        if self.prev_odom is not None:
            # Get current odometry pose in odom frame
            curr_x = current_odom.pose.pose.position.x
            curr_y = current_odom.pose.pose.position.y
            _, _, curr_yaw = euler_from_quaternion(
                current_odom.pose.pose.orientation.x,
                current_odom.pose.pose.orientation.y,
                current_odom.pose.pose.orientation.z,
                current_odom.pose.pose.orientation.w
            )
            
            # Get previous odometry pose in odom frame
            prev_x = self.prev_odom.pose.pose.position.x
            prev_y = self.prev_odom.pose.pose.position.y
            _, _, prev_yaw = euler_from_quaternion(
                self.prev_odom.pose.pose.orientation.x,
                self.prev_odom.pose.pose.orientation.y,
                self.prev_odom.pose.pose.orientation.z,
                self.prev_odom.pose.pose.orientation.w
            )
            
            # Calculate odometry delta in GLOBAL odom frame
            # These are global displacements, not in robot's local frame
            dx_global = curr_x - prev_x
            dy_global = curr_y - prev_y
            dtheta = curr_yaw - prev_yaw
            
            # Normalize angle
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
            
            # For simple SLAM where map = odom initially:
            # We directly use odom position as our state
            # No coordinate transformation needed!
            self.state[0] = curr_x
            self.state[1] = curr_y
            self.state[2] = curr_yaw
            
            # Update covariance (simplified - just add process noise)
            self.covariance[:3, :3] += self.Q
            
            # Publish estimated pose
            self.publish_pose()
        else:
            # First odometry message: initialize robot state to current odom position
            curr_x = current_odom.pose.pose.position.x
            curr_y = current_odom.pose.pose.position.y
            _, _, curr_yaw = euler_from_quaternion(
                current_odom.pose.pose.orientation.x,
                current_odom.pose.pose.orientation.y,
                current_odom.pose.pose.orientation.z,
                current_odom.pose.pose.orientation.w
            )
            self.state[0] = curr_x
            self.state[1] = curr_y
            self.state[2] = curr_yaw
        
        self.prev_odom = current_odom
    
    def predict(self, dx_odom, dy_odom, dtheta):
        """EKF prediction step based on odometry"""
        # Current state in map frame
        x, y, theta = self.state[0], self.state[1], self.state[2]
        
        # For simple SLAM without loop closure: map frame = odom frame
        # Odometry gives us delta in odom frame (relative to previous robot pose)
        # We need to transform this delta to the current map frame orientation
        # 
        # The odometry delta (dx_odom, dy_odom) is in the robot's LOCAL frame at previous timestep
        # We need to rotate it to the global map frame using PREVIOUS robot orientation
        # NOT current orientation!
        
        # Use PREVIOUS theta (before update) for rotation
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Transform delta from robot frame to map frame
        dx_map = dx_odom * cos_theta - dy_odom * sin_theta
        dy_map = dx_odom * sin_theta + dy_odom * cos_theta
        
        # Motion model: update robot pose in map frame
        self.state[0] += dx_map
        self.state[1] += dy_map
        self.state[2] += dtheta
        
        # Normalize theta
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Jacobian of motion model
        n = len(self.state)
        G = np.eye(n)
        G[0, 2] = -dx_odom * sin_theta - dy_odom * cos_theta
        G[1, 2] = dx_odom * cos_theta - dy_odom * sin_theta
        
        # Update covariance
        self.covariance = G @ self.covariance @ G.T
        self.covariance[:3, :3] += self.Q
        
        # Publish estimated pose
        self.publish_pose()
    
    def scan_callback(self, msg):
        """Process laser scan data for mapping and landmark detection"""
        if len(self.state) < 3:
            return
        
        robot_x = self.state[0]
        robot_y = self.state[1]
        robot_theta = self.state[2]
        
        # Debug: log on first scan
        if self.first_scan:
            self.get_logger().info(f'First scan - Robot pose: x={robot_x:.3f}, y={robot_y:.3f}, theta={robot_theta:.3f} rad ({np.degrees(robot_theta):.1f} deg)')
            self.get_logger().info(f'Scan angle range: {msg.angle_min:.3f} to {msg.angle_max:.3f} rad')
            
            # Find the closest obstacle in front (angle ≈ 0)
            front_idx = 0
            if msg.ranges[front_idx] < self.max_range:
                self.get_logger().info(f'Front obstacle distance: {msg.ranges[front_idx]:.3f}m at angle 0 rad')
            
            # Find closest obstacle to the left (angle ≈ π/2)
            left_idx = int(len(msg.ranges) * 0.25)
            if left_idx < len(msg.ranges) and msg.ranges[left_idx] < self.max_range:
                angle_left = msg.angle_min + left_idx * msg.angle_increment
                self.get_logger().info(f'Left obstacle distance: {msg.ranges[left_idx]:.3f}m at angle {angle_left:.3f} rad ({np.degrees(angle_left):.1f} deg)')
            
            self.first_scan = False
        
        # Laser scan is in base_scan frame which follows ROS REP-105
        # X-axis points forward, Y-axis points left, Z-axis points up
        # angle_min = 0 typically points in the +X direction (forward)
        # Angles increase counter-clockwise (towards +Y / left side)
        
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            # Filter invalid ranges
            if r < self.min_range or r > self.max_range or not np.isfinite(r):
                angle += msg.angle_increment
                continue
            
            # Point in base_scan frame (laser frame)
            # ROS uses right-hand rule: X forward, Y left, Z up
            # angle=0 is forward (+X), angle=π/2 is left (+Y)
            scan_x = r * np.cos(angle)
            scan_y = r * np.sin(angle)
            
            # Transform to map frame using 2D rotation matrix
            # map frame also follows ROS convention: X forward (east), Y left (north)
            cos_theta = np.cos(robot_theta)
            sin_theta = np.sin(robot_theta)
            
            # 2D rotation: [x'] = [cos -sin] [x]
            #              [y']   [sin  cos] [y]
            global_x = robot_x + (scan_x * cos_theta - scan_y * sin_theta)
            global_y = robot_y + (scan_x * sin_theta + scan_y * cos_theta)
            
            # Update occupancy grid
            self.update_map(global_x, global_y, occupied=True)
            
            # Raytracing for free space (simplified)
            self.raytrace(robot_x, robot_y, global_x, global_y)
            
            angle += msg.angle_increment
    
    def update_map(self, x, y, occupied=True):
        """Update occupancy grid map"""
        # Convert to grid coordinates
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Check bounds
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            if occupied:
                self.map_data[grid_y, grid_x] = min(100, self.map_data[grid_y, grid_x] + 10)
            else:
                self.map_data[grid_y, grid_x] = max(0, self.map_data[grid_y, grid_x] - 5)
    
    def raytrace(self, x0, y0, x1, y1):
        """Simple Bresenham raytracing for free space"""
        # Convert to grid coordinates
        gx0 = int((x0 - self.map_origin_x) / self.map_resolution)
        gy0 = int((y0 - self.map_origin_y) / self.map_resolution)
        gx1 = int((x1 - self.map_origin_x) / self.map_resolution)
        gy1 = int((y1 - self.map_origin_y) / self.map_resolution)
        
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        
        x, y = gx0, gy0
        step = 0
        max_steps = 200
        
        while step < max_steps:
            if x == gx1 and y == gy1:
                break
            
            # Mark as free
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.map_data[y, x] = max(-1, self.map_data[y, x] - 2)
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            
            step += 1
    
    def publish_map(self):
        """Publish occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(map_msg)
    
    def publish_pose(self):
        """Publish estimated robot pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.state[0]
        pose_msg.pose.position.y = self.state[1]
        pose_msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        quat = quaternion_from_euler(0, 0, self.state[2])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
        
        # Publish TF: map -> odom
        # For simple SLAM without loop closure detection, map and odom are aligned
        # So the transform is identity (no rotation, no translation)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Identity transform (map = odom)
        t.transform.translation.x = self.map_to_odom_x
        t.transform.translation.y = self.map_to_odom_y
        t.transform.translation.z = 0.0
        
        quat_tf = quaternion_from_euler(0, 0, self.map_to_odom_theta)
        t.transform.rotation.x = quat_tf[0]
        t.transform.rotation.y = quat_tf[1]
        t.transform.rotation.z = quat_tf[2]
        t.transform.rotation.w = quat_tf[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
