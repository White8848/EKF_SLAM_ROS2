#!/usr/bin/env python3
"""
EKF SLAM Node for ROS 2
Implements Feature-based Extended Kalman Filter SLAM using laser scan data.

State Vector: [x, y, θ, lx₁, ly₁, lx₂, ly₂, ..., lxₙ, lyₙ]
  - x, y, θ: Robot pose in map frame
  - lxᵢ, lyᵢ: i-th landmark position in map frame

Key Components:
  1. Prediction: Update robot pose using odometry, propagate uncertainty
  2. Feature Extraction: Cluster laser points to detect point landmarks
  3. Data Association: Match observations to known landmarks (Mahalanobis distance)
  4. EKF Update: Correct state using observation residuals (Kalman Gain)
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


# =============================================================================
# Utility Functions
# =============================================================================

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion [x, y, z, w]."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy   # w
    ]


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler angles, returns (roll, pitch, yaw)."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw


def normalize_angle(angle):
    """Normalize angle to [-π, π]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


# =============================================================================
# EKF SLAM Node
# =============================================================================

class EKFSLAMNode(Node):
    """
    Feature-based EKF SLAM node.
    
    Subscribes to /scan (LaserScan) and /odom (Odometry).
    Publishes /map (OccupancyGrid), /ekf_pose (PoseStamped), /landmarks (PoseArray).
    """
    
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 200)
        self.declare_parameter('map_height', 200)
        self.declare_parameter('max_laser_range', 3.5)
        self.declare_parameter('min_laser_range', 0.12)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.max_range = self.get_parameter('max_laser_range').value
        self.min_range = self.get_parameter('min_laser_range').value
        
        # ---------------------------------------------------------------------
        # EKF State: [x, y, θ, lx₁, ly₁, ..., lxₙ, lyₙ]
        # ---------------------------------------------------------------------
        self.state = np.array([0.0, 0.0, 0.0])  # Initial robot pose
        self.covariance = np.eye(3) * 0.01      # Initial covariance (small uncertainty)
        self.num_landmarks = 0                   # Number of landmarks in state
        
        # Noise parameters
        self.Q = np.diag([0.02, 0.02, 0.01])    # Process noise (motion model)
        self.R = np.diag([0.1, 0.05])           # Measurement noise [range, bearing]
        
        # Data association threshold (Mahalanobis distance squared)
        self.association_threshold = 5.0  # χ² with 2 DOF, 95% confidence ≈ 5.99
        
        # Landmark extraction parameters
        self.cluster_threshold = 0.3     # Max distance between points in a cluster (m)
        self.min_cluster_size = 3        # Minimum points to form a landmark
        self.max_cluster_size = 20       # Maximum points (filter out walls)
        
        # ---------------------------------------------------------------------
        # Occupancy Grid Map
        # ---------------------------------------------------------------------
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0
        
        # Previous odometry for computing delta
        self.prev_odom = None
        self.initialized = False
        
        # ---------------------------------------------------------------------
        # ROS 2 Interfaces
        # ---------------------------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback, 10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback, 10
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map',
            rclpy.qos.QoSProfile(
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        self.landmarks_pub = self.create_publisher(PoseArray, '/landmarks', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publish map periodically
        self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('EKF SLAM Node initialized (Feature-based)')
    
    # =========================================================================
    # EKF Prediction Step
    # =========================================================================
    
    def odom_callback(self, msg):
        """
        Process odometry for EKF prediction step.
        Computes motion delta and updates robot pose with uncertainty propagation.
        """
        if self.prev_odom is None:
            # First odometry: initialize robot pose
            self.prev_odom = msg
            _, _, yaw = euler_from_quaternion(
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            )
            self.state[0] = msg.pose.pose.position.x
            self.state[1] = msg.pose.pose.position.y
            self.state[2] = yaw
            self.initialized = True
            return
        
        # Extract current and previous poses
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        _, _, curr_yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )
        
        prev_x = self.prev_odom.pose.pose.position.x
        prev_y = self.prev_odom.pose.pose.position.y
        _, _, prev_yaw = euler_from_quaternion(
            self.prev_odom.pose.pose.orientation.x, self.prev_odom.pose.pose.orientation.y,
            self.prev_odom.pose.pose.orientation.z, self.prev_odom.pose.pose.orientation.w
        )
        
        # Compute odometry delta in robot's local frame
        dx_global = curr_x - prev_x
        dy_global = curr_y - prev_y
        dtheta = normalize_angle(curr_yaw - prev_yaw)
        
        # Transform global delta to robot's local frame
        cos_prev = np.cos(prev_yaw)
        sin_prev = np.sin(prev_yaw)
        dx_local = dx_global * cos_prev + dy_global * sin_prev
        dy_local = -dx_global * sin_prev + dy_global * cos_prev
        
        # Run EKF prediction with local motion
        self.predict(dx_local, dy_local, dtheta)
        
        self.prev_odom = msg
        self.publish_pose()
    
    def predict(self, dx_local, dy_local, dtheta):
        """
        EKF Prediction Step.
        
        Motion model: robot moves by (dx, dy) in its local frame, then rotates by dθ.
        Updates state mean and covariance.
        """
        theta = self.state[2]
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        # Transform local motion to global frame
        dx_global = dx_local * cos_t - dy_local * sin_t
        dy_global = dx_local * sin_t + dy_local * cos_t
        
        # Update robot pose
        self.state[0] += dx_global
        self.state[1] += dy_global
        self.state[2] = normalize_angle(self.state[2] + dtheta)
        
        # Jacobian of motion model w.r.t. state (only robot pose part affects motion)
        n = len(self.state)
        G = np.eye(n)
        G[0, 2] = -dx_local * sin_t - dy_local * cos_t
        G[1, 2] = dx_local * cos_t - dy_local * sin_t
        
        # Propagate covariance: P = G P Gᵀ + Q_augmented
        self.covariance = G @ self.covariance @ G.T
        self.covariance[:3, :3] += self.Q  # Add process noise to robot pose only
    
    # =========================================================================
    # Feature Extraction
    # =========================================================================
    
    def scan_callback(self, msg):
        """
        Process laser scan: extract landmarks, perform data association and EKF update.
        Also updates occupancy grid for visualization.
        """
        if not self.initialized:
            return
        
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        # ---------------------------------------------------------------------
        # 1. Update occupancy grid (for visualization)
        # ---------------------------------------------------------------------
        self.update_occupancy_grid(msg, robot_x, robot_y, robot_theta)
        
        # ---------------------------------------------------------------------
        # 2. Extract point landmarks from scan
        # ---------------------------------------------------------------------
        landmarks_polar = self.extract_landmarks(msg)  # List of (range, bearing)
        
        # ---------------------------------------------------------------------
        # 3. Data association and EKF update for each observed landmark
        # ---------------------------------------------------------------------
        for z_range, z_bearing in landmarks_polar:
            landmark_idx = self.associate_landmark(z_range, z_bearing)
            
            if landmark_idx >= 0:
                # Known landmark: perform EKF update
                self.ekf_update(landmark_idx, z_range, z_bearing)
            else:
                # New landmark: add to state
                self.add_landmark(z_range, z_bearing)
        
        # Publish landmarks for visualization
        self.publish_landmarks()
    
    def extract_landmarks(self, msg):
        """
        Extract point landmarks from laser scan using clustering.
        
        Groups consecutive scan points into clusters. Each cluster's centroid
        is considered a potential point landmark (e.g., pillars, boxes).
        
        Returns: List of (range, bearing) in robot frame
        """
        landmarks = []
        
        # Build list of valid points in polar coordinates
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_range < r < self.max_range and np.isfinite(r):
                points.append((r, angle))
            angle += msg.angle_increment
        
        if len(points) < self.min_cluster_size:
            return landmarks
        
        # Cluster consecutive points by distance
        clusters = []
        current_cluster = [points[0]]
        
        for i in range(1, len(points)):
            r1, a1 = points[i - 1]
            r2, a2 = points[i]
            
            # Convert to Cartesian for distance calculation
            x1, y1 = r1 * np.cos(a1), r1 * np.sin(a1)
            x2, y2 = r2 * np.cos(a2), r2 * np.sin(a2)
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            if dist < self.cluster_threshold:
                current_cluster.append(points[i])
            else:
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(current_cluster)
                current_cluster = [points[i]]
        
        # Don't forget the last cluster
        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)
        
        # For each valid cluster, compute centroid as landmark
        for cluster in clusters:
            if len(cluster) > self.max_cluster_size:
                continue  # Skip large clusters (likely walls)
            
            # Compute centroid in Cartesian
            xs = [r * np.cos(a) for r, a in cluster]
            ys = [r * np.sin(a) for r, a in cluster]
            cx, cy = np.mean(xs), np.mean(ys)
            
            # Convert back to polar
            lm_range = np.sqrt(cx**2 + cy**2)
            lm_bearing = np.arctan2(cy, cx)
            landmarks.append((lm_range, lm_bearing))
        
        return landmarks
    
    # =========================================================================
    # Data Association
    # =========================================================================
    
    def associate_landmark(self, z_range, z_bearing):
        """
        Associate observation to known landmark using Mahalanobis distance.
        
        Returns: Landmark index (0-based) if matched, -1 if new landmark.
        """
        if self.num_landmarks == 0:
            return -1
        
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        best_idx = -1
        best_dist = self.association_threshold
        
        for i in range(self.num_landmarks):
            # Get landmark position from state
            lx = self.state[3 + 2 * i]
            ly = self.state[3 + 2 * i + 1]
            
            # Predicted observation
            dx = lx - robot_x
            dy = ly - robot_y
            pred_range = np.sqrt(dx**2 + dy**2)
            pred_bearing = normalize_angle(np.arctan2(dy, dx) - robot_theta)
            
            # Innovation (observation - prediction)
            innovation = np.array([
                z_range - pred_range,
                normalize_angle(z_bearing - pred_bearing)
            ])
            
            # Compute Jacobian H for this landmark
            H = self._compute_jacobian_H(i, dx, dy, pred_range)
            
            # Innovation covariance: S = H P Hᵀ + R
            S = H @ self.covariance @ H.T + self.R
            
            # Mahalanobis distance: d² = yᵀ S⁻¹ y
            try:
                S_inv = np.linalg.inv(S)
                mahal_dist = innovation.T @ S_inv @ innovation
            except np.linalg.LinAlgError:
                continue
            
            if mahal_dist < best_dist:
                best_dist = mahal_dist
                best_idx = i
        
        return best_idx
    
    # =========================================================================
    # EKF Update Step
    # =========================================================================
    
    def ekf_update(self, landmark_idx, z_range, z_bearing):
        """
        EKF Update Step using observation of a known landmark.
        
        Corrects robot pose and landmark positions using Kalman gain.
        """
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        # Get landmark position
        lx = self.state[3 + 2 * landmark_idx]
        ly = self.state[3 + 2 * landmark_idx + 1]
        
        # Predicted observation
        dx = lx - robot_x
        dy = ly - robot_y
        pred_range = np.sqrt(dx**2 + dy**2)
        pred_bearing = normalize_angle(np.arctan2(dy, dx) - robot_theta)
        
        # Innovation
        innovation = np.array([
            z_range - pred_range,
            normalize_angle(z_bearing - pred_bearing)
        ])
        
        # Jacobian H (2 x n matrix)
        H = self._compute_jacobian_H(landmark_idx, dx, dy, pred_range)
        
        # Innovation covariance: S = H P Hᵀ + R
        S = H @ self.covariance @ H.T + self.R
        
        # Kalman Gain: K = P Hᵀ S⁻¹
        try:
            K = self.covariance @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Singular matrix in EKF update, skipping')
            return
        
        # State update: x = x + K * innovation
        self.state = self.state + K @ innovation
        self.state[2] = normalize_angle(self.state[2])
        
        # Covariance update: P = (I - K H) P
        n = len(self.state)
        I = np.eye(n)
        self.covariance = (I - K @ H) @ self.covariance
        
        # Ensure symmetry
        self.covariance = (self.covariance + self.covariance.T) / 2
    
    def _compute_jacobian_H(self, landmark_idx, dx, dy, q):
        """
        Compute observation Jacobian H for a landmark.
        
        H = ∂h/∂state where h(state) = [range, bearing]
        
        Args:
            landmark_idx: Index of the landmark (0-based)
            dx, dy: Difference between landmark and robot position
            q: Predicted range (sqrt(dx² + dy²))
        """
        n = len(self.state)
        H = np.zeros((2, n))
        
        if q < 1e-6:
            return H  # Avoid division by zero
        
        q2 = q * q
        
        # Partial derivatives w.r.t. robot pose [x, y, θ]
        H[0, 0] = -dx / q       # ∂range/∂x
        H[0, 1] = -dy / q       # ∂range/∂y
        H[0, 2] = 0             # ∂range/∂θ
        
        H[1, 0] = dy / q2       # ∂bearing/∂x
        H[1, 1] = -dx / q2      # ∂bearing/∂y
        H[1, 2] = -1            # ∂bearing/∂θ
        
        # Partial derivatives w.r.t. landmark [lx, ly]
        lm_start = 3 + 2 * landmark_idx
        H[0, lm_start] = dx / q       # ∂range/∂lx
        H[0, lm_start + 1] = dy / q   # ∂range/∂ly
        
        H[1, lm_start] = -dy / q2     # ∂bearing/∂lx
        H[1, lm_start + 1] = dx / q2  # ∂bearing/∂ly
        
        return H
    
    # =========================================================================
    # State Augmentation
    # =========================================================================
    
    def add_landmark(self, z_range, z_bearing):
        """
        Add a new landmark to the state vector.
        
        Computes landmark global position from observation and expands covariance.
        """
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        # Compute landmark global position
        global_angle = robot_theta + z_bearing
        lx = robot_x + z_range * np.cos(global_angle)
        ly = robot_y + z_range * np.sin(global_angle)
        
        # Augment state vector
        self.state = np.append(self.state, [lx, ly])
        
        # Augment covariance matrix
        # New landmark's initial uncertainty depends on robot pose uncertainty and measurement noise
        n_old = len(self.covariance)
        n_new = n_old + 2
        
        # Jacobian of landmark initialization w.r.t. robot pose
        # lx = rx + r*cos(θ + β), ly = ry + r*sin(θ + β)
        cos_a = np.cos(global_angle)
        sin_a = np.sin(global_angle)
        
        # Gp: Jacobian w.r.t. robot pose [x, y, θ]
        Gp = np.array([
            [1, 0, -z_range * sin_a],
            [0, 1, z_range * cos_a]
        ])
        
        # Gz: Jacobian w.r.t. measurement [range, bearing]
        Gz = np.array([
            [cos_a, -z_range * sin_a],
            [sin_a, z_range * cos_a]
        ])
        
        # New covariance matrix
        P_new = np.zeros((n_new, n_new))
        P_new[:n_old, :n_old] = self.covariance
        
        # Covariance of new landmark
        P_robot = self.covariance[:3, :3]
        P_lm = Gp @ P_robot @ Gp.T + Gz @ self.R @ Gz.T
        P_new[n_old:, n_old:] = P_lm
        
        # Cross-covariance between new landmark and existing state
        P_new[n_old:, :n_old] = Gp @ self.covariance[:3, :]
        P_new[:n_old, n_old:] = P_new[n_old:, :n_old].T
        
        self.covariance = P_new
        self.num_landmarks += 1
        
        self.get_logger().info(
            f'Added landmark #{self.num_landmarks} at ({lx:.2f}, {ly:.2f})'
        )
    
    # =========================================================================
    # Occupancy Grid Update (for visualization)
    # =========================================================================
    
    def update_occupancy_grid(self, msg, robot_x, robot_y, robot_theta):
        """Update occupancy grid from laser scan."""
        angle = msg.angle_min
        cos_t, sin_t = np.cos(robot_theta), np.sin(robot_theta)
        
        for r in msg.ranges:
            if not (self.min_range < r < self.max_range and np.isfinite(r)):
                angle += msg.angle_increment
                continue
            
            # Point in robot frame -> map frame
            scan_x = r * np.cos(angle)
            scan_y = r * np.sin(angle)
            global_x = robot_x + scan_x * cos_t - scan_y * sin_t
            global_y = robot_y + scan_x * sin_t + scan_y * cos_t
            
            # Mark as occupied
            self._update_cell(global_x, global_y, occupied=True)
            
            # Raytrace free space
            self._raytrace_free(robot_x, robot_y, global_x, global_y)
            
            angle += msg.angle_increment
    
    def _update_cell(self, x, y, occupied=True):
        """Update a single cell in the occupancy grid."""
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            if occupied:
                self.map_data[gy, gx] = min(100, self.map_data[gy, gx] + 10)
            else:
                self.map_data[gy, gx] = max(0, self.map_data[gy, gx] - 5)
    
    def _raytrace_free(self, x0, y0, x1, y1):
        """Bresenham raytracing for free space."""
        gx0 = int((x0 - self.map_origin_x) / self.map_resolution)
        gy0 = int((y0 - self.map_origin_y) / self.map_resolution)
        gx1 = int((x1 - self.map_origin_x) / self.map_resolution)
        gy1 = int((y1 - self.map_origin_y) / self.map_resolution)
        
        dx, dy = abs(gx1 - gx0), abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        
        x, y = gx0, gy0
        for _ in range(200):
            if x == gx1 and y == gy1:
                break
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.map_data[y, x] = max(-1, self.map_data[y, x] - 2)
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    # =========================================================================
    # Publishers
    # =========================================================================
    
    def publish_map(self):
        """Publish occupancy grid map."""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(map_msg)
    
    def publish_pose(self):
        """Publish estimated robot pose and TF."""
        # Pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.state[0]
        pose_msg.pose.position.y = self.state[1]
        
        quat = quaternion_from_euler(0, 0, self.state[2])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
        
        # TF: map -> odom (identity for simple SLAM)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
    
    def publish_landmarks(self):
        """Publish detected landmarks for RViz visualization."""
        if self.num_landmarks == 0:
            return
        
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for i in range(self.num_landmarks):
            pose = Pose()
            pose.position.x = self.state[3 + 2 * i]
            pose.position.y = self.state[3 + 2 * i + 1]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.landmarks_pub.publish(msg)


# =============================================================================
# Main Entry Point
# =============================================================================

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
