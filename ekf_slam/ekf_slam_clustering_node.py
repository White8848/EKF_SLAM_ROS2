#!/usr/bin/env python3
"""
EKF SLAM Node with Clustering-based Landmark Detection

Uses DBSCAN clustering to identify point landmarks from laser scan data.
Suitable for environments with cylindrical or point-like obstacles.

State Vector: [x, y, θ, lx₁, ly₁, lx₂, ly₂, ..., lxₙ, lyₙ]
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math


# =============================================================================
# TUNABLE PARAMETERS
# =============================================================================

# EKF Noise Parameters
PROCESS_NOISE = [0.01, 0.01, 0.01]      # Q matrix diagonal: [x, y, θ]
MEASUREMENT_NOISE = [0.3, 0.3]          # R matrix diagonal: [range, bearing]

# Data Association
ASSOCIATION_THRESHOLD = 5.99             # Mahalanobis distance threshold

# EKF Update Limits
MAX_POS_DELTA = 0.05                    # Max position change per update (m)
MAX_THETA_DELTA = 0.05                  # Max angle change per update (rad)

# Clustering Parameters (DBSCAN-like)
CLUSTER_EPS = 0.3                       # Max distance between points in cluster (m)
CLUSTER_MIN_POINTS = 3                  # Min points to form a cluster
MAX_CLUSTER_SIZE = 1.5                  # Max cluster diameter (m) - reject large clusters

# Circle Fitting Parameters
MIN_CIRCLE_RADIUS = 0.1                 # Minimum cylinder radius (m)
MAX_CIRCLE_RADIUS = 0.6                 # Maximum cylinder radius (m)
CIRCLE_FIT_ERROR_THRESHOLD = 0.1        # Max fitting error to accept (m)

# Landmark Management
MAX_LANDMARKS = 30                      # Maximum number of landmarks
MIN_LANDMARK_SEPARATION = 0.6           # Minimum distance between landmarks (m)
LANDMARK_CONFIRM_FRAMES = 3             # Frames to confirm new landmark
LANDMARK_CONFIRM_DISTANCE = 0.4         # Distance threshold for confirmation
MAX_LANDMARK_RANGE = 3.0                # Max range for landmark detection (< laser range)

# Map Generation
USE_EKF_POSE_FOR_MAP = True

# =============================================================================
# Utility Functions
# =============================================================================

def quaternion_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    ]


def euler_from_quaternion(x, y, z, w):
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
    return np.arctan2(np.sin(angle), np.cos(angle))


# =============================================================================
# EKF SLAM Node with Clustering
# =============================================================================

class EKFSLAMClusteringNode(Node):
    """
    EKF SLAM node using clustering for landmark detection.
    Suitable for point-like obstacles (cylinders, poles, etc.)
    """
    
    def __init__(self):
        super().__init__('ekf_slam_clustering_node')
        
        # Parameters
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 400)
        self.declare_parameter('map_height', 400)
        self.declare_parameter('max_laser_range', 3.5)
        self.declare_parameter('min_laser_range', 0.12)
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.max_range = self.get_parameter('max_laser_range').value
        self.min_range = self.get_parameter('min_laser_range').value
        
        # EKF State
        self.state = np.array([0.0, 0.0, 0.0])
        self.covariance = np.eye(3) * 0.01
        self.num_landmarks = 0
        
        self.Q = np.diag(PROCESS_NOISE)
        self.R = np.diag(MEASUREMENT_NOISE)
        self.association_threshold = ASSOCIATION_THRESHOLD
        
        # Occupancy Grid
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0
        
        # State tracking
        self.prev_odom = None
        self.initialized = False
        self.frame_counter = 0
        
        # Landmark candidates for stability verification
        self.landmark_candidates = []
        
        # ROS 2 Interfaces
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
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
        self.clusters_pub = self.create_publisher(MarkerArray, '/detected_clusters', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('EKF SLAM Clustering Node initialized')
    
    # =========================================================================
    # EKF Prediction
    # =========================================================================
    
    def odom_callback(self, msg):
        if self.prev_odom is None:
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
        
        dx_global = curr_x - prev_x
        dy_global = curr_y - prev_y
        dtheta = normalize_angle(curr_yaw - prev_yaw)
        
        cos_prev = np.cos(prev_yaw)
        sin_prev = np.sin(prev_yaw)
        dx_local = dx_global * cos_prev + dy_global * sin_prev
        dy_local = -dx_global * sin_prev + dy_global * cos_prev
        
        self.predict(dx_local, dy_local, dtheta)
        self.prev_odom = msg
        self.publish_pose()
    
    def predict(self, dx_local, dy_local, dtheta):
        theta = self.state[2]
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        dx_global = dx_local * cos_t - dy_local * sin_t
        dy_global = dx_local * sin_t + dy_local * cos_t
        
        self.state[0] += dx_global
        self.state[1] += dy_global
        self.state[2] = normalize_angle(self.state[2] + dtheta)
        
        n = len(self.state)
        G = np.eye(n)
        G[0, 2] = -dx_local * sin_t - dy_local * cos_t
        G[1, 2] = dx_local * cos_t - dy_local * sin_t
        
        self.covariance = G @ self.covariance @ G.T
        self.covariance[:3, :3] += self.Q
    
    # =========================================================================
    # Clustering-based Landmark Detection
    # =========================================================================
    
    def scan_callback(self, msg):
        if not self.initialized or self.prev_odom is None:
            return
        
        self.frame_counter += 1
        
        # Get pose for map
        if USE_EKF_POSE_FOR_MAP:
            map_x, map_y, map_theta = self.state[0], self.state[1], self.state[2]
        else:
            map_x = self.prev_odom.pose.pose.position.x
            map_y = self.prev_odom.pose.pose.position.y
            _, _, map_theta = euler_from_quaternion(
                self.prev_odom.pose.pose.orientation.x,
                self.prev_odom.pose.pose.orientation.y,
                self.prev_odom.pose.pose.orientation.z,
                self.prev_odom.pose.pose.orientation.w
            )
        
        # Update occupancy grid
        self.update_occupancy_grid(msg, map_x, map_y, map_theta)
        
        # Extract landmarks using clustering
        landmarks_polar = self.extract_landmarks_clustering(msg)
        
        # Data association and EKF update
        self.process_landmarks(landmarks_polar)
        
        # Publish landmarks
        self.publish_landmarks()
        self.publish_clusters(landmarks_polar)
    
    def extract_landmarks_clustering(self, msg):
        """
        Extract landmarks using clustering + circle fitting.
        Returns list of (range, bearing) for circle centers.
        """
        # Convert scan to cartesian points
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_range < r < self.max_range and np.isfinite(r):
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y, angle, r))
            angle += msg.angle_increment
        
        if len(points) < CLUSTER_MIN_POINTS:
            return []
        
        # DBSCAN-like clustering
        clusters = self.dbscan_cluster(points)
        
        # Convert clusters to polar coordinates using circle fitting
        landmarks = []
        for cluster in clusters:
            if len(cluster) < CLUSTER_MIN_POINTS:
                continue
            
            # Extract x, y coordinates
            xs = np.array([p[0] for p in cluster])
            ys = np.array([p[1] for p in cluster])
            
            # Check cluster extent
            diameter = max(np.max(xs) - np.min(xs), np.max(ys) - np.min(ys))
            if diameter > MAX_CLUSTER_SIZE:
                continue  # Skip large clusters (walls)
            
            # Try circle fitting
            result = self.fit_circle(xs, ys)
            
            if result is not None:
                cx, cy, radius, error = result
                
                # Validate circle fit
                if (MIN_CIRCLE_RADIUS <= radius <= MAX_CIRCLE_RADIUS and
                    error < CIRCLE_FIT_ERROR_THRESHOLD):
                    # Use circle center as landmark
                    lm_range = np.sqrt(cx**2 + cy**2)
                    lm_bearing = np.arctan2(cy, cx)
                    
                    # Only accept if within landmark range (prevents misidentifying distant walls)
                    if self.min_range < lm_range < MAX_LANDMARK_RANGE:
                        landmarks.append((lm_range, lm_bearing))
                        continue
            
            # Fallback to centroid if circle fitting fails
            cx = np.mean(xs)
            cy = np.mean(ys)
            lm_range = np.sqrt(cx**2 + cy**2)
            lm_bearing = np.arctan2(cy, cx)
            
            # Only accept if within landmark range
            if self.min_range < lm_range < MAX_LANDMARK_RANGE:
                landmarks.append((lm_range, lm_bearing))
        
        return landmarks
    
    def fit_circle(self, xs, ys):
        """
        Fit a circle to points using algebraic least squares.
        
        Uses the method: minimize sum of (x^2 + y^2 - 2*cx*x - 2*cy*y + (cx^2 + cy^2 - r^2))^2
        Which simplifies to solving a linear system.
        
        Returns: (cx, cy, radius, mean_error) or None if fitting fails
        """
        n = len(xs)
        if n < 3:
            return None
        
        # Build design matrix for algebraic circle fit
        # Model: x^2 + y^2 + D*x + E*y + F = 0
        # Where: cx = -D/2, cy = -E/2, r = sqrt(cx^2 + cy^2 - F)
        
        A = np.column_stack([xs, ys, np.ones(n)])
        b = -(xs**2 + ys**2)
        
        try:
            # Solve least squares: A @ [D, E, F]^T = b
            result, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            D, E, F = result
        except np.linalg.LinAlgError:
            return None
        
        # Extract circle parameters
        cx = -D / 2.0
        cy = -E / 2.0
        r_squared = cx**2 + cy**2 - F
        
        if r_squared <= 0:
            return None
        
        radius = np.sqrt(r_squared)
        
        # Compute fitting error (mean distance from points to circle)
        distances = np.sqrt((xs - cx)**2 + (ys - cy)**2)
        errors = np.abs(distances - radius)
        mean_error = np.mean(errors)
        
        return (cx, cy, radius, mean_error)
    
    def dbscan_cluster(self, points):
        """
        Simple DBSCAN-like clustering based on Euclidean distance.
        """
        n = len(points)
        visited = [False] * n
        clusters = []
        
        for i in range(n):
            if visited[i]:
                continue
            
            # Find neighbors
            neighbors = []
            for j in range(n):
                if i != j:
                    dist = np.sqrt((points[i][0] - points[j][0])**2 + 
                                   (points[i][1] - points[j][1])**2)
                    if dist < CLUSTER_EPS:
                        neighbors.append(j)
            
            if len(neighbors) >= CLUSTER_MIN_POINTS - 1:
                # Start new cluster
                cluster = [points[i]]
                visited[i] = True
                
                # Expand cluster
                queue = list(neighbors)
                while queue:
                    idx = queue.pop(0)
                    if visited[idx]:
                        continue
                    visited[idx] = True
                    cluster.append(points[idx])
                    
                    # Find neighbors of this point
                    for k in range(n):
                        if not visited[k]:
                            dist = np.sqrt((points[idx][0] - points[k][0])**2 + 
                                           (points[idx][1] - points[k][1])**2)
                            if dist < CLUSTER_EPS:
                                queue.append(k)
                
                clusters.append(cluster)
        
        return clusters
    
    def process_landmarks(self, landmarks_polar):
        """Process detected landmarks: associate or add new ones."""
        best_match = None
        best_mahal = float('inf')
        new_landmarks = []
        
        for z_range, z_bearing in landmarks_polar:
            landmark_idx = self.associate_landmark(z_range, z_bearing)
            
            if landmark_idx >= 0:
                # Compute innovation
                robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
                lx = self.state[3 + 2 * landmark_idx]
                ly = self.state[3 + 2 * landmark_idx + 1]
                dx = lx - robot_x
                dy = ly - robot_y
                pred_range = np.sqrt(dx**2 + dy**2)
                pred_bearing = normalize_angle(np.arctan2(dy, dx) - robot_theta)
                
                innovation_range = abs(z_range - pred_range)
                innovation_bearing = abs(normalize_angle(z_bearing - pred_bearing))
                
                # Innovation gating
                if innovation_range < 0.5 and innovation_bearing < 0.3:
                    innovation_mag = np.sqrt(innovation_range**2 + innovation_bearing**2)
                    if innovation_mag < best_mahal:
                        best_mahal = innovation_mag
                        best_match = (landmark_idx, z_range, z_bearing)
                else:
                    new_landmarks.append((z_range, z_bearing))
            else:
                new_landmarks.append((z_range, z_bearing))
        
        # Update with best match
        if best_match is not None:
            self.ekf_update(best_match[0], best_match[1], best_match[2])
        
        # Process new landmarks with stability check
        if new_landmarks and self.num_landmarks < MAX_LANDMARKS:
            self._process_landmark_candidates(new_landmarks)
        
        # Cleanup stale candidates
        self._cleanup_candidates()
    
    def associate_landmark(self, z_range, z_bearing):
        """Find best matching landmark using Mahalanobis distance."""
        if self.num_landmarks == 0:
            return -1
        
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        best_idx = -1
        best_dist = self.association_threshold
        
        for i in range(self.num_landmarks):
            lx = self.state[3 + 2 * i]
            ly = self.state[3 + 2 * i + 1]
            
            dx = lx - robot_x
            dy = ly - robot_y
            pred_range = np.sqrt(dx**2 + dy**2)
            pred_bearing = normalize_angle(np.arctan2(dy, dx) - robot_theta)
            
            # Innovation
            innov = np.array([
                z_range - pred_range,
                normalize_angle(z_bearing - pred_bearing)
            ])
            
            # Simplified Mahalanobis (use R as covariance)
            dist = np.sqrt(innov @ np.linalg.inv(self.R) @ innov)
            
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        
        return best_idx
    
    def ekf_update(self, landmark_idx, z_range, z_bearing):
        """EKF measurement update."""
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        lx = self.state[3 + 2 * landmark_idx]
        ly = self.state[3 + 2 * landmark_idx + 1]
        
        dx = lx - robot_x
        dy = ly - robot_y
        pred_range = np.sqrt(dx**2 + dy**2)
        pred_bearing = normalize_angle(np.arctan2(dy, dx) - robot_theta)
        
        innovation = np.array([
            z_range - pred_range,
            normalize_angle(z_bearing - pred_bearing)
        ])
        
        H = self._compute_jacobian_H(landmark_idx, dx, dy, pred_range)
        S = H @ self.covariance @ H.T + self.R
        
        try:
            K = self.covariance @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        
        delta = K @ innovation
        
        # Clamp updates
        delta[0] = np.clip(delta[0], -MAX_POS_DELTA, MAX_POS_DELTA)
        delta[1] = np.clip(delta[1], -MAX_POS_DELTA, MAX_POS_DELTA)
        delta[2] = np.clip(delta[2], -MAX_THETA_DELTA, MAX_THETA_DELTA)
        
        self.state = self.state + delta
        self.state[2] = normalize_angle(self.state[2])
        
        n = len(self.state)
        I = np.eye(n)
        self.covariance = (I - K @ H) @ self.covariance
        self.covariance = (self.covariance + self.covariance.T) / 2
    
    def _compute_jacobian_H(self, landmark_idx, dx, dy, q):
        n = len(self.state)
        H = np.zeros((2, n))
        
        if q < 1e-6:
            return H
        
        q2 = q * q
        
        H[0, 0] = -dx / q
        H[0, 1] = -dy / q
        H[0, 2] = 0
        
        H[1, 0] = dy / q2
        H[1, 1] = -dx / q2
        H[1, 2] = -1
        
        lm_start = 3 + 2 * landmark_idx
        H[0, lm_start] = dx / q
        H[0, lm_start + 1] = dy / q
        H[1, lm_start] = -dy / q2
        H[1, lm_start + 1] = dx / q2
        
        return H
    
    def _process_landmark_candidates(self, new_landmarks):
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        for z_range, z_bearing in new_landmarks:
            global_angle = robot_theta + z_bearing
            lx = robot_x + z_range * np.cos(global_angle)
            ly = robot_y + z_range * np.sin(global_angle)
            
            matched = False
            for candidate in self.landmark_candidates:
                dist = np.sqrt((lx - candidate['x'])**2 + (ly - candidate['y'])**2)
                if dist < LANDMARK_CONFIRM_DISTANCE:
                    candidate['count'] += 1
                    candidate['last_seen'] = self.frame_counter
                    candidate['x'] = (candidate['x'] * (candidate['count'] - 1) + lx) / candidate['count']
                    candidate['y'] = (candidate['y'] * (candidate['count'] - 1) + ly) / candidate['count']
                    matched = True
                    
                    if candidate['count'] >= LANDMARK_CONFIRM_FRAMES:
                        # Check separation from existing landmarks
                        too_close = False
                        for i in range(self.num_landmarks):
                            ex = self.state[3 + 2 * i]
                            ey = self.state[3 + 2 * i + 1]
                            if np.sqrt((candidate['x'] - ex)**2 + (candidate['y'] - ey)**2) < MIN_LANDMARK_SEPARATION:
                                too_close = True
                                break
                        
                        if not too_close:
                            self.add_landmark_at(candidate['x'], candidate['y'])
                        
                        self.landmark_candidates.remove(candidate)
                    break
            
            if not matched:
                self.landmark_candidates.append({
                    'x': lx, 'y': ly,
                    'count': 1,
                    'last_seen': self.frame_counter
                })
    
    def _cleanup_candidates(self):
        self.landmark_candidates = [
            c for c in self.landmark_candidates
            if (self.frame_counter - c['last_seen']) < 10
        ]
    
    def add_landmark_at(self, lx, ly):
        """Add a new landmark at global position (lx, ly)."""
        self.state = np.append(self.state, [lx, ly])
        
        n_old = len(self.covariance)
        n_new = n_old + 2
        
        P_new = np.zeros((n_new, n_new))
        P_new[:n_old, :n_old] = self.covariance
        
        # Initial uncertainty for new landmark
        P_new[n_old, n_old] = 1.0
        P_new[n_old + 1, n_old + 1] = 1.0
        
        self.covariance = P_new
        self.num_landmarks += 1
        
        self.get_logger().info(f'Added landmark #{self.num_landmarks} at ({lx:.2f}, {ly:.2f})')
    
    # =========================================================================
    # Occupancy Grid
    # =========================================================================
    
    def update_occupancy_grid(self, msg, robot_x, robot_y, robot_theta):
        angle = msg.angle_min
        cos_t, sin_t = np.cos(robot_theta), np.sin(robot_theta)
        
        for r in msg.ranges:
            if self.min_range < r < self.max_range and np.isfinite(r):
                scan_x = r * np.cos(angle)
                scan_y = r * np.sin(angle)
                global_x = robot_x + scan_x * cos_t - scan_y * sin_t
                global_y = robot_y + scan_x * sin_t + scan_y * cos_t
                
                self._update_cell(global_x, global_y, occupied=True)
                self._raytrace_free(robot_x, robot_y, global_x, global_y)
            elif r >= self.max_range or not np.isfinite(r):
                scan_x = self.max_range * np.cos(angle)
                scan_y = self.max_range * np.sin(angle)
                global_x = robot_x + scan_x * cos_t - scan_y * sin_t
                global_y = robot_y + scan_x * sin_t + scan_y * cos_t
                self._raytrace_free(robot_x, robot_y, global_x, global_y)
            
            angle += msg.angle_increment
    
    def _update_cell(self, x, y, occupied=True):
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            if occupied:
                self.map_data[gy, gx] = min(100, self.map_data[gy, gx] + 10)
            else:
                self.map_data[gy, gx] = max(0, self.map_data[gy, gx] - 5)
    
    def _raytrace_free(self, x0, y0, x1, y1):
        gx0 = int((x0 - self.map_origin_x) / self.map_resolution)
        gy0 = int((y0 - self.map_origin_y) / self.map_resolution)
        gx1 = int((x1 - self.map_origin_x) / self.map_resolution)
        gy1 = int((y1 - self.map_origin_y) / self.map_resolution)
        
        dx, dy = abs(gx1 - gx0), abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        
        x, y = gx0, gy0
        for _ in range(300):
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
        if self.prev_odom is None:
            return
        
        ekf_x, ekf_y, ekf_theta = self.state[0], self.state[1], self.state[2]
        
        odom_x = self.prev_odom.pose.pose.position.x
        odom_y = self.prev_odom.pose.pose.position.y
        _, _, odom_theta = euler_from_quaternion(
            self.prev_odom.pose.pose.orientation.x,
            self.prev_odom.pose.pose.orientation.y,
            self.prev_odom.pose.pose.orientation.z,
            self.prev_odom.pose.pose.orientation.w
        )
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = ekf_x
        pose_msg.pose.position.y = ekf_y
        
        quat = quaternion_from_euler(0, 0, ekf_theta)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
        
        # TF: map -> odom
        theta_diff = normalize_angle(ekf_theta - odom_theta)
        cos_diff = np.cos(theta_diff)
        sin_diff = np.sin(theta_diff)
        tx = ekf_x - (cos_diff * odom_x - sin_diff * odom_y)
        ty = ekf_y - (sin_diff * odom_x + cos_diff * odom_y)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        
        q = quaternion_from_euler(0, 0, theta_diff)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_landmarks(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        
        for i in range(self.num_landmarks):
            pose = Pose()
            pose.position.x = self.state[3 + 2 * i]
            pose.position.y = self.state[3 + 2 * i + 1]
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.landmarks_pub.publish(pose_array)
    
    def publish_clusters(self, landmarks_polar):
        """Publish detected cluster markers for visualization."""
        marker_array = MarkerArray()
        
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        for i, (lm_range, lm_bearing) in enumerate(landmarks_polar):
            global_angle = robot_theta + lm_bearing
            lx = robot_x + lm_range * np.cos(global_angle)
            ly = robot_y + lm_range * np.sin(global_angle)
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'clusters'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = lx
            marker.pose.position.y = ly
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000
            
            marker_array.markers.append(marker)
        
        self.clusters_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
