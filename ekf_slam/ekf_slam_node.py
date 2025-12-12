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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math


# =============================================================================
# TUNABLE PARAMETERS - Adjust these values to tune EKF SLAM behavior
# =============================================================================

# EKF Noise Parameters
PROCESS_NOISE = [0.01, 0.01, 0.01]      # Q matrix diagonal: [x, y, θ] motion uncertainty
MEASUREMENT_NOISE = [0.1, 0.1]          # R matrix diagonal: [range, bearing] sensor uncertainty

# Data Association
ASSOCIATION_THRESHOLD = 1.0             # Mahalanobis distance threshold (χ² 2-DOF 95% ≈ 5.99)

# EKF Update Limits (prevents large jumps)
MAX_POS_DELTA = 0.01                      # Max position change per update (meters)
MAX_THETA_DELTA = 0.01                    # Max angle change per update (radians, ~5.7°)

# Corner Detection (Split-and-Merge)
SPLIT_THRESHOLD = 0.05                    # Max distance from point to fitted line (m)
MIN_LINE_POINTS = 10                     # Minimum points to form a line segment
MIN_LINE_LENGTH = 0.5                    # Minimum line length (m)
POINT_CLUSTER_THRESHOLD = 0.3            # Max distance between consecutive points (m)
CORNER_ANGLE_SPLIT = 50                  # Angle threshold for splitting at corners (degrees)
CORNER_ANGLE_MIN = 80.0                  # Min line angle for corner detection (degrees)
CORNER_ANGLE_MAX = 100.0                  # Max line angle for corner detection (degrees)

# Landmark Management
MAX_LANDMARKS = 20                       # Maximum number of landmarks to track
MIN_LANDMARK_SEPARATION = 0.3            # Minimum distance (m) between landmarks

# Corner Stability Verification
CORNER_CONFIRM_FRAMES = 5                # Number of frames a corner must be seen to confirm
CORNER_CONFIRM_DISTANCE = 0.2            # Max distance (m) to consider same corner across frames
CORNER_CANDIDATE_TIMEOUT = 10            # Frames before a candidate corner is forgotten

# Map Generation
USE_EKF_POSE_FOR_MAP = True              # True: use EKF pose, False: use odom pose (more stable)

# =============================================================================
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
        
        # Use tunable parameters from top of file
        self.Q = np.diag(PROCESS_NOISE)          # Process noise (motion model)
        self.R = np.diag(MEASUREMENT_NOISE)      # Measurement noise [range, bearing]
        
        # Data association threshold (Mahalanobis distance squared)
        self.association_threshold = ASSOCIATION_THRESHOLD
        
        # Corner detection parameters (Split-and-Merge)
        self.split_threshold = SPLIT_THRESHOLD
        self.min_line_points = MIN_LINE_POINTS
        self.min_line_length = MIN_LINE_LENGTH
        self.corner_angle_min = CORNER_ANGLE_MIN
        self.corner_angle_max = CORNER_ANGLE_MAX
        
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
        self.lines_pub = self.create_publisher(MarkerArray, '/detected_lines', 10)
        self.corners_pub = self.create_publisher(MarkerArray, '/detected_corners', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store detected line segments for visualization
        self.current_line_segments = []
        
        # Corner candidate tracking for stability verification
        # Each candidate: {'x': float, 'y': float, 'count': int, 'last_seen': int}
        self.corner_candidates = []
        self.frame_counter = 0
        
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
    # Feature Extraction - Corner Detection
    # =========================================================================
    
    def scan_callback(self, msg):
        """
        Process laser scan: extract corner landmarks, perform data association and EKF update.
        Also updates occupancy grid for visualization.
        """
        if not self.initialized or self.prev_odom is None:
            return
        
        # Get pose for map generation based on switch
        if USE_EKF_POSE_FOR_MAP:
            # Use EKF estimated pose
            map_x, map_y, map_theta = self.state[0], self.state[1], self.state[2]
        else:
            # Use odometry pose (more stable, no EKF corrections)
            map_x = self.prev_odom.pose.pose.position.x
            map_y = self.prev_odom.pose.pose.position.y
            _, _, map_theta = euler_from_quaternion(
                self.prev_odom.pose.pose.orientation.x,
                self.prev_odom.pose.pose.orientation.y,
                self.prev_odom.pose.pose.orientation.z,
                self.prev_odom.pose.pose.orientation.w
            )
        
        # ---------------------------------------------------------------------
        # 1. Update occupancy grid
        # ---------------------------------------------------------------------
        self.update_occupancy_grid(msg, map_x, map_y, map_theta)
        
        # ---------------------------------------------------------------------
        # 2. Extract corner landmarks from scan
        # ---------------------------------------------------------------------
        landmarks_polar = self.extract_landmarks(msg)  # List of (range, bearing)
        
        # ---------------------------------------------------------------------
        # 3. Data association and EKF update
        # Only update with the single best matching landmark to avoid accumulated drift
        # ---------------------------------------------------------------------
        best_match = None
        best_mahal = float('inf')
        new_landmarks = []
        
        for z_range, z_bearing in landmarks_polar:
            landmark_idx = self.associate_landmark(z_range, z_bearing)
            
            if landmark_idx >= 0:
                # Compute innovation for this match
                robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
                lx = self.state[3 + 2 * landmark_idx]
                ly = self.state[3 + 2 * landmark_idx + 1]
                dx = lx - robot_x
                dy = ly - robot_y
                pred_range = np.sqrt(dx**2 + dy**2)
                pred_bearing = normalize_angle(np.arctan2(dy, dx) - robot_theta)
                
                innovation_range = abs(z_range - pred_range)
                innovation_bearing = abs(normalize_angle(z_bearing - pred_bearing))
                
                # Innovation gating: reject observations with large innovation
                max_range_innovation = 0.5   # meters
                max_bearing_innovation = 0.3  # radians (~17 degrees)
                
                if innovation_range < max_range_innovation and innovation_bearing < max_bearing_innovation:
                    innovation_mag = np.sqrt(innovation_range**2 + innovation_bearing**2)
                    if innovation_mag < best_mahal:
                        best_mahal = innovation_mag
                        best_match = (landmark_idx, z_range, z_bearing)
                # If innovation is too large, treat as new landmark candidate
                else:
                    new_landmarks.append((z_range, z_bearing))
            else:
                # Collect new landmarks to add later
                new_landmarks.append((z_range, z_bearing))
        
        # Update with only the best matching landmark
        if best_match is not None:
            self.ekf_update(best_match[0], best_match[1], best_match[2])
        
        # Increment frame counter
        self.frame_counter += 1
        
        # Process new landmark candidates with stability verification
        if new_landmarks and self.num_landmarks < MAX_LANDMARKS:
            self._process_corner_candidates(new_landmarks)
        
        # Remove stale candidates
        self._cleanup_corner_candidates()
        
        # Publish landmarks for visualization
        self.publish_landmarks()
    
    def _process_corner_candidates(self, new_corners):
        """
        Process new corner detections with stability verification.
        Only add corners as landmarks if they've been seen consistently.
        """
        robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
        
        for z_range, z_bearing in new_corners:
            # Convert to global coordinates
            global_angle = robot_theta + z_bearing
            cx = robot_x + z_range * np.cos(global_angle)
            cy = robot_y + z_range * np.sin(global_angle)
            
            # Check if this corner matches an existing candidate
            matched = False
            for candidate in self.corner_candidates:
                dist = np.sqrt((cx - candidate['x'])**2 + (cy - candidate['y'])**2)
                if dist < CORNER_CONFIRM_DISTANCE:
                    # Update existing candidate
                    candidate['count'] += 1
                    candidate['last_seen'] = self.frame_counter
                    # Average position for stability
                    candidate['x'] = (candidate['x'] * (candidate['count'] - 1) + cx) / candidate['count']
                    candidate['y'] = (candidate['y'] * (candidate['count'] - 1) + cy) / candidate['count']
                    matched = True
                    
                    # Check if candidate is confirmed
                    if candidate['count'] >= CORNER_CONFIRM_FRAMES:
                        # Check if too close to existing landmarks
                        too_close = False
                        for i in range(self.num_landmarks):
                            lx = self.state[3 + 2 * i]
                            ly = self.state[3 + 2 * i + 1]
                            dist_to_lm = np.sqrt((candidate['x'] - lx)**2 + (candidate['y'] - ly)**2)
                            if dist_to_lm < MIN_LANDMARK_SEPARATION:
                                too_close = True
                                break
                        
                        if not too_close:
                            # Add as landmark
                            lm_dx = candidate['x'] - robot_x
                            lm_dy = candidate['y'] - robot_y
                            lm_range = np.sqrt(lm_dx**2 + lm_dy**2)
                            lm_bearing = normalize_angle(np.arctan2(lm_dy, lm_dx) - robot_theta)
                            self.add_landmark(lm_range, lm_bearing)
                        
                        # Remove from candidates (whether added or rejected)
                        self.corner_candidates.remove(candidate)
                    break
            
            if not matched:
                # Add as new candidate
                self.corner_candidates.append({
                    'x': cx,
                    'y': cy,
                    'count': 1,
                    'last_seen': self.frame_counter
                })
    
    def _cleanup_corner_candidates(self):
        """Remove stale corner candidates that haven't been seen recently."""
        self.corner_candidates = [
            c for c in self.corner_candidates
            if (self.frame_counter - c['last_seen']) < CORNER_CANDIDATE_TIMEOUT
        ]
    
    def _scan_to_cartesian(self, msg):
        """
        Convert laser scan to list of (x, y) points in robot frame.
        Only keeps valid points within range limits.
        """
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_range < r < self.max_range and np.isfinite(r):
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment
        return points
    
    def _fit_line(self, points):
        """
        Fit a line to points using least squares.
        
        Returns: (a, b, c) where ax + by + c = 0, normalized so a² + b² = 1
                 Also returns the endpoints (x1, y1), (x2, y2)
        """
        if len(points) < 2:
            return None, None, None
        
        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])
        
        # Use PCA-based line fitting for robustness
        mean_x, mean_y = np.mean(xs), np.mean(ys)
        centered_x = xs - mean_x
        centered_y = ys - mean_y
        
        # Covariance matrix
        cov_xx = np.sum(centered_x * centered_x)
        cov_xy = np.sum(centered_x * centered_y)
        cov_yy = np.sum(centered_y * centered_y)
        
        # Eigenvector for smallest eigenvalue gives line normal
        # For 2x2: use direct formula
        trace = cov_xx + cov_yy
        det = cov_xx * cov_yy - cov_xy * cov_xy
        
        # Eigenvalues: λ = (trace ± sqrt(trace² - 4*det)) / 2
        discriminant = trace * trace - 4 * det
        if discriminant < 0:
            discriminant = 0
        sqrt_disc = np.sqrt(discriminant)
        
        # Smaller eigenvalue
        lambda_min = (trace - sqrt_disc) / 2
        
        # Eigenvector for lambda_min: (a, b) is the normal
        if abs(cov_xy) > 1e-10:
            a = cov_xy
            b = lambda_min - cov_xx
        elif abs(cov_xx - lambda_min) > 1e-10:
            a = 1.0
            b = 0.0
        else:
            a = 0.0
            b = 1.0
        
        # Normalize
        norm = np.sqrt(a*a + b*b)
        if norm < 1e-10:
            return None, None, None
        a, b = a / norm, b / norm
        
        # c = -(a*mean_x + b*mean_y)
        c = -(a * mean_x + b * mean_y)
        
        # Compute endpoints: project first and last point onto line
        # Direction vector (perpendicular to normal)
        dx, dy = -b, a
        
        # Project points onto line direction
        projs = [dx * (p[0] - mean_x) + dy * (p[1] - mean_y) for p in points]
        min_proj, max_proj = min(projs), max(projs)
        
        x1 = mean_x + dx * min_proj
        y1 = mean_y + dy * min_proj
        x2 = mean_x + dx * max_proj
        y2 = mean_y + dy * max_proj
        
        return (a, b, c), (x1, y1), (x2, y2)
    
    def _point_to_line_distance(self, point, line_params):
        """Calculate perpendicular distance from point to line ax + by + c = 0."""
        a, b, c = line_params
        x, y = point
        return abs(a * x + b * y + c)
    
    def _split_and_merge(self, points, depth=0):
        """
        Split-and-Merge algorithm for line segment detection.
        
        Recursively splits point set until all points are within threshold
        distance from the fitted line. Also detects corners by checking
        for significant direction changes.
        
        Args:
            points: List of (x, y) cartesian coordinates
            depth: Recursion depth for safety
            
        Returns:
            List of line segments as ((x1,y1), (x2,y2), [points])
        """
        if len(points) < self.min_line_points or depth > 50:
            return []
        
        # Fit line to all points
        line_params, p1, p2 = self._fit_line(points)
        if line_params is None:
            return []
        
        # Method 1: Find point with maximum distance from line
        max_dist = 0
        max_dist_idx = -1
        for i, pt in enumerate(points):
            d = self._point_to_line_distance(pt, line_params)
            if d > max_dist:
                max_dist = d
                max_dist_idx = i
        
        # Method 2: Find point with maximum direction change (corner detection)
        max_angle_change = 0
        max_angle_idx = -1
        window = 3  # Look at direction change over this window
        
        if len(points) >= 2 * window + 1:
            for i in range(window, len(points) - window):
                # Direction vector before point i
                dx1 = points[i][0] - points[i - window][0]
                dy1 = points[i][1] - points[i - window][1]
                
                # Direction vector after point i
                dx2 = points[i + window][0] - points[i][0]
                dy2 = points[i + window][1] - points[i][1]
                
                # Normalize
                len1 = np.sqrt(dx1*dx1 + dy1*dy1)
                len2 = np.sqrt(dx2*dx2 + dy2*dy2)
                
                if len1 > 1e-6 and len2 > 1e-6:
                    dx1, dy1 = dx1/len1, dy1/len1
                    dx2, dy2 = dx2/len2, dy2/len2
                    
                    # Angle between directions (using cross product for signed angle)
                    cross = dx1 * dy2 - dy1 * dx2
                    dot = dx1 * dx2 + dy1 * dy2
                    angle = abs(np.arctan2(cross, dot))
                    
                    if angle > max_angle_change:
                        max_angle_change = angle
                        max_angle_idx = i
        
        # Decide where to split
        split_idx = -1
        angle_threshold = np.radians(CORNER_ANGLE_SPLIT)
        
        # Prefer angle-based split if significant corner detected
        if max_angle_change > angle_threshold and max_angle_idx > 0 and max_angle_idx < len(points) - 1:
            split_idx = max_angle_idx
        # Otherwise use distance-based split
        elif max_dist > self.split_threshold and max_dist_idx > 0 and max_dist_idx < len(points) - 1:
            split_idx = max_dist_idx
        
        if split_idx >= 0:
            # Split into two segments
            left_points = points[:split_idx + 1]
            right_points = points[split_idx:]
            
            left_segments = self._split_and_merge(left_points, depth + 1)
            right_segments = self._split_and_merge(right_points, depth + 1)
            
            return left_segments + right_segments
        else:
            # Points fit the line well, return this segment
            # Check minimum length
            length = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            if length >= self.min_line_length:
                return [(p1, p2, points)]
            else:
                return []
    
    def _line_intersection(self, seg1, seg2):
        """
        Calculate intersection point of two line segments.
        
        Args:
            seg1, seg2: Line segments as ((x1,y1), (x2,y2), points)
            
        Returns:
            (x, y) intersection point or None if parallel/no intersection
        """
        (x1, y1), (x2, y2), _ = seg1
        (x3, y3), (x4, y4), _ = seg2
        
        # Line 1: (x1,y1) to (x2,y2)
        # Line 2: (x3,y3) to (x4,y4)
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None  # Parallel lines
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        
        # Intersection point
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        
        return (ix, iy)
    
    def _angle_between_lines(self, seg1, seg2):
        """
        Calculate angle between two line segments in degrees.
        
        Returns angle in [0, 180] degrees.
        """
        (x1, y1), (x2, y2), _ = seg1
        (x3, y3), (x4, y4), _ = seg2
        
        # Direction vectors
        d1 = np.array([x2 - x1, y2 - y1])
        d2 = np.array([x4 - x3, y4 - y3])
        
        # Normalize
        n1 = np.linalg.norm(d1)
        n2 = np.linalg.norm(d2)
        if n1 < 1e-10 or n2 < 1e-10:
            return 0
        
        d1 = d1 / n1
        d2 = d2 / n2
        
        # Angle between vectors
        cos_angle = np.clip(np.dot(d1, d2), -1.0, 1.0)
        angle = np.degrees(np.arccos(abs(cos_angle)))
        
        return angle
    
    def _segments_are_adjacent(self, seg1, seg2, threshold=0.2):
        """
        Check if two line segments are adjacent (share an endpoint approximately).
        
        Returns: True if segments share an endpoint within threshold distance
        """
        (p1_start, p1_end, _) = seg1
        (p2_start, p2_end, _) = seg2
        
        # Check all endpoint combinations
        def dist(a, b):
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        min_dist = min(
            dist(p1_start, p2_start),
            dist(p1_start, p2_end),
            dist(p1_end, p2_start),
            dist(p1_end, p2_end)
        )
        
        return min_dist < threshold
    
    def detect_corners(self, line_segments):
        """
        Detect corners from line segment intersections.
        
        Finds adjacent line segments that meet at approximately 90 degrees
        and computes their intersection as a corner landmark.
        
        Args:
            line_segments: List of ((x1,y1), (x2,y2), points) line segments
            
        Returns:
            List of (range, bearing) corner landmarks in robot frame
        """
        corners = []
        n = len(line_segments)
        
        for i in range(n):
            for j in range(i + 1, n):
                seg1 = line_segments[i]
                seg2 = line_segments[j]
                
                # Check if segments are adjacent
                if not self._segments_are_adjacent(seg1, seg2):
                    continue
                
                # Calculate angle between segments
                # _angle_between_lines returns angle in [0, 90] degrees
                angle = self._angle_between_lines(seg1, seg2)
                
                # For a 90° corner, the angle between line directions should be ~90°
                # But _angle_between_lines uses abs(cos), so it returns [0, 90]
                # A perpendicular corner would have angle close to 90°
                # Allow 80° to 100° corners (which means angle should be 80-90°)
                
                min_corner_angle = CORNER_ANGLE_MIN
                max_corner_angle = CORNER_ANGLE_MAX
                
                if min_corner_angle <= angle <= max_corner_angle:
                    # Calculate intersection point
                    intersection = self._line_intersection(seg1, seg2)
                    if intersection is None:
                        continue
                    
                    ix, iy = intersection
                    
                    # Check if intersection is within reasonable range
                    dist_to_robot = np.sqrt(ix**2 + iy**2)
                    if dist_to_robot < self.min_range or dist_to_robot > self.max_range:
                        continue
                    
                    # Convert to polar (range, bearing)
                    lm_range = dist_to_robot
                    lm_bearing = np.arctan2(iy, ix)
                    
                    corners.append((lm_range, lm_bearing))
        
        return corners
    
    def _cluster_points_by_distance(self, points, threshold=POINT_CLUSTER_THRESHOLD):
        """
        Cluster consecutive points by distance.
        
        Points that are further apart than threshold are considered
        to belong to different objects/surfaces.
        
        Args:
            points: List of (x, y) cartesian coordinates
            threshold: Max distance between consecutive points in same cluster
            
        Returns:
            List of clusters, each cluster is a list of (x, y) points
        """
        if len(points) < 2:
            return [points] if points else []
        
        clusters = []
        current_cluster = [points[0]]
        
        for i in range(1, len(points)):
            x1, y1 = points[i - 1]
            x2, y2 = points[i]
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            if dist < threshold:
                current_cluster.append(points[i])
            else:
                if len(current_cluster) >= self.min_line_points:
                    clusters.append(current_cluster)
                current_cluster = [points[i]]
        
        # Don't forget the last cluster
        if len(current_cluster) >= self.min_line_points:
            clusters.append(current_cluster)
        
        return clusters
    
    def extract_landmarks(self, msg):
        """
        Extract corner landmarks from laser scan using Split-and-Merge.
        
        Steps:
        1. Convert scan points to Cartesian coordinates
        2. Cluster consecutive points by distance (separate objects)
        3. Apply Split-and-Merge to each cluster to detect line segments
        4. Find adjacent line segment intersections (corners)
        
        Returns: List of (range, bearing) corner landmarks in robot frame
        """
        # 1. Convert scan to Cartesian points
        points = self._scan_to_cartesian(msg)
        if len(points) < self.min_line_points * 2:
            self.current_line_segments = []
            self.publish_line_segments()
            return []
        
        # 2. Cluster consecutive points by distance
        clusters = self._cluster_points_by_distance(points, threshold=POINT_CLUSTER_THRESHOLD)
        
        # 3. Apply Split-and-Merge to each cluster
        all_line_segments = []
        for cluster in clusters:
            segments = self._split_and_merge(cluster)
            all_line_segments.extend(segments)
        
        # Store for visualization
        self.current_line_segments = all_line_segments
        
        # Publish visualization (even if no corners detected)
        self.publish_line_segments()
        
        if len(all_line_segments) < 2:
            self.publish_corner_markers([])
            return []  # Need at least 2 lines to form a corner
        
        # 4. Detect corners from line intersections
        corners = self.detect_corners(all_line_segments)
        
        # Publish corner visualization
        self.publish_corner_markers(corners)
        
        return corners
    
    def publish_line_segments(self):
        """Publish detected line segments as MarkerArray for RViz visualization."""
        marker_array = MarkerArray()
        
        # Publish in base_scan frame to match laser scan display
        for i, seg in enumerate(self.current_line_segments):
            (x1, y1), (x2, y2), _ = seg
            
            marker = Marker()
            marker.header.frame_id = 'base_scan'  # Same frame as laser scan
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'line_segments'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Line points (already in robot/sensor frame)
            from geometry_msgs.msg import Point
            p1, p2 = Point(), Point()
            p1.x, p1.y, p1.z = float(x1), float(y1), 0.0
            p2.x, p2.y, p2.z = float(x2), float(y2), 0.0
            marker.points = [p1, p2]
            
            # Line appearance
            marker.scale.x = 0.05  # Line width
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2 seconds
            
            marker_array.markers.append(marker)
        
        self.lines_pub.publish(marker_array)
    
    def publish_corner_markers(self, corners):
        """Publish detected corners as MarkerArray for RViz visualization."""
        marker_array = MarkerArray()
        
        for i, (r, bearing) in enumerate(corners):
            # Convert polar to Cartesian in sensor frame
            x = r * np.cos(bearing)
            y = r * np.sin(bearing)
            
            marker = Marker()
            marker.header.frame_id = 'base_scan'  # Same frame as laser scan
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'corners'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Sphere size
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2 seconds
            
            marker_array.markers.append(marker)
        
        self.corners_pub.publish(marker_array)
    
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
        
        # Compute state update
        delta = K @ innovation
        
        # Clamp state update to prevent large jumps
        max_pos_delta = MAX_POS_DELTA
        max_theta_delta = MAX_THETA_DELTA
        
        delta[0] = np.clip(delta[0], -max_pos_delta, max_pos_delta)
        delta[1] = np.clip(delta[1], -max_pos_delta, max_pos_delta)
        delta[2] = np.clip(delta[2], -max_theta_delta, max_theta_delta)
        
        # State update: x = x + clamped delta
        self.state = self.state + delta
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
        if self.prev_odom is None:
            return
        
        # Get EKF estimated pose in map frame
        ekf_x, ekf_y, ekf_theta = self.state[0], self.state[1], self.state[2]
        
        # Get odometry pose
        odom_x = self.prev_odom.pose.pose.position.x
        odom_y = self.prev_odom.pose.pose.position.y
        _, _, odom_theta = euler_from_quaternion(
            self.prev_odom.pose.pose.orientation.x,
            self.prev_odom.pose.pose.orientation.y,
            self.prev_odom.pose.pose.orientation.z,
            self.prev_odom.pose.pose.orientation.w
        )
        
        # Pose message
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
        # We need: map_T_odom such that map_T_base = map_T_odom * odom_T_base
        # Therefore: map_T_odom = map_T_base * inv(odom_T_base)
        
        # inv(odom_T_base): rotate by -odom_theta, translate by -R(-odom_theta) * odom_pos
        # Then multiply with map_T_base (ekf pose)
        
        # Combined rotation: map_theta = theta_diff + odom_theta => theta_diff = map_theta - odom_theta
        theta_diff = normalize_angle(ekf_theta - odom_theta)
        
        # For the translation:
        # map_T_odom.translation = map_T_base.translation - map_T_odom.rotation * odom_T_base.translation
        # Solving: tx, ty = ekf_x, ekf_y - R(theta_diff) * (odom_x, odom_y)
        cos_d, sin_d = np.cos(theta_diff), np.sin(theta_diff)
        tx = ekf_x - (cos_d * odom_x - sin_d * odom_y)
        ty = ekf_y - (sin_d * odom_x + cos_d * odom_y)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = 0.0
        
        quat_tf = quaternion_from_euler(0, 0, theta_diff)
        t.transform.rotation.x = quat_tf[0]
        t.transform.rotation.y = quat_tf[1]
        t.transform.rotation.z = quat_tf[2]
        t.transform.rotation.w = quat_tf[3]
        
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
