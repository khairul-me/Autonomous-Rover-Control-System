#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Autonomous RC Car Navigation System using Intel RealSense D455
================================================================
This comprehensive system provides real-time autonomous navigation with:
- Advanced point cloud processing and ground plane detection
- Robust obstacle detection with multiple validation layers
- Dynamic speed control based on obstacle proximity
- Multi-angle pathfinding with collision prediction
- Real-time visualization and debugging tools

Author: RC Car Navigation System
Version: 2.0
Date: 2024
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import time
from collections import deque
from dataclasses import dataclass
from typing import Tuple, Optional, List
import logging
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ===================================
# CONFIGURATION CLASSES
# ===================================

class SteeringCommand(Enum):
    """Enumeration for steering commands"""
    STOP = "STOP"
    STRAIGHT = "Go Straight"
    SLIGHT_LEFT = "Slight Left"
    LEFT = "Turn Left"
    HARD_LEFT = "Hard Left"
    SLIGHT_RIGHT = "Slight Right"
    RIGHT = "Turn Right"
    HARD_RIGHT = "Hard Right"
    REVERSE = "Reverse"

@dataclass
class CameraConfig:
    """Camera configuration parameters"""
    depth_width: int = 848
    depth_height: int = 480
    color_width: int = 848
    color_height: int = 480
    fps: int = 30
    
    # D455 specific optimal settings
    laser_power: float = 150.0  # 0-360 mW
    confidence_threshold: int = 2  # 0-3
    
@dataclass
class ProcessingConfig:
    """Point cloud processing configuration"""
    voxel_size: float = 0.03  # 3cm voxels for better detail
    
    # Ground plane detection
    ransac_distance_threshold: float = 0.02  # 2cm tolerance
    ransac_n_points: int = 3
    ransac_iterations: int = 1000
    
    # Statistical outlier removal
    nb_neighbors: int = 20
    std_ratio: float = 2.0
    
    # Clustering parameters
    cluster_eps: float = 0.05  # 5cm
    cluster_min_points: int = 10
    
@dataclass
class NavigationConfig:
    """Navigation and occupancy grid configuration"""
    # Grid parameters
    grid_size_m: float = 4.0  # 4x4 meter area
    grid_resolution: int = 200  # 200x200 cells for 2cm resolution
    
    # Obstacle parameters
    min_obstacle_height: float = 0.03  # 3cm minimum height
    max_obstacle_height: float = 2.0   # 2m maximum height
    
    # Car dimensions (in meters)
    car_width: float = 0.25
    car_length: float = 0.35
    
    # Safety parameters
    safety_margin: float = 0.1  # 10cm safety buffer
    emergency_stop_distance: float = 0.3  # 30cm
    
    # Pathfinding parameters
    num_angles: int = 21  # Number of angles to evaluate
    max_steering_angle: float = 45.0  # Maximum steering angle in degrees
    look_ahead_distance: float = 2.0  # How far to look ahead in meters
    
    # Speed control parameters
    max_speed: float = 1.0  # Maximum speed (normalized)
    min_speed: float = 0.2  # Minimum speed (normalized)
    speed_reduction_distance: float = 1.5  # Start slowing at this distance

# ===================================
# MAIN NAVIGATION SYSTEM CLASS
# ===================================

class AutonomousNavigationSystem:
    def __init__(self, camera_config=None, processing_config=None, nav_config=None):
        """Initialize the autonomous navigation system"""
        self.camera_config = camera_config or CameraConfig()
        self.processing_config = processing_config or ProcessingConfig()
        self.nav_config = nav_config or NavigationConfig()
        
        # Calculate derived parameters
        self.cell_size_m = self.nav_config.grid_size_m / self.nav_config.grid_resolution
        self.car_width_cells = int(self.nav_config.car_width / self.cell_size_m)
        self.car_length_cells = int(self.nav_config.car_length / self.cell_size_m)
        self.safety_margin_cells = int(self.nav_config.safety_margin / self.cell_size_m)
        
        # Initialize components
        self.pipeline = None
        self.align = None
        self.depth_intrinsics = None
        self.post_processing_filters = []
        
        # Visualization components
        self.vis_pcd = o3d.geometry.PointCloud()
        self.vis = None
        
        # History buffers for temporal filtering
        self.obstacle_history = deque(maxlen=5)
        self.path_history = deque(maxlen=3)
        
        # Performance monitoring
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()
        
        logger.info("Navigation system initialized")
        
    def initialize_camera(self) -> bool:
        """Initialize RealSense D455 with optimal settings"""
        try:
            # Create pipeline and config
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable streams
            config.enable_stream(
                rs.stream.depth,
                self.camera_config.depth_width,
                self.camera_config.depth_height,
                rs.format.z16,
                self.camera_config.fps
            )
            config.enable_stream(
                rs.stream.color,
                self.camera_config.color_width,
                self.camera_config.color_height,
                rs.format.bgr8,
                self.camera_config.fps
            )
            
            # Start pipeline
            logger.info("Starting RealSense pipeline...")
            profile = self.pipeline.start(config)
            
            # Get depth sensor and configure D455 specific settings
            depth_sensor = profile.get_device().first_depth_sensor()
            
            # Set laser power for better range
            if depth_sensor.supports(rs.option.laser_power):
                depth_sensor.set_option(rs.option.laser_power, self.camera_config.laser_power)
                
            # Set confidence threshold
            if depth_sensor.supports(rs.option.confidence_threshold):
                depth_sensor.set_option(rs.option.confidence_threshold, self.camera_config.confidence_threshold)
            
            # Enable high accuracy preset
            if depth_sensor.supports(rs.option.visual_preset):
                depth_sensor.set_option(rs.option.visual_preset, rs.l500_visual_preset.max_range)
            
            # Get intrinsics
            self.depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            
            # Create alignment object
            self.align = rs.align(rs.stream.color)
            
            # Setup post-processing filters
            self.setup_post_processing_filters()
            
            logger.info("✅ Camera initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            return False
    
    def setup_post_processing_filters(self):
        """Configure advanced post-processing filters for D455"""
        # Decimation filter - reduces depth frame density
        decimation = rs.decimation_filter()
        decimation.set_option(rs.option.filter_magnitude, 2)
        
        # Spatial filter - edge-preserving smoothing
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 3)
        spatial.set_option(rs.option.filter_smooth_alpha, 0.55)
        spatial.set_option(rs.option.filter_smooth_delta, 25)
        spatial.set_option(rs.option.holes_fill, 3)  # Fill all holes
        
        # Temporal filter - reduces temporal noise
        temporal = rs.temporal_filter()
        temporal.set_option(rs.option.filter_smooth_alpha, 0.35)
        temporal.set_option(rs.option.filter_smooth_delta, 50)
        
        # Hole filling filter
        hole_filling = rs.hole_filling_filter()
        hole_filling.set_option(rs.option.holes_fill, 2)  # Fill with farthest from around
        
        # Threshold filter - remove points outside reliable range
        threshold = rs.threshold_filter()
        threshold.set_option(rs.option.min_distance, 0.2)  # 20cm minimum
        threshold.set_option(rs.option.max_distance, 4.0)  # 4m maximum for indoor use
        
        self.post_processing_filters = [
            decimation,
            threshold,
            spatial,
            temporal,
            hole_filling
        ]
        
        logger.info("Post-processing filters configured")
    
    def apply_filters(self, depth_frame):
        """Apply all post-processing filters to depth frame"""
        filtered_frame = depth_frame
        for filter_obj in self.post_processing_filters:
            filtered_frame = filter_obj.process(filtered_frame)
        return filtered_frame
    
    def create_point_cloud(self, depth_frame, color_frame=None) -> Optional[o3d.geometry.PointCloud]:
        """Create Open3D point cloud from depth frame with optional color"""
        try:
            if not depth_frame:
                return None
            
            # Create point cloud from depth
            depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
            
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                self.depth_intrinsics.width,
                self.depth_intrinsics.height,
                self.depth_intrinsics.fx,
                self.depth_intrinsics.fy,
                self.depth_intrinsics.ppx,
                self.depth_intrinsics.ppy
            )
            
            if color_frame:
                color_image = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))
                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_image, depth_image, depth_scale=1000.0, convert_rgb_to_intensity=False
                )
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            else:
                pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image, intrinsic)
            
            # Transform to align with world coordinates (camera facing forward)
            # D455 specific transformation
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            
            return pcd
            
        except Exception as e:
            logger.error(f"Error creating point cloud: {e}")
            return None
    
    def detect_ground_plane(self, pcd: o3d.geometry.PointCloud) -> Tuple[Optional[np.ndarray], Optional[List[int]]]:
        """
        Robust ground plane detection using RANSAC with validation
        Returns: (plane_model, inlier_indices)
        """
        try:
            if not pcd or not pcd.has_points():
                return None, None
            
            # Downsample for faster processing
            pcd_down = pcd.voxel_down_sample(voxel_size=self.processing_config.voxel_size)
            
            # Remove statistical outliers
            pcd_clean, _ = pcd_down.remove_statistical_outlier(
                nb_neighbors=self.processing_config.nb_neighbors,
                std_ratio=self.processing_config.std_ratio
            )
            
            # RANSAC plane segmentation
            plane_model, inliers = pcd_clean.segment_plane(
                distance_threshold=self.processing_config.ransac_distance_threshold,
                ransac_n=self.processing_config.ransac_n_points,
                num_iterations=self.processing_config.ransac_iterations
            )
            
            # Validate plane model (should be roughly horizontal)
            if plane_model is not None:
                a, b, c, d = plane_model
                normal = np.array([a, b, c])
                normal = normal / np.linalg.norm(normal)
                
                # Check if plane is roughly horizontal (normal pointing up/down)
                vertical_alignment = abs(normal[1])  # Y is vertical in our coordinate system
                if vertical_alignment < 0.8:  # Plane is too tilted
                    logger.warning(f"Detected plane is not horizontal enough: {vertical_alignment:.2f}")
                    return None, None
            
            return plane_model, inliers
            
        except Exception as e:
            logger.error(f"Ground plane detection failed: {e}")
            return None, None
    
    def identify_obstacles(self, pcd: o3d.geometry.PointCloud, ground_inliers: List[int]) -> o3d.geometry.PointCloud:
        """
        Advanced obstacle detection with clustering and filtering
        """
        try:
            if not pcd or not pcd.has_points() or ground_inliers is None:
                return o3d.geometry.PointCloud()
            
            # Extract non-ground points
            obstacles_pcd = pcd.select_by_index(ground_inliers, invert=True)
            
            if not obstacles_pcd.has_points():
                return obstacles_pcd
            
            # Filter by height
            points = np.asarray(obstacles_pcd.points)
            height_mask = (points[:, 1] > -self.nav_config.max_obstacle_height) & \
                         (points[:, 1] < -self.nav_config.min_obstacle_height)
            
            valid_points = points[height_mask]
            
            if valid_points.shape[0] == 0:
                return o3d.geometry.PointCloud()
            
            # Create filtered point cloud
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(valid_points)
            
            # Cluster obstacles to remove noise
            if filtered_pcd.has_points():
                labels = np.array(filtered_pcd.cluster_dbscan(
                    eps=self.processing_config.cluster_eps,
                    min_points=self.processing_config.cluster_min_points,
                    print_progress=False
                ))
                
                # Keep only valid clusters (label >= 0)
                mask = labels >= 0
                filtered_pcd = filtered_pcd.select_by_index(np.where(mask)[0])
            
            return filtered_pcd
            
        except Exception as e:
            logger.error(f"Obstacle identification failed: {e}")
            return o3d.geometry.PointCloud()
    
    def create_occupancy_grid(self, obstacles_pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """
        Create a 2D occupancy grid with temporal filtering
        """
        # Initialize empty grid
        grid = np.zeros((self.nav_config.grid_resolution, self.nav_config.grid_resolution), dtype=np.float32)
        
        if not obstacles_pcd or not obstacles_pcd.has_points():
            return grid.astype(np.uint8)
        
        # Get obstacle points
        points = np.asarray(obstacles_pcd.points)
        
        # Project to 2D grid
        x_coords = points[:, 0]
        z_coords = points[:, 2]
        
        # Convert to grid coordinates
        grid_x = (x_coords / self.cell_size_m + self.nav_config.grid_resolution / 2).astype(int)
        grid_z = (z_coords / self.cell_size_m).astype(int)
        
        # Filter valid coordinates
        valid_mask = (grid_x >= 0) & (grid_x < self.nav_config.grid_resolution) & \
                    (grid_z >= 0) & (grid_z < self.nav_config.grid_resolution)
        
        # Update grid with obstacle probability
        grid[grid_z[valid_mask], grid_x[valid_mask]] = 1.0
        
        # Apply temporal filtering
        self.obstacle_history.append(grid)
        if len(self.obstacle_history) > 1:
            # Average over recent frames for stability
            grid = np.mean(self.obstacle_history, axis=0)
        
        # Apply Gaussian blur for smoother transitions
        grid = cv2.GaussianBlur(grid, (5, 5), 1.0)
        
        # Threshold to binary
        grid = (grid > 0.3).astype(np.uint8) * 255
        
        return grid
    
    def inflate_obstacles(self, grid: np.ndarray) -> np.ndarray:
        """Apply safety margin around obstacles"""
        if self.safety_margin_cells <= 0:
            return grid
        
        kernel_size = 2 * self.safety_margin_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated = cv2.dilate(grid, kernel, iterations=1)
        
        return inflated
    
    def find_best_path(self, grid: np.ndarray) -> Tuple[int, SteeringCommand, float, float]:
        """
        Advanced pathfinding with collision prediction and speed control
        Returns: (best_path_column, steering_command, recommended_speed, closest_obstacle_distance)
        """
        # Inflate obstacles for safety
        inflated_grid = self.inflate_obstacles(grid)
        
        # Car position (bottom center of grid)
        car_row = self.nav_config.grid_resolution - 1
        car_col = self.nav_config.grid_resolution // 2
        
        # Generate angles to evaluate
        angles = np.linspace(
            -self.nav_config.max_steering_angle,
            self.nav_config.max_steering_angle,
            self.nav_config.num_angles
        )
        
        # Evaluate each path
        path_scores = []
        path_distances = []
        collision_distances = []
        
        for angle_deg in angles:
            angle_rad = np.deg2rad(angle_deg)
            
            # Trace path
            clear_distance = 0
            closest_obstacle = float('inf')
            
            for step in range(1, int(self.nav_config.look_ahead_distance / self.cell_size_m)):
                # Calculate position along path
                row = int(car_row - step * np.cos(angle_rad))
                col = int(car_col - step * np.sin(angle_rad))
                
                # Check bounds
                if not (0 <= row < self.nav_config.grid_resolution and 
                       0 <= col < self.nav_config.grid_resolution):
                    break
                
                # Check for collision with car width consideration
                collision = False
                for offset in range(-self.car_width_cells//2, self.car_width_cells//2 + 1):
                    check_col = col + offset
                    if 0 <= check_col < self.nav_config.grid_resolution:
                        if inflated_grid[row, check_col] > 0:
                            collision = True
                            distance = step * self.cell_size_m
                            closest_obstacle = min(closest_obstacle, distance)
                            break
                
                if collision:
                    break
                    
                clear_distance = step * self.cell_size_m
            
            path_distances.append(clear_distance)
            collision_distances.append(closest_obstacle)
            
            # Calculate score with straight-ahead bias
            center_index = self.nav_config.num_angles // 2
            angle_penalty = abs(angles.tolist().index(angle_deg) - center_index) / center_index
            score = clear_distance * (1.0 - 0.3 * angle_penalty)  # 30% penalty for turning
            path_scores.append(score)
        
        # Find best path
        best_index = np.argmax(path_scores)
        best_angle = angles[best_index]
        best_distance = path_distances[best_index]
        closest_obstacle_dist = min(collision_distances)
        
        # Smooth path selection using history
        self.path_history.append(best_index)
        if len(self.path_history) == self.path_history.maxlen:
            # Use weighted average of recent paths
            weights = np.array([0.2, 0.3, 0.5])  # More weight to recent
            smoothed_index = int(np.average(self.path_history, weights=weights))
            smoothed_index = np.clip(smoothed_index, 0, len(angles) - 1)
            best_angle = angles[smoothed_index]
            best_index = smoothed_index
        
        # Determine steering command
        if closest_obstacle_dist < self.nav_config.emergency_stop_distance:
            steering_command = SteeringCommand.STOP
        elif abs(best_angle) < 5:
            steering_command = SteeringCommand.STRAIGHT
        elif abs(best_angle) < 15:
            steering_command = SteeringCommand.SLIGHT_LEFT if best_angle < 0 else SteeringCommand.SLIGHT_RIGHT
        elif abs(best_angle) < 30:
            steering_command = SteeringCommand.LEFT if best_angle < 0 else SteeringCommand.RIGHT
        else:
            steering_command = SteeringCommand.HARD_LEFT if best_angle < 0 else SteeringCommand.HARD_RIGHT
        
        # Calculate recommended speed based on obstacles
        if closest_obstacle_dist < self.nav_config.emergency_stop_distance:
            speed = 0.0
        elif closest_obstacle_dist < self.nav_config.speed_reduction_distance:
            # Linear speed reduction
            speed_factor = (closest_obstacle_dist - self.nav_config.emergency_stop_distance) / \
                          (self.nav_config.speed_reduction_distance - self.nav_config.emergency_stop_distance)
            speed = self.nav_config.min_speed + speed_factor * \
                   (self.nav_config.max_speed - self.nav_config.min_speed)
        else:
            speed = self.nav_config.max_speed
        
        # Further reduce speed when turning
        if abs(best_angle) > 15:
            speed *= 0.7  # 30% speed reduction when turning
        
        # Calculate visualization column
        vis_col = int(car_col - np.sin(np.deg2rad(best_angle)) * (self.nav_config.grid_resolution * 0.4))
        vis_col = np.clip(vis_col, 0, self.nav_config.grid_resolution - 1)
        
        return vis_col, steering_command, speed, closest_obstacle_dist
    
    def visualize_occupancy_grid(self, grid: np.ndarray, best_path_col: int, 
                                steering_command: SteeringCommand, speed: float) -> np.ndarray:
        """Enhanced visualization with more information"""
        # Create color visualization
        viz = np.zeros((self.nav_config.grid_resolution, self.nav_config.grid_resolution, 3), dtype=np.uint8)
        
        # Show inflated obstacles in gray
        inflated = self.inflate_obstacles(grid)
        viz[inflated > 0] = (80, 80, 80)
        
        # Show actual obstacles in red
        viz[grid > 0] = (0, 0, 255)
        
        # Draw best path in green with gradient
        car_row = self.nav_config.grid_resolution - 1
        car_col = self.nav_config.grid_resolution // 2
        
        # Draw path line
        for row in range(car_row, -1, -1):
            alpha = (car_row - row) / car_row
            col = int(car_col + (best_path_col - car_col) * alpha)
            if 0 <= col < self.nav_config.grid_resolution:
                # Path with width
                for offset in range(-self.car_width_cells//2, self.car_width_cells//2 + 1):
                    if 0 <= col + offset < self.nav_config.grid_resolution:
                        color_intensity = int(255 * (1 - alpha * 0.5))
                        viz[row, col + offset] = (0, color_intensity, 0)
        
        # Draw car position
        cv2.rectangle(
            viz,
            (car_col - self.car_width_cells//2, car_row - self.car_length_cells),
            (car_col + self.car_width_cells//2, car_row),
            (255, 255, 0),  # Yellow for car
            -1
        )
        
        # Add car direction indicator
        cv2.arrowedLine(
            viz,
            (car_col, car_row - self.car_length_cells//2),
            (car_col, car_row - self.car_length_cells - 10),
            (255, 255, 255),
            2
        )
        
        # Flip for correct orientation
        viz = cv2.flip(viz, 0)
        
        # Resize for better visibility
        viz = cv2.resize(viz, (400, 400), interpolation=cv2.INTER_NEAREST)
        
        # Add text overlays
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(viz, f"Command: {steering_command.value}", (10, 30), 
                   font, 0.7, (255, 255, 255), 2)
        cv2.putText(viz, f"Speed: {speed:.1%}", (10, 60), 
                   font, 0.7, (255, 255, 255), 2)
        
        # Add legend
        cv2.rectangle(viz, (10, 360), (30, 380), (0, 0, 255), -1)
        cv2.putText(viz, "Obstacle", (35, 375), font, 0.5, (255, 255, 255), 1)
        cv2.rectangle(viz, (120, 360), (140, 380), (80, 80, 80), -1)
        cv2.putText(viz, "Safety", (145, 375), font, 0.5, (255, 255, 255), 1)
        cv2.rectangle(viz, (210, 360), (230, 380), (0, 255, 0), -1)
        cv2.putText(viz, "Path", (235, 375), font, 0.5, (255, 255, 255), 1)
        cv2.rectangle(viz, (290, 360), (310, 380), (255, 255, 0), -1)
        cv2.putText(viz, "Car", (315, 375), font, 0.5, (255, 255, 255), 1)
        
        return viz
    
    def process_frame(self, depth_frame, color_frame):
        """Main processing pipeline for a single frame"""
        try:
            # Apply filters to depth frame
            filtered_depth = self.apply_filters(depth_frame)
            
            # Create point cloud
            pcd = self.create_point_cloud(filtered_depth, color_frame)
            if not pcd or not pcd.has_points():
                return None, None, None, None, None
            
            # Detect ground plane
            plane_model, ground_inliers = self.detect_ground_plane(pcd)
            if plane_model is None or ground_inliers is None:
                logger.warning("Ground plane detection failed")
                return None, None, None, None, None
            
            # Identify obstacles
            obstacles_pcd = self.identify_obstacles(pcd, ground_inliers)
            
            # Create occupancy grid
            occupancy_grid = self.create_occupancy_grid(obstacles_pcd)
            
            # Find best path
            best_path_col, steering_command, speed, obstacle_distance = self.find_best_path(occupancy_grid)
            
            # Prepare visualization point cloud
            ground_pcd = pcd.select_by_index(ground_inliers)
            ground_pcd.paint_uniform_color([0, 0.7, 0])  # Green for ground
            obstacles_pcd.paint_uniform_color([1, 0, 0])  # Red for obstacles
            
            # Combine for visualization
            vis_pcd = ground_pcd + obstacles_pcd
            
            return occupancy_grid, best_path_col, steering_command, speed, vis_pcd
            
        except Exception as e:
            logger.error(f"Frame processing error: {e}")
            return None, None, None, None, None
    
    def update_performance_stats(self):
        """Track and display performance metrics"""
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.frame_times.append(frame_time)
        self.last_frame_time = current_time
        
        if len(self.frame_times) == self.frame_times.maxlen:
            avg_fps = 1.0 / np.mean(self.frame_times)
            return avg_fps
        return 0
    
    def run(self):
        """Main execution loop"""
        try:
            # Initialize camera
            if not self.initialize_camera():
                logger.error("Failed to initialize camera")
                return
            
            # Setup visualization windows
            cv2.namedWindow('Live Camera Feed', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Navigation Map', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Depth View', cv2.WINDOW_AUTOSIZE)
            
            # Setup Open3D visualization
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window('3D Point Cloud', width=800, height=600)
            
            # Add coordinate frame for reference
            coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
            self.vis.add_geometry(coordinate_frame)
            
            first_frame = True
            logger.info("🚗 Autonomous navigation system started! Press 'q' to quit")
            
            # Control variables
            current_steering = SteeringCommand.STOP
            current_speed = 0.0
            
            while True:
                # Get frames
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # Get images
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Process frame
                occupancy_grid, best_path_col, steering_command, speed, vis_pcd = \
                    self.process_frame(depth_frame, color_frame)
                
                # Update control smoothly
                if steering_command is not None:
                    current_steering = steering_command
                    current_speed = speed
                
                # Create visualizations
                display_image = color_image.copy()
                
                # Add overlay information
                fps = self.update_performance_stats()
                cv2.putText(display_image, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_image, f"Command: {current_steering.value}", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(display_image, f"Speed: {current_speed:.1%}", (10, 110),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
                # Display color image
                cv2.imshow('Live Camera Feed', display_image)
                
                # Display depth colormap
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.08),
                    cv2.COLORMAP_JET
                )
                cv2.imshow('Depth View', depth_colormap)
                
                # Display navigation map
                if occupancy_grid is not None and best_path_col is not None:
                    nav_viz = self.visualize_occupancy_grid(
                        occupancy_grid, best_path_col, current_steering, current_speed
                    )
                    cv2.imshow('Navigation Map', nav_viz)
                
                # Update 3D visualization
                if vis_pcd is not None and vis_pcd.has_points():
                    self.vis_pcd.points = vis_pcd.points
                    self.vis_pcd.colors = vis_pcd.colors
                    
                    if first_frame:
                        self.vis.add_geometry(self.vis_pcd)
                        
                        # Set camera view
                        ctr = self.vis.get_view_control()
                        ctr.set_lookat([0, 0, 1])
                        ctr.set_front([0, -0.5, -0.5])
                        ctr.set_up([0, -1, 0])
                        ctr.set_zoom(0.5)
                        
                        first_frame = False
                    else:
                        self.vis.update_geometry(self.vis_pcd)
                    
                    self.vis.poll_events()
                    self.vis.update_renderer()
                
                # Check for exit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q or ESC
                    logger.info("Shutting down...")
                    break
                elif key == ord('s'):  # Save snapshot
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"snapshot_{timestamp}.png", display_image)
                    if occupancy_grid is not None:
                        cv2.imwrite(f"map_{timestamp}.png", nav_viz)
                    logger.info(f"Snapshot saved: {timestamp}")
                elif key == ord('r'):  # Reset
                    self.obstacle_history.clear()
                    self.path_history.clear()
                    logger.info("History buffers reset")
                
        except Exception as e:
            logger.error(f"Runtime error: {e}")
            
        finally:
            # Cleanup
            if self.pipeline:
                self.pipeline.stop()
            cv2.destroyAllWindows()
            if self.vis:
                self.vis.destroy_window()
            logger.info("✅ System shutdown complete")

# ===================================
# MAIN ENTRY POINT
# ===================================

def main():
    """Main entry point for the autonomous navigation system"""
    
    # You can customize configurations here
    camera_config = CameraConfig(
        depth_width=848,
        depth_height=480,
        fps=30,
        laser_power=150.0  # D455 optimal power
    )
    
    processing_config = ProcessingConfig(
        voxel_size=0.03,
        ransac_distance_threshold=0.02
    )
    
    nav_config = NavigationConfig(
        grid_size_m=4.0,
        grid_resolution=200,
        car_width=0.25,
        car_length=0.35,
        emergency_stop_distance=0.3
    )
    
    # Create and run the navigation system
    nav_system = AutonomousNavigationSystem(
        camera_config=camera_config,
        processing_config=processing_config,
        nav_config=nav_config
    )
    
    try:
        nav_system.run()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"System error: {e}")
        raise

if __name__ == "__main__":
    main()
