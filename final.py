#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simplified and Robust Autonomous RC Car Navigation System
==========================================================
Designed specifically for Intel RealSense D455
Focus on reliability and real-time obstacle detection
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
from collections import deque
from enum import Enum
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SteeringCommand(Enum):
    STOP = "STOP"
    STRAIGHT = "Go Straight"
    LEFT = "Turn Left"
    RIGHT = "Turn Right"
    SLIGHT_LEFT = "Slight Left"
    SLIGHT_RIGHT = "Slight Right"

class SimpleNavigationSystem:
    def __init__(self):
        """Initialize the simplified navigation system"""
        # Camera settings
        self.WIDTH = 640
        self.HEIGHT = 480
        self.FPS = 30
        
        # Navigation parameters
        self.GRID_SIZE = 80  # Simplified grid
        self.SAFE_DISTANCE = 0.5  # meters
        self.STOP_DISTANCE = 0.3  # meters
        
        # Components
        self.pipeline = None
        self.align = None
        
        # State tracking
        self.obstacle_buffer = deque(maxlen=3)  # Short history
        self.last_command = SteeringCommand.STRAIGHT
        self.last_speed = 0.5
        
        logger.info("Simple Navigation System Initialized")
    
    def initialize_camera(self):
        """Initialize RealSense D455"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable streams
            config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)
            config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Get device and configure
            device = profile.get_device()
            depth_sensor = device.first_depth_sensor()
            
            # Set laser power
            if depth_sensor.supports(rs.option.laser_power):
                depth_sensor.set_option(rs.option.laser_power, 200)
            
            # Set visual preset for better accuracy
            if depth_sensor.supports(rs.option.visual_preset):
                depth_sensor.set_option(rs.option.visual_preset, 3)  # High Accuracy
            
            # Align depth to color
            self.align = rs.align(rs.stream.color)
            
            # Setup filters
            self.setup_filters()
            
            logger.info("Camera initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Camera initialization failed: {e}")
            return False
    
    def setup_filters(self):
        """Setup simplified depth filters"""
        # Basic filters for clean depth
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.holes_fill, 3)
        
        self.temporal = rs.temporal_filter()
        
        self.hole_filling = rs.hole_filling_filter()
        
    def process_depth_frame(self, depth_frame):
        """Apply filters to depth frame"""
        depth = depth_frame
        depth = self.spatial.process(depth)
        depth = self.temporal.process(depth)
        depth = self.hole_filling.process(depth)
        return depth
    
    def create_obstacle_map(self, depth_frame):
        """Create a simple 2D obstacle map from depth data"""
        # Convert to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Convert depth to meters
        depth_meters = depth_image * 0.001
        
        # Create occupancy grid
        grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.uint8)
        
        # Sample the depth image in regions
        h, w = depth_image.shape
        
        # Focus on the lower 2/3 of the image (ground level)
        start_row = h // 3
        
        # Divide into grid cells
        cell_h = (h - start_row) // self.GRID_SIZE
        cell_w = w // self.GRID_SIZE
        
        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                # Get region of depth image
                row_start = start_row + i * cell_h
                row_end = min(row_start + cell_h, h)
                col_start = j * cell_w
                col_end = min(col_start + cell_w, w)
                
                # Get minimum distance in this cell
                region = depth_meters[row_start:row_end, col_start:col_end]
                
                # Filter out zero/invalid readings
                valid_depths = region[(region > 0.2) & (region < 3.0)]
                
                if len(valid_depths) > 0:
                    min_distance = np.min(valid_depths)
                    
                    # Mark as obstacle if too close
                    if min_distance < 1.5:  # 1.5 meters
                        # Invert the grid (bottom of image = bottom of grid)
                        grid[self.GRID_SIZE - 1 - i, j] = int(255 * (1.5 - min_distance) / 1.5)
        
        return grid
    
    def find_best_direction(self, obstacle_grid):
        """Find the best direction to move"""
        # Get the bottom portion of the grid (near the car)
        near_field = obstacle_grid[self.GRID_SIZE//2:, :]
        
        # Divide into three sections: left, center, right
        width = obstacle_grid.shape[1]
        third = width // 3
        
        left_section = near_field[:, :third]
        center_section = near_field[:, third:2*third]
        right_section = near_field[:, 2*third:]
        
        # Calculate obstacle density in each section
        left_score = np.sum(left_section)
        center_score = np.sum(center_section)
        right_score = np.sum(right_section)
        
        # Calculate minimum distance to obstacle
        center_distances = []
        for row in range(len(center_section)):
            if np.max(center_section[row]) > 100:  # Obstacle detected
                # Convert row to distance (rough approximation)
                distance = (len(center_section) - row) * 0.05  # 5cm per cell
                center_distances.append(distance)
        
        min_distance = min(center_distances) if center_distances else 2.0
        
        # Determine command based on scores
        if min_distance < self.STOP_DISTANCE:
            command = SteeringCommand.STOP
            speed = 0.0
        elif center_score < 1000:  # Clear ahead
            command = SteeringCommand.STRAIGHT
            speed = min(1.0, min_distance / 1.5)  # Speed based on distance
        elif left_score < right_score:  # Left is clearer
            if left_score < 1000:
                command = SteeringCommand.LEFT
            else:
                command = SteeringCommand.SLIGHT_LEFT
            speed = 0.5
        else:  # Right is clearer
            if right_score < 1000:
                command = SteeringCommand.RIGHT
            else:
                command = SteeringCommand.SLIGHT_RIGHT
            speed = 0.5
        
        # Smooth speed changes
        speed = 0.7 * self.last_speed + 0.3 * speed
        
        return command, speed, min_distance
    
    def visualize_grid(self, grid, command, speed):
        """Create visualization of the obstacle grid"""
        # Create color image
        viz = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
        
        # Add car position
        car_y = self.GRID_SIZE - 5
        car_x = self.GRID_SIZE // 2
        cv2.rectangle(viz, (car_x - 5, car_y - 8), (car_x + 5, car_y), (255, 255, 0), -1)
        
        # Add direction arrow
        if command == SteeringCommand.STRAIGHT:
            cv2.arrowedLine(viz, (car_x, car_y - 4), (car_x, car_y - 20), (0, 255, 0), 2)
        elif command in [SteeringCommand.LEFT, SteeringCommand.SLIGHT_LEFT]:
            cv2.arrowedLine(viz, (car_x, car_y - 4), (car_x - 15, car_y - 20), (0, 255, 0), 2)
        elif command in [SteeringCommand.RIGHT, SteeringCommand.SLIGHT_RIGHT]:
            cv2.arrowedLine(viz, (car_x, car_y - 4), (car_x + 15, car_y - 20), (0, 255, 0), 2)
        
        # Resize for display
        viz = cv2.resize(viz, (400, 400), interpolation=cv2.INTER_NEAREST)
        
        # Add text
        cv2.putText(viz, f"Command: {command.value}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(viz, f"Speed: {speed:.0%}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Add legend
        cv2.putText(viz, "Red = Close", (10, 380), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(viz, "Yellow = Car", (120, 380), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(viz, "Green = Path", (230, 380), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return viz
    
    def run(self):
        """Main loop"""
        if not self.initialize_camera():
            return
        
        # Create windows
        cv2.namedWindow('Camera View', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Depth View', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Navigation Map', cv2.WINDOW_AUTOSIZE)
        
        logger.info("Starting navigation loop... Press 'q' to quit")
        
        frame_count = 0
        fps_timer = time.time()
        fps = 0
        
        try:
            while True:
                # Get frames
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # Apply filters
                depth_frame = self.process_depth_frame(depth_frame)
                
                # Get images
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Create obstacle map
                obstacle_grid = self.create_obstacle_map(depth_frame)
                
                # Buffer for stability
                self.obstacle_buffer.append(obstacle_grid)
                if len(self.obstacle_buffer) > 1:
                    # Average recent grids
                    obstacle_grid = np.mean(self.obstacle_buffer, axis=0).astype(np.uint8)
                
                # Find best direction
                command, speed, min_distance = self.find_best_direction(obstacle_grid)
                
                # Update state
                self.last_command = command
                self.last_speed = speed
                
                # Calculate FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    current_time = time.time()
                    fps = 30 / (current_time - fps_timer)
                    fps_timer = current_time
                
                # Visualize color image with overlay
                display = color_image.copy()
                cv2.putText(display, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display, f"Command: {command.value}", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(display, f"Speed: {speed:.0%}", (10, 110),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(display, f"Min Dist: {min_distance:.2f}m", (10, 150),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                
                # Show obstacle count
                obstacle_pixels = np.sum(obstacle_grid > 100)
                cv2.putText(display, f"Obstacles: {obstacle_pixels}", (10, 190),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                
                # Display windows
                cv2.imshow('Camera View', display)
                
                # Depth visualization
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.08),
                    cv2.COLORMAP_JET
                )
                cv2.imshow('Depth View', depth_colormap)
                
                # Navigation map
                nav_viz = self.visualize_grid(obstacle_grid, command, speed)
                cv2.imshow('Navigation Map', nav_viz)
                
                # Check for exit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    # Reset
                    self.obstacle_buffer.clear()
                    self.last_command = SteeringCommand.STRAIGHT
                    self.last_speed = 0.5
                    logger.info("System reset")
                elif key == ord('s'):
                    # Save snapshot
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"camera_{timestamp}.png", display)
                    cv2.imwrite(f"depth_{timestamp}.png", depth_colormap)
                    cv2.imwrite(f"nav_{timestamp}.png", nav_viz)
                    logger.info(f"Saved snapshot: {timestamp}")
                
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
        
        finally:
            # Cleanup
            self.pipeline.stop()
            cv2.destroyAllWindows()
            logger.info("System shutdown complete")

def main():
    """Entry point"""
    system = SimpleNavigationSystem()
    try:
        system.run()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"System error: {e}")
        raise

if __name__ == "__main__":
    main()
