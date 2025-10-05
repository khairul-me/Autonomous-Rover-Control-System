# -*- coding: utf-8 -*-
"""
A script for REAL-TIME autonomous navigation using an Intel RealSense camera.
This script performs:
1. Live 3D point cloud generation of the entire scene.
2. Ground plane detection using RANSAC.
3. Obstacle identification using clustering to form solid objects.
4. Creation of a 2D top-down occupancy grid for navigation.
5. Dynamic, multi-angle pathfinding to identify the safest direction.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import time

# ---------------------------------
# -- CONFIGURATION --
# ---------------------------------

# -- Stream settings (tuned for D455)
DEPTH_WIDTH = 848
DEPTH_HEIGHT = 480
COLOR_WIDTH = 848
COLOR_HEIGHT = 480
FRAME_RATE = 30

# -- Point Cloud and Voxelization
VOXEL_SIZE = 0.05  # 5cm voxels for downsampling

# -- Ground Plane Detection (RANSAC)
GROUND_PLANE_DISTANCE_THRESHOLD = 0.04  # 4cm, tolerance for points to be on the plane

# -- Occupancy Grid Configuration
GRID_SIZE_M = 5.0  # Real-world size of the grid (5x5 meters)
GRID_RESOLUTION = 100  # Grid dimensions (100x100 cells)
CELL_SIZE_M = GRID_SIZE_M / GRID_RESOLUTION

# -- Obstacle Definition
OBSTACLE_MIN_HEIGHT_M = 0.05  # Lowered to 5cm to detect shorter objects
OBSTACLE_MAX_HEIGHT_M = 1.5   # 1.5m, ignore ceilings or high objects
DBSCAN_EPS = 0.15             # 15cm, clustering distance for points to be one object
DBSCAN_MIN_POINTS = 10        # Minimum number of points to form an object

# -- Pathfinding Configuration
CAR_WIDTH_CELLS = int(0.3 / CELL_SIZE_M) # Assume car is 30cm wide
INFLATION_RADIUS_CELLS = CAR_WIDTH_CELLS // 2 + 1 # Safety buffer around obstacles

# ---------------------------------
# -- GLOBAL VARIABLES --
# ---------------------------------
pipeline = None
align = None
depth_intrinsics = None
vis_pcd = o3d.geometry.PointCloud() # Reusable point cloud for visualization

def initialize_camera():
    """Initialize RealSense camera pipeline"""
    global pipeline, align, depth_intrinsics

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FRAME_RATE)
    config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FRAME_RATE)

    print("🎥 Starting RealSense pipeline...")
    profile = pipeline.start(config)
    depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    align = rs.align(rs.stream.color)

    print("✅ Camera ready!")

def create_point_cloud_from_depth(depth_frame):
    """Generate an Open3D point cloud from a RealSense depth frame."""
    if not depth_frame:
        return None

    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        o3d.geometry.Image(np.asanyarray(depth_frame.get_data())),
        o3d.camera.PinholeCameraIntrinsic(
            depth_intrinsics.width, depth_intrinsics.height,
            depth_intrinsics.fx, depth_intrinsics.fy,
            depth_intrinsics.ppx, depth_intrinsics.ppy
        )
    )
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd

def process_frame_for_navigation(depth_frame, color_image):
    """Main processing function for navigation."""
    if not depth_frame:
        return color_image, None, None

    pcd_full = create_point_cloud_from_depth(depth_frame)
    if not pcd_full or not pcd_full.has_points():
        return color_image, None, None
    pcd_down = pcd_full.voxel_down_sample(voxel_size=VOXEL_SIZE)

    try:
        plane_model, inlier_indices = pcd_down.segment_plane(
            distance_threshold=GROUND_PLANE_DISTANCE_THRESHOLD,
            ransac_n=3,
            num_iterations=1000
        )
    except Exception:
        return color_image, None, None

    ground_pcd = pcd_down.select_by_index(inlier_indices)
    obstacles_pcd = pcd_down.select_by_index(inlier_indices, invert=True)

    ground_pcd.paint_uniform_color([0, 1, 0])
    obstacles_pcd.paint_uniform_color([1, 0, 0])

    vis_pcd.points = ground_pcd.points
    vis_pcd.points.extend(obstacles_pcd.points)
    vis_pcd.colors = ground_pcd.colors
    vis_pcd.colors.extend(obstacles_pcd.colors)

    occupancy_grid, obstacle_points_2d = create_occupancy_grid(obstacles_pcd)

    best_path_col, steering_command = find_best_path(occupancy_grid)

    map_viz = visualize_occupancy_grid(occupancy_grid, best_path_col, obstacle_points_2d)

    cv2.putText(color_image, f"Steering: {steering_command}", (20, 40),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

    return color_image, map_viz, vis_pcd

def create_occupancy_grid(obstacles_pcd):
    """
    UPGRADED: Create a 2D grid by clustering points into objects and projecting
    their bounding boxes, creating solid obstacles.
    """
    grid = np.zeros((GRID_RESOLUTION, GRID_RESOLUTION), dtype=np.uint8)
    
    obstacle_points = np.asarray(obstacles_pcd.points)
    if obstacle_points.shape[0] < DBSCAN_MIN_POINTS:
        return grid, np.array([])

    # Filter obstacles by height to remove floor noise and ceiling
    valid_height_mask = (obstacle_points[:, 1] > -OBSTACLE_MAX_HEIGHT_M) & (obstacle_points[:, 1] < -OBSTACLE_MIN_HEIGHT_M)
    obstacles_pcd_filtered = obstacles_pcd.select_by_index(np.where(valid_height_mask)[0])
    
    if not obstacles_pcd_filtered.has_points():
        return grid, np.array([])
        
    # Cluster points to identify distinct objects
    labels = np.array(obstacles_pcd_filtered.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_POINTS))
    
    unique_labels = np.unique(labels)
    all_obstacle_pixels = []

    for label in unique_labels:
        if label == -1:  # -1 is noise in DBSCAN
            continue
        
        # Get all points belonging to the current object
        object_indices = np.where(labels == label)[0]
        object_pcd = obstacles_pcd_filtered.select_by_index(object_indices)
        
        # Get the 3D bounding box of the object
        bbox = object_pcd.get_axis_aligned_bounding_box()
        corners = np.asarray(bbox.get_box_points())

        # Project the 3D bounding box corners to 2D grid coordinates
        x_coords = corners[:, 0]
        z_coords = corners[:, 2]
        
        grid_c = (x_coords / CELL_SIZE_M + GRID_RESOLUTION / 2).astype(int)
        grid_r = (z_coords / CELL_SIZE_M).astype(int)

        # Get the convex hull of the projected 2D points (the object's footprint)
        footprint_points = np.vstack((grid_c, grid_r)).T
        if footprint_points.shape[0] > 2:
            hull = cv2.convexHull(footprint_points)
            # Fill the footprint polygon on the grid to create a solid obstacle
            cv2.fillConvexPoly(grid, hull, 255)
            all_obstacle_pixels.extend(footprint_points)

    return grid, np.array(all_obstacle_pixels)


def inflate_obstacles(grid, inflation_radius):
    """Adds a safety buffer around obstacles using morphological dilation."""
    if inflation_radius <= 0:
        return grid
    kernel = np.ones((inflation_radius * 2 + 1, inflation_radius * 2 + 1), np.uint8)
    return cv2.dilate(grid, kernel, iterations=1)

def find_best_path(grid):
    """
    More dynamic pathfinding that evaluates multiple steering angles,
    prefers to go straight, and considers a safety buffer.
    """
    NUM_ANGLES = 15
    MAX_ANGLE_DEG = 60
    STRAIGHT_BIAS_FACTOR = 0.8

    inflated_grid = inflate_obstacles(grid, INFLATION_RADIUS_CELLS)

    angles = np.linspace(-MAX_ANGLE_DEG, MAX_ANGLE_DEG, NUM_ANGLES)
    car_pos_r, car_pos_c = GRID_RESOLUTION - 1, GRID_RESOLUTION // 2

    path_scores = []
    for angle_deg in angles:
        angle_rad = np.deg2rad(angle_deg)
        score = 0
        for step in range(1, GRID_RESOLUTION):
            r = int(car_pos_r - step * np.cos(angle_rad))
            c = int(car_pos_c - step * np.sin(angle_rad))
            
            if not (0 <= r < GRID_RESOLUTION and 0 <= c < GRID_RESOLUTION) or inflated_grid[r, c] > 0:
                break
            score = step
        path_scores.append(score)

    center_index = NUM_ANGLES // 2
    penalties = np.abs(np.arange(NUM_ANGLES) - center_index)
    final_scores = np.array(path_scores) * (1 - (penalties / NUM_ANGLES) * STRAIGHT_BIAS_FACTOR)

    best_path_index = np.argmax(final_scores)
    best_angle_deg = angles[best_path_index]

    if abs(best_angle_deg) < 10:
        command = "Go Straight"
    elif best_angle_deg < -35:
        command = "Hard Left"
    elif best_angle_deg < 0:
        command = "Turn Left"
    elif best_angle_deg > 35:
        command = "Hard Right"
    else:
        command = "Turn Right"

    vis_path_col = int(car_pos_c - np.sin(np.deg2rad(best_angle_deg)) * (GRID_RESOLUTION * 0.25))
    vis_path_col = np.clip(vis_path_col, 0, GRID_RESOLUTION - 1)
        
    return vis_path_col, command

def visualize_occupancy_grid(grid, best_path_col, obstacle_points_2d):
    """Create a visual representation of the grid, safety buffers, and path."""
    map_viz = cv2.cvtColor(np.zeros_like(grid), cv2.COLOR_GRAY2BGR)

    inflated_grid = inflate_obstacles(grid, INFLATION_RADIUS_CELLS)
    map_viz[inflated_grid > 0] = (50, 50, 50)

    cv2.rectangle(map_viz,
                  (best_path_col - CAR_WIDTH_CELLS // 2, 0),
                  (best_path_col + CAR_WIDTH_CELLS // 2, GRID_RESOLUTION),
                  (0, 255, 0), -1)
                  
    map_viz[grid > 0] = (0, 0, 255) # Draw actual obstacles on top in red

    car_pos = (GRID_RESOLUTION // 2, GRID_RESOLUTION - 1)
    cv2.circle(map_viz, car_pos, CAR_WIDTH_CELLS, (255, 0, 0), -1)

    map_viz = cv2.flip(map_viz, 0)
    
    return cv2.resize(map_viz, (400, 400), interpolation=cv2.INTER_NEAREST)

def main():
    global pipeline
    try:
        initialize_camera()

        cv2.namedWindow('Live Feed', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Top-Down Map', cv2.WINDOW_AUTOSIZE)
        
        vis = o3d.visualization.Visualizer()
        vis.create_window('3D Point Cloud', width=800, height=600)
        first_frame = True
        
        print("🚀 Starting LIVE NAVIGATION...")
        print("⏹️  Press 'q' in any window or ESC to quit")

        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            
            processed_color, map_viz, pcd_viz = process_frame_for_navigation(depth_frame, color_image)
            
            cv2.imshow('Live Feed', processed_color)

            if map_viz is not None:
                cv2.imshow('Top-Down Map', map_viz)

            if pcd_viz is not None and pcd_viz.has_points():
                if first_frame:
                    vis.add_geometry(pcd_viz)
                    first_frame = False
                else:
                    vis.update_geometry(pcd_viz)
                vis.poll_events()
                vis.update_renderer()

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("🛑 Quitting...")
                break

    finally:
        if pipeline:
            pipeline.stop()
        cv2.destroyAllWindows()
        if 'vis' in locals():
            vis.destroy_window()
        print("✅ Cleanup complete")

if __name__ == "__main__":
    main()

