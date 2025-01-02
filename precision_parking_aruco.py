"""
Precision Parking Using ArUco Markers

This program implements precision parking functionality for a drone/rover using ArUco marker
detection. It uses computer vision to detect ArUco markers and guides the vehicle to park
precisely relative to the detected marker.

Author: Md Khairul Islam
Institution: Hobart and William Smith Colleges, Geneva, NY
Major: Robotics and Computer Science
Contact: khairul.islam@hws.edu

Created: January 2025
Last Modified: January 2025

Dependencies:
- OpenCV
- DroneKit
- NumPy
- imutils
- pymavlink
"""

# Standard library imports
import time
import socket
import math
import argparse

# Third-party imports
import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil

# Constants
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
ARUCO_ID_TO_FIND = 72
MARKER_SIZE_CM = 19  # Size of ArUco marker in centimeters

# Initialize camera
cap = WebcamVideoStream(src=0, height=CAMERA_HEIGHT, width=CAMERA_WIDTH).start()

# ArUco setup
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Camera calibration
CALIB_PATH = "/home/caleberg/repos/video2calibration/calibrationFiles/"
camera_matrix = np.loadtxt(CALIB_PATH + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(CALIB_PATH + 'cameraDistortion.txt', delimiter=',')

def get_aruco_coordinates():
    """
    Detects ArUco marker and returns its coordinates relative to the camera.
    
    Returns:
        tuple: (x, y, z) coordinates of the marker, (0,0,0) if not found
    """
    frame = cap.read()
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    
    corners, ids, rejected = aruco.detectMarkers(
        image=gray_img,
        dictionary=aruco_dict,
        parameters=parameters
    )
    
    if ids is not None and ids[0] == ARUCO_ID_TO_FIND:
        ret = aruco.estimatePoseSingleMarkers(
            corners,
            MARKER_SIZE_CM,
            cameraMatrix=camera_matrix,
            distCoeffs=camera_distortion
        )
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
        
        x = f"{tvec[0]:.2f}"
        y = f"{tvec[1]:.2f}"
        z = f"{tvec[2]:.2f}"
        print(f"MARKER POSITION: x={x} y={y} z={z}")
        return x, y, z
    
    return "0", "0", "0"

def park_at_aruco():
    """
    Controls the vehicle to park at the detected ArUco marker.
    
    Returns:
        int: 0 if parking successful, None otherwise
    """
    # Control parameters
    TURN_ANGLE_MAX = 2  # Maximum turning angle (-2 to 2)
    SPEED_MAX = 0.5     # Maximum speed in m/s
    
    x, y, z = get_aruco_coordinates()
    x, y, z = float(x), float(y), float(z)
    
    if x == 0 and y == 0 and z == 0:
        print("Marker not found. Skipping iteration")
        return None

    # Convert to meters and calculate control values
    x_meters = x/100
    z_meters = z/100
    
    # Determine turn direction and gain
    turn_angle_sign = 1 if x > 0 else -1
    p_turn_gain = 0.125 if z < 500 else 0.061
    
    # Calculate turn angle
    turn_angle = min(abs(x_meters) * p_turn_gain, TURN_ANGLE_MAX)
    turn_angle *= turn_angle_sign
    
    # Calculate speed based on distance
    if z > 200:
        speed = SPEED_MAX
    elif 100 < z < 200:
        speed = 0.3
    else:
        print("TARGET REACHED")
        send_local_ned_velocity(0, 0, 0)
        return 0
    
    print(f"VELOCITY= {speed}")
    send_local_ned_velocity(speed, turn_angle, 0)
    return None

[... Rest of the improved functions remain similar but with added documentation ...]

def main():
    """
    Main execution function that handles vehicle setup and parking procedure.
    """
    vehicle = connect_my_copter()
    arm()

    success_counter = 0
    SUCCESS_THRESHOLD = 5
    
    while True:
        ret = park_at_aruco()
        if ret == 0:
            success_counter += 1
            if success_counter == SUCCESS_THRESHOLD:
                vehicle.armed = False
                break
        time.sleep(0.05)

if __name__ == "__main__":
    main()
