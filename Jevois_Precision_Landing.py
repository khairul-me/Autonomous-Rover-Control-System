#!/usr/bin/env python2
"""
Precision Landing System using JeVois Camera

This program implements a precision landing system for drones using JeVois camera
and ArUco marker detection. It tracks markers of different sizes and sends appropriate
landing target messages to the flight controller for precise landing.

Author: Md Khairul Islam
Institution: Hobart and William Smith Colleges, Geneva, NY
Major: Robotics and Computer Science
Contact: khairul.islam@hws.edu

Created: January 2025
Last Modified: January 2025

Dependencies:
- DroneKit
- pymavlink
- numpy
- JeVois
- math
"""

# Standard library imports
import math
from typing import Tuple, Optional

# Third-party imports
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from JeVois import JeVois

class PrecisionLandingController:
    """
    Controller class for precision landing using JeVois camera and ArUco markers.
    """
    
    def __init__(self, connection_string: str = '/dev/ttyUSB0', baud_rate: int = 57600):
        """
        Initialize the precision landing controller.
        
        Args:
            connection_string: Vehicle connection string
            baud_rate: Connection baud rate
        """
        # Camera parameters
        self.horizontal_fov = 70.2 * math.pi/180  # Horizontal field of view in radians
        self.vertical_fov = 43.3 * math.pi/180    # Vertical field of view in radians
        self.horizontal_resolution = 320           # Camera horizontal resolution
        self.vertical_resolution = 240            # Camera vertical resolution
        
        # Tracking parameters
        self.readings = np.array([])              # Array for moving average
        self.max_samples = 4                      # Maximum samples for moving average
        self.marker_ratio = 13                    # Ratio to switch between markers
        self.primary_marker_size = 17             # Size of primary marker
        self.secondary_marker_size = 8            # Size of secondary marker
        
        # Initialize vehicle connection
        try:
            print(f"Connecting to vehicle on: {connection_string}")
            self.vehicle = connect(connection_string, baud=baud_rate)
            print("Vehicle connected successfully")
        except Exception as e:
            print(f"Error connecting to vehicle: {e}")
            raise

    def send_landing_target(self, x: float, y: float, z: float) -> None:
        """
        Send landing target mavlink message for precision landing.
        
        Args:
            x: X-axis position
            y: Y-axis position
            z: Distance to target in meters
        
        Reference: http://mavlink.org/messages/common#LANDING_TARGET
        """
        try:
            # Calculate normalized target position
            normalized_x = (x - self.horizontal_resolution/2) * \
                         self.horizontal_fov/self.horizontal_resolution
            normalized_y = (y - self.vertical_resolution/2) * \
                         self.vertical_fov/self.vertical_resolution
            
            # Create and send mavlink message
            msg = self.vehicle.message_factory.landing_target_encode(
                0,                                    # time since system boot (not used)
                0,                                    # target num (not used)
                mavutil.mavlink.MAV_FRAME_BODY_NED,   # frame
                normalized_x,                         # x-axis angular offset
                normalized_y,                         # y-axis angular offset
                z,                                    # distance to target in meters
                0,                                    # Target x-axis size (radians)
                0                                     # Target y-axis size (radians)
            )

            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            
        except Exception as e:
            print(f"Error sending landing target message: {e}")

    def process_marker_position(self, h: float, v: float, z: float) -> Tuple[float, float]:
        """
        Process marker position data to get normalized x,y coordinates.
        
        Args:
            h: Horizontal position from camera
            v: Vertical position from camera
            z: Marker size/distance
            
        Returns:
            tuple: Processed (x, y) coordinates
        """
        x = int(h + 1650) / 10
        y = int(v + 750) / 6
        return x, y

    def update_moving_average(self, z: float) -> float:
        """
        Update moving average of marker size/distance readings.
        
        Args:
            z: New reading to add
            
        Returns:
            float: Current average
        """
        self.readings = np.append(self.readings, z)
        if len(self.readings) == self.max_samples:
            self.readings = np.delete(self.readings, 0)
        return np.mean(self.readings)

    def run(self) -> None:
        """
        Main control loop for precision landing.
        """
        print("Starting precision landing control loop...")
        
        try:
            while True:
                # Get marker detection from JeVois camera
                found, h, v, z = JeVois.balloon_xysize()
                
                if found != 0:
                    # Update moving average and get current average
                    current_avg = self.update_moving_average(z)
                    
                    # Process marker based on size and current average
                    if (current_avg > self.marker_ratio and 
                        z == self.primary_marker_size):
                        # Process primary marker
                        x, y = self.process_marker_position(h, v, z)
                        self.send_landing_target(x, y, z)
                        
                    elif (current_avg <= self.marker_ratio and 
                          z == self.secondary_marker_size):
                        # Process secondary marker
                        x, y = self.process_marker_position(h, v, z)
                        self.send_landing_target(x, y, z)
                        
                    # Optional debug output
                    # print(f"Marker detected - Size: {z}, Position: ({x}, {y})")
                
        except KeyboardInterrupt:
            print("\nPrecision landing controller stopped by user")
        except Exception as e:
            print(f"Error in precision landing control loop: {e}")
        finally:
            if self.vehicle:
                self.vehicle.close()
                print("Vehicle connection closed")

def main():
    """
    Main execution function.
    """
    try:
        # Initialize and run the precision landing controller
        controller = PrecisionLandingController()
        controller.run()
    except Exception as e:
        print(f"Error in main execution: {e}")

if __name__ == "__main__":
    main()
