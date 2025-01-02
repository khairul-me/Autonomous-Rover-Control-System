"""
Rover Location-Based Movement Control

This program implements location-based movement control for a rover using GPS coordinates.
It allows the rover to navigate through predefined waypoints using DroneKit's simple_goto
functionality.

Author: Md Khairul Islam
Institution: Hobart and William Smith Colleges, Geneva, NY
Major: Robotics and Computer Science
Contact: khairul.islam@hws.edu

Created: January 2025
Last Modified: January 2025

Dependencies:
- DroneKit
- time
- argparse
"""

# Standard library imports
import time
import socket
import math
import argparse

# Third-party imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException

# Configuration
DEFAULT_WAYPOINTS = [
    LocationGlobalRelative(36.00550, -95.86124, 10),  # Waypoint 1
    LocationGlobalRelative(36.00607, -95.86107, 10),  # Waypoint 2
    LocationGlobalRelative(36.00604, -95.86037, 10)   # Waypoint 3
]

class RoverController:
    """
    A class to handle rover control and navigation operations.
    """
    
    def __init__(self, connection_string=None, baud_rate=57600):
        """
        Initialize RoverController with connection parameters.
        
        Args:
            connection_string (str): Vehicle connection string
            baud_rate (int): Connection baud rate
        """
        self.vehicle = None
        self.connection_string = connection_string
        self.baud_rate = baud_rate

    def connect_rover(self):
        """
        Establish connection with the rover using provided parameters.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        if not self.connection_string:
            parser = argparse.ArgumentParser(description='Rover movement commands')
            parser.add_argument('--connect', help='Vehicle connection string')
            args = parser.parse_args()
            self.connection_string = args.connect

        try:
            print(f"Connecting to rover on: {self.connection_string}")
            self.vehicle = connect(self.connection_string, 
                                 baud=self.baud_rate,
                                 wait_ready=True)
            print("Successfully connected to rover")
            return True
        except Exception as e:
            print(f"Error connecting to rover: {e}")
            return False

    def arm_rover(self):
        """
        Arm the rover and set it to GUIDED mode.
        
        Returns:
            bool: True if arming successful, False otherwise
        """
        print("Beginning rover arming sequence...")
        
        # Wait for vehicle to become armable
        while not self.vehicle.is_armable:
            print("Waiting for rover to initialize...")
            time.sleep(1)
        print("Rover initialization complete")

        # Switch to GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print("Waiting for GUIDED mode...")
            time.sleep(1)
        print("Rover in GUIDED mode")

        # Arm the rover
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)
        print("Rover armed successfully")
        
        return True

    def get_distance_meters(self, target_location):
        """
        Calculate the distance between current location and target location.
        
        Args:
            target_location: LocationGlobalRelative object representing target position
            
        Returns:
            float: Distance in meters
        """
        current_location = self.vehicle.location.global_relative_frame
        
        d_lat = target_location.lat - current_location.lat
        d_lon = target_location.lon - current_location.lon
        
        return math.sqrt((d_lon * d_lon) + (d_lat * d_lat)) * 1.113195e5

    def goto_location(self, target_location, tolerance=0.05):
        """
        Navigate to specified GPS location.
        
        Args:
            target_location: LocationGlobalRelative object
            tolerance (float): Distance tolerance as fraction of total distance
            
        Returns:
            bool: True if destination reached, False otherwise
        """
        initial_distance = self.get_distance_meters(target_location)
        print(f"Navigating to Location: Lat={target_location.lat}, Lon={target_location.lon}")
        
        self.vehicle.simple_goto(target_location)
        
        while self.vehicle.mode.name == "GUIDED":
            current_distance = self.get_distance_meters(target_location)
            if current_distance < initial_distance * tolerance:
                print("Reached target waypoint")
                time.sleep(2)
                return True
            time.sleep(1)
            
        return False

    def return_to_launch(self):
        """
        Command the rover to return to launch location.
        """
        print("Initiating return to launch...")
        self.vehicle.mode = VehicleMode("RTL")
        
        while self.vehicle.mode != 'RTL':
            print("Waiting for RTL mode...")
            time.sleep(1)
        print("Returning to launch point")

def main():
    """
    Main execution function that handles the rover's waypoint navigation sequence.
    """
    # Initialize rover controller
    controller = RoverController()
    
    # Connect to rover
    if not controller.connect_rover():
        print("Failed to connect to rover. Exiting...")
        return
    
    # Set movement speed
    controller.vehicle.parameters['WP_SPEED'] = 2
    
    # Arm the rover
    if not controller.arm_rover():
        print("Failed to arm rover. Exiting...")
        return
    
    # Navigate through waypoints
    try:
        for i, waypoint in enumerate(DEFAULT_WAYPOINTS, 1):
            print(f"\nNavigating to waypoint {i}...")
            if not controller.goto_location(waypoint):
                print(f"Failed to reach waypoint {i}")
                break
        
        # Return to launch point
        controller.return_to_launch()
        
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
        controller.return_to_launch()
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        controller.return_to_launch()
    finally:
        if controller.vehicle:
            controller.vehicle.close()

if __name__ == "__main__":
    main()
