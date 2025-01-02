"""
DroneKit Control Template

This template provides a foundation for developing DroneKit-based drone/rover control programs.
It includes basic functionality for connection, arming, and movement control using both
position-based and velocity-based commands.

Author: Md Khairul Islam
Institution: Hobart and William Smith Colleges, Geneva, NY
Major: Robotics and Computer Science
Contact: khairul.islam@hws.edu

Created: January 2025
Last Modified: January 2025

Dependencies:
- DroneKit
- pymavlink
- time
- argparse
"""

# Standard library imports
import time
import socket
import math
import argparse
from typing import Optional, Tuple

# Third-party imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil

class DroneController:
    """
    A comprehensive drone/rover control class that combines both position-based
    and velocity-based movement capabilities.
    """
    
    def __init__(self, connection_string: Optional[str] = None, baud_rate: int = 57600):
        """
        Initialize DroneController with connection parameters.
        
        Args:
            connection_string: Vehicle connection string
            baud_rate: Connection baud rate
        """
        self.vehicle = None
        self.connection_string = connection_string
        self.baud_rate = baud_rate

    def connect_vehicle(self) -> bool:
        """
        Establish connection with the vehicle using provided parameters.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        if not self.connection_string:
            parser = argparse.ArgumentParser(description='Vehicle control commands')
            parser.add_argument('--connect', required=True, help='Vehicle connection string')
            args = parser.parse_args()
            self.connection_string = args.connect

        try:
            print(f"Connecting to vehicle on: {self.connection_string}")
            self.vehicle = connect(self.connection_string, 
                                 baud=self.baud_rate,
                                 wait_ready=True)
            print("Successfully connected to vehicle")
            return True
        except Exception as e:
            print(f"Error connecting to vehicle: {e}")
            return False

    def arm_vehicle(self) -> bool:
        """
        Arm the vehicle and set it to GUIDED mode.
        
        Returns:
            bool: True if arming successful, False otherwise
        """
        try:
            print("Beginning pre-arm checks...")
            
            # Wait for vehicle to become armable
            while not self.vehicle.is_armable:
                print("Waiting for vehicle to initialize...")
                time.sleep(1)
            print("Vehicle initialization complete")

            # Switch to GUIDED mode
            self.vehicle.mode = VehicleMode("GUIDED")
            while self.vehicle.mode != 'GUIDED':
                print("Waiting for GUIDED mode...")
                time.sleep(1)
            print("Vehicle in GUIDED mode")

            # Arm the vehicle
            self.vehicle.armed = True
            while not self.vehicle.armed:
                print("Waiting for arming...")
                time.sleep(1)
            print("Vehicle armed successfully")
            
            return True
            
        except Exception as e:
            print(f"Error during arming sequence: {e}")
            return False

    def get_distance_meters(self, target_location: LocationGlobalRelative) -> float:
        """
        Calculate the ground distance between two locations.
        
        Args:
            target_location: Target LocationGlobalRelative object
            
        Returns:
            float: Distance in meters
        """
        current_location = self.vehicle.location.global_relative_frame
        
        d_lat = target_location.lat - current_location.lat
        d_lon = target_location.lon - current_location.lon
        
        return math.sqrt((d_lon * d_lon) + (d_lat * d_lat)) * 1.113195e5

    def goto_location(self, target_location: LocationGlobalRelative, 
                     tolerance: float = 0.01) -> bool:
        """
        Navigate to specified GPS location.
        
        Args:
            target_location: Target LocationGlobalRelative object
            tolerance: Distance tolerance as fraction of total distance
            
        Returns:
            bool: True if destination reached, False otherwise
        """
        try:
            initial_distance = self.get_distance_meters(target_location)
            print(f"Navigating to Location: Lat={target_location.lat}, Lon={target_location.lon}")
            
            self.vehicle.simple_goto(target_location)
            
            while self.vehicle.mode.name == "GUIDED":
                current_distance = self.get_distance_meters(target_location)
                if current_distance < initial_distance * tolerance:
                    print("Reached target location")
                    time.sleep(2)
                    return True
                time.sleep(1)
                
            return False
            
        except Exception as e:
            print(f"Error during navigation: {e}")
            return False

    def send_local_ned_velocity(self, vx: float, vy: float, vz: float) -> None:
        """
        Send velocity command in local NED frame.
        
        Args:
            vx: Velocity in x direction (forward/backward)
            vy: Velocity in y direction (left/right)
            vz: Velocity in z direction (up/down)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)  # yaw, yaw_rate
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_global_ned_velocity(self, vx: float, vy: float, vz: float) -> None:
        """
        Send velocity command in global NED frame.
        
        Args:
            vx: Velocity in North direction
            vy: Velocity in East direction
            vz: Velocity in Down direction
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)  # yaw, yaw_rate
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def backup_manual(self, duration: float = 1.0) -> None:
        """
        Perform a manual backup maneuver.
        
        Args:
            duration: Duration of backup maneuver in seconds
        """
        print("Initiating manual backup maneuver...")
        self.vehicle.mode = VehicleMode("MANUAL")
        while self.vehicle.mode != 'MANUAL':
            print("Waiting for MANUAL mode...")
            time.sleep(1)

        # Apply reverse throttle
        self.vehicle.channels.overrides = {'2': 1400}
        time.sleep(duration)
        
        # Reset throttle
        self.vehicle.channels.overrides = {'2': 1500}

        # Return to GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print("Waiting for GUIDED mode...")
            time.sleep(1)

    def cleanup(self) -> None:
        """
        Clean up vehicle connection and resources.
        """
        if self.vehicle:
            print("Closing vehicle connection...")
            self.vehicle.close()
            print("Vehicle connection closed")

def main():
    """
    Main execution function demonstrating basic usage of the DroneController class.
    """
    controller = DroneController()
    
    try:
        # Connect to vehicle
        if not controller.connect_vehicle():
            print("Failed to connect to vehicle. Exiting...")
            return

        # Arm the vehicle
        if not controller.arm_vehicle():
            print("Failed to arm vehicle. Exiting...")
            return

        print("Vehicle ready for operation")
        
        # Add your control code here
        
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()
