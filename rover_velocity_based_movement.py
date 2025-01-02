"""
Rover Velocity-Based Movement Control

This program implements velocity-based movement control for a rover using DroneKit.
It demonstrates both local NED (North-East-Down) and global NED velocity control,
allowing for precise movement and maneuverability.

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

# Third-party imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil

class RoverVelocityController:
    """
    A class to handle rover velocity-based movement operations.
    """
    
    def __init__(self, connection_string=None, baud_rate=57600):
        """
        Initialize RoverVelocityController with connection parameters.
        
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
            parser = argparse.ArgumentParser(description='Rover velocity commands')
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
        
        while not self.vehicle.is_armable:
            print("Waiting for rover to initialize...")
            time.sleep(1)
        print("Rover initialization complete")

        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print("Waiting for GUIDED mode...")
            time.sleep(1)
        print("Rover in GUIDED mode")

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)
        print("Rover armed successfully")
        
        return True

    def send_local_ned_velocity(self, vx, vy, vz):
        """
        Send velocity command in local NED frame.
        
        Args:
            vx (float): Velocity in x direction (forward/backward)
            vy (float): Velocity in y direction (left/right)
            vz (float): Velocity in z direction (up/down)
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

    def send_global_ned_velocity(self, vx, vy, vz):
        """
        Send velocity command in global NED frame.
        
        Args:
            vx (float): Velocity in North direction
            vy (float): Velocity in East direction
            vz (float): Velocity in Down direction
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

    def backup_manual(self):
        """
        Perform a manual backup maneuver using RC channel overrides.
        """
        print("Switching to MANUAL mode for backup maneuver...")
        self.vehicle.mode = VehicleMode("MANUAL")
        while self.vehicle.mode != 'MANUAL':
            print("Waiting for MANUAL mode...")
            time.sleep(1)

        # Apply reverse throttle
        self.vehicle.channels.overrides = {'2': 1400}
        time.sleep(1)
        
        # Reset throttle
        self.vehicle.channels.overrides = {'2': 1500}

        # Return to GUIDED mode
        print("Returning to GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print("Waiting for GUIDED mode...")
            time.sleep(1)

def main():
    """
    Main execution function that demonstrates various movement patterns.
    """
    # Initialize controller
    controller = RoverVelocityController()
    
    # Connect to rover
    if not controller.connect_rover():
        print("Failed to connect to rover. Exiting...")
        return
    
    # Arm the rover
    if not controller.arm_rover():
        print("Failed to arm rover. Exiting...")
        return

    try:
        # Demonstrate forward movement
        print("\nDemonstrating forward movement...")
        for _ in range(5):
            controller.send_local_ned_velocity(1, 0, 0)
            print("Moving forward at 1 m/s")
            time.sleep(1)

        # Demonstrate turning movements
        print("\nDemonstrating turning movements...")
        for _ in range(5):
            controller.send_local_ned_velocity(1, 1, 0)
            print("Turning right while moving forward")
            time.sleep(2)
            controller.send_local_ned_velocity(1, -1, 0)
            print("Turning left while moving forward")
            time.sleep(2)

        # Demonstrate cardinal direction movements
        print("\nDemonstrating cardinal direction movements...")
        for _ in range(5):
            controller.send_global_ned_velocity(1, 0, 0)
            print("Moving TRUE NORTH")
            time.sleep(3)
            controller.send_global_ned_velocity(-1, 0, 0)
            print("Moving TRUE SOUTH")
            time.sleep(3)
            controller.send_global_ned_velocity(0, 1, 0)
            print("Moving TRUE EAST")
            time.sleep(3)
            controller.send_global_ned_velocity(0, -1, 0)
            print("Moving TRUE WEST")
            time.sleep(3)

    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # Stop all movement
        controller.send_local_ned_velocity(0, 0, 0)
        if controller.vehicle:
            controller.vehicle.close()

if __name__ == "__main__":
    print("Starting Velocity-Based Movement Demo...")
    main()
    print("Demo completed")
