"""
DroneKit Connection Test Program

This program establishes a connection with a drone/rover using DroneKit and performs basic
initialization and arming procedures. It serves as a template for testing drone connectivity
and basic functionalities.

Author: Md Khairul Islam
Institution: Hobart and William Smith Colleges, Geneva, NY
Major: Robotics and Computer Science
Contact: khairul.islam@hws.edu

Created: January 2025
Last Modified: January 2025

Dependencies:
- DroneKit
- time
- socket
- argparse
"""

# Standard library imports
import time
import socket
import math
import argparse

# Third-party imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException

def connect_my_copter():
    """
    Establishes connection with the drone using command line arguments.
    
    Returns:
        vehicle: DroneKit vehicle object representing the connected drone
    """
    parser = argparse.ArgumentParser(description='Drone connection commands')
    parser.add_argument('--connect', help='Connection string (e.g., /dev/ttyTHS1)')
    args = parser.parse_args()

    # Establish connection using provided connection string
    try:
        vehicle = connect(args.connect, baud=57600, wait_ready=True)
        print("Successfully connected to vehicle")
        return vehicle
    except Exception as e:
        print(f"Error connecting to vehicle: {e}")
        return None

def arm_vehicle(vehicle):
    """
    Arms the vehicle and sets it to GUIDED mode.
    
    Args:
        vehicle: DroneKit vehicle object
    
    Returns:
        bool: True if arming successful, False otherwise
    """
    # Wait for vehicle to become armable
    print("Basic pre-arm checks...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    print("Vehicle is now armable")

    # Switch to GUIDED mode
    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode...")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE")

    # Arm the vehicle
    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for vehicle to become armed...")
        time.sleep(1)
    print("Vehicle is now armed")
    
    return True

def main():
    """
    Main execution function that handles vehicle connection and arming.
    """
    # Connect to the vehicle
    vehicle = connect_my_copter()
    if not vehicle:
        print("Failed to connect to vehicle. Exiting...")
        return

    # Wait for autopilot to be ready
    vehicle.wait_ready('autopilot_version')
    print(f'Autopilot version: {vehicle.version}')

    # Arm the vehicle
    if arm_vehicle(vehicle):
        print("Vehicle setup completed successfully")
    else:
        print("Vehicle setup failed")

    return vehicle

if __name__ == "__main__":
    main()
