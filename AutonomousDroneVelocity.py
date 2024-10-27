"""
Drone Velocity and Navigation Control Script

This script provides autonomous control of a drone's movement using both local and global NED (North-East-Down) reference frames.
It connects to a drone using the DroneKit library, arms it, and commands the drone to perform various velocity-based movements.
These movements include:
1. Moving forward, turning, and navigating in specific directions using the local NED frame (relative to the drone).
2. Navigating in global directions such as True North, South, East, and West using the global NED frame (fixed reference).
3. Reversing the drone using manual overrides, allowing for backup without relying on GPS.

The code demonstrates how to use MAVLink commands for velocity control and can be extended for more complex autonomous missions.

Dependencies:
- dronekit: For communication with the drone.
- pymavlink: For MAVLink message generation.
- argparse: For passing the connection string as a command-line argument.
"""
########## DEPENDENCIES ##########

# Import necessary libraries from DroneKit for drone control
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse
from pymavlink import mavutil

########## FUNCTIONS ##########

# Function to connect to the drone (vehicle)
def connectMyCopter():
    # Argument parser for the connection string
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')  # Add a connection argument (e.g., the vehicle's connection string)
    args = parser.parse_args()

    # Get the connection string from the arguments
    connection_string = args.connect

    # Connect to the vehicle using the connection string
    vehicle = connect(connection_string, baud=57600, wait_ready=True)

    # Return the connected vehicle object
    return vehicle

# Function to arm the vehicle and set its flight mode to GUIDED
def arm():
    # Wait for the vehicle to become armable (i.e., ready to be armed)
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")

    # Set the vehicle's mode to 'GUIDED'
    vehicle.mode = VehicleMode("GUIDED")

    # Wait until the mode has changed to GUIDED
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    # Arm the vehicle
    vehicle.armed = True

    # Wait for the vehicle to be fully armed
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Vehicle is now armed.")

    return None

# Function to send velocity commands in the drone's local NED (North-East-Down) frame
# +x is the forward direction, +y is to the right, and +z is downward
def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # NED frame relative to the drone's body
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocities (in m/s)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to reverse the direction of the drone
def reverse(direction):
    # Create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_REVERSE,  # MAV command to reverse
        0,  # confirmation
        direction,  # Param 1, 0 for forward, 1 for backward
        0,  # Param 2, yaw speed deg/s (not used)
        0,  # Param 3, direction (not used here)
        0,  # Param 4, relative offset 1, absolute angle 0
        0, 0, 0  # Params 5-7 not used
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to send velocity commands in the global NED frame (fixed reference frame)
def send_global_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Global NED frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocities in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet)
        0, 0  # yaw, yaw_rate (not supported yet)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to switch to manual mode and override control to reverse the drone
def backup():
    # Set the vehicle mode to MANUAL
    vehicle.mode = VehicleMode("MANUAL")
    while vehicle.mode != 'MANUAL':
        print("Waiting for drone to enter MANUAL flight mode")
        time.sleep(1)

    # Override the throttle channel to reverse the drone slightly
    vehicle.channels.overrides = {'2': 1400}
    time.sleep(1)
    
    # Reset the channel override to stop the movement
    vehicle.channels.overrides = {'2': 1500}

    # Set back to GUIDED mode after reversing
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)

########## MAIN EXECUTABLE ##########

# Connect to the drone
vehicle = connectMyCopter()

# Arm the drone and set it to GUIDED mode
arm()

# Move the drone forward at 1 m/s for 5 seconds using local NED frame
counter = 0
while counter < 5:
    send_local_ned_velocity(1, 0, 0)  # Move forward with 1 m/s
    print("Moving forward at 1 m/s with local NED")
    time.sleep(1)
    counter += 1

# Turn the drone right and left using local NED frame
counter = 0
while counter < 5:
    send_local_ned_velocity(1, 1, 0)  # Move forward-right
    print("Turning to the right")
    time.sleep(2)
    
    send_local_ned_velocity(1, -1, 0)  # Move forward-left
    print("Turning to the left")
    time.sleep(2)
    
    counter += 1

# Move the drone in global directions (NORTH, SOUTH, EAST, WEST) using global NED frame
counter = 0
while counter < 5:
    send_global_ned_velocity(1, 0, 0)  # Move north
    print("Moving TRUE NORTH")
    time.sleep(3)
    
    send_global_ned_velocity(-1, 0, 0)  # Move south
    print("MOVING TRUE SOUTH")
    time.sleep(3)
    
    send_global_ned_velocity(0, 1, 0)  # Move east
    print("MOVING TRUE EAST")
    time.sleep(3)
    
    send_global_ned_velocity(0, -1, 0)  # Move west
    print("MOVING TRUE WEST")
    time.sleep(3)
    
    counter += 5
