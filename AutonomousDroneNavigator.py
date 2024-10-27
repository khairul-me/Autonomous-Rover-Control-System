"""
Drone Autonomous Flight Control Script

This script allows for the autonomous control of a drone using the DroneKit library. 
It connects to the drone, arms it, and navigates it to predefined waypoints. 
Once the waypoints are reached, the drone returns to its home position using the 'Return to Launch' (RTL) mode.

Key Features:
1. Connects to the drone via a specified connection string.
2. Arms the drone and sets it to 'GUIDED' mode for waypoint navigation.
3. Flies the drone to three predefined waypoints, adjusting speed and monitoring progress.
4. Returns the drone to the launch point using 'RTL' (Return to Launch) mode after reaching all waypoints.

Dependencies:
- dronekit: For communication with the drone.
- time: For introducing pauses in the program for status checks.
- argparse: For passing the connection string as a command-line argument.
- math: For calculating distances between waypoints.
"""
########## DEPENDENCIES ##########

# Import necessary libraries from DroneKit for drone control
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse

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

# Function to calculate the distance in meters between two locations (target and current)
def get_distance_meters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat  # Latitude difference
    dLon = targetLocation.lon - currentLocation.lon  # Longitude difference

    # Calculate the Euclidean distance and convert it into meters
    return math.sqrt((dLon ** 2) + (dLat ** 2)) * 1.113195e5

# Function to command the vehicle to fly to a target location (waypoint)
def goto(targetLocation):
    # Get the initial distance to the target location
    distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)

    # Command the vehicle to fly to the target location
    vehicle.simple_goto(targetLocation)

    # Monitor the distance to the target location while in 'GUIDED' mode
    while vehicle.mode.name == "GUIDED":
        currentDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)

        # If the vehicle is close enough to the target (within 5% of the initial distance)
        if currentDistance < distanceToTargetLocation * 0.05:
            print("Reached target waypoint.")
            time.sleep(2)  # Wait a moment before breaking the loop
            break

        time.sleep(1)

    return None

########## MAIN EXECUTABLE ##########

# Define waypoints with latitude, longitude, and altitude (in meters)
wp1 = LocationGlobalRelative(36.00550, -95.86124, 10)
wp2 = LocationGlobalRelative(36.00607, -95.86107, 10)
wp3 = LocationGlobalRelative(36.00604, -95.86037, 10)

# Connect to the vehicle (drone)
vehicle = connectMyCopter()

# Set the drone's waypoint speed to 2 m/s
vehicle.parameters['WP_SPEED'] = 2

# Arm the vehicle and prepare for flight
arm()

# Fly to the specified waypoints
goto(wp1)
goto(wp2)
goto(wp3)

# Set the vehicle's mode to 'Return to Launch' (RTL) to make it return to the starting point
vehicle.mode = VehicleMode("RTL")

# Wait until the mode is confirmed as 'RTL'
while vehicle.mode != 'RTL':
    print("Waiting for drone to enter RTL flight mode")
    time.sleep(1)

print("Vehicle now in RTL mode. Returning to home position.")
