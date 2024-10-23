# Smart UGV (Unmanned Ground Vehicle) with ArduRover and Jetson Nano

Welcome to the **Smart UGV Project**, a guide to building and programming an autonomous ground vehicle using the ArduRover firmware, Pixhawk flight controller, and Jetson Nano as a companion computer. This project enables autonomous driving, machine learning, and AI-based control for your Unmanned Ground Vehicle (UGV).

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Setup](#software-setup)
- [Building the UGV](#building-the-ugv)
- [Python Control](#python-control)
- [Autonomous Mission Example](#autonomous-mission-example)
- [License](#license)

---

## Project Overview

This project is based on the **ArduRover** platform and aims to build a smart, autonomous UGV. The vehicle is powered by a Pixhawk flight controller running ArduPilot's **ArduRover** firmware, paired with a **Jetson Nano** companion computer to enable AI-based tasks like computer vision, machine learning, and autonomous navigation. With the setup described here, you can control the UGV using Python scripts, allowing it to perform various autonomous missions such as waypoint navigation, obstacle avoidance, and more.

## Features

The Smart UGV comes equipped with the following capabilities:

- **Autonomous Driving**: Drive autonomously using waypoint navigation or velocity control.
- **AI Integration**: Use computer vision (OpenCV) and machine learning algorithms for smarter control.
- **Python Scripting**: Control the UGV using Python for flexibility in defining custom missions.
- **Real-time Telemetry**: Monitor and control the UGV with live data from the Pixhawk flight controller.
- **Manual Override**: Switch between autonomous and manual driving modes using an RC transmitter.

## Hardware Requirements

To build this Smart UGV, you will need the following components:

1. **1/10th Scale RC Crawler**: A conventional steering chassis to mount all the electronics.
2. **Pixhawk Flight Controller**: The brain that runs the ArduRover firmware.
3. **Jetson Nano**: A powerful companion computer for running AI and computer vision algorithms.
4. **Brushed Motor ESC**: Controls the motor speed.
5. **Steering Servo**: For turning the wheels.
6. **GPS Module**: For position tracking and navigation.
7. **Telemetry Module**: For real-time communication between the UGV and a ground station.
8. **RC Receiver and Transmitter**: For manual control of the UGV.
9. **Power Module**: To supply power to the electronics.
10. **Lipo Battery**: Provides power to the entire system.

Optional hardware:
- **Webcam**: For computer vision tasks.
- **Delivery Servo**: If you want to add payload delivery capabilities.

## Software Setup

### 1. ArduRover Firmware
You need to flash the Pixhawk with the **ArduRover** firmware. This open-source firmware provides the essential functionality for autonomous vehicle control, including:

- Sensor integration
- Position estimation
- Motor control

More information on setting up ArduRover can be found on the [ArduPilot Rover Documentation](https://ardupilot.org/rover/).

### 2. Jetson Nano Configuration
The Jetson Nano will act as the companion computer to run higher-level AI tasks. You will need to set up the following dependencies on the Jetson Nano:

- **Python**: For scripting control logic.
- **MAVLink**: For communication between the Jetson Nano and Pixhawk.
- **OpenCV**: For computer vision tasks.
- **TensorFlow/PyTorch**: For running machine learning models (optional).

Once configured, the Jetson Nano can send movement commands to the Pixhawk over a UART connection.

### 3. Ground Control Software (Mission Planner/QGroundControl)
To manually control the rover or set autonomous missions, you can use **Mission Planner** or **QGroundControl**, which provide telemetry, mission planning, and real-time vehicle control.

## Building the UGV

Follow these steps to assemble the UGV:

1. **Chassis Setup**: Mount the Pixhawk, Jetson Nano, GPS, telemetry module, and battery to the 1/10th scale RC crawler.
2. **Motor and Servo Connections**: Connect the motor to the ESC and the steering servo to the Pixhawk.
3. **Power Supply**: Wire the power module to the Pixhawk and the Jetson Nano.
4. **Telemetry and GPS Setup**: Install the telemetry module and GPS for real-time monitoring and autonomous navigation.
5. **Testing**: Before switching to autonomous modes, perform manual drive tests to ensure proper motor and steering functionality.

## Python Control

The Jetson Nano allows Python-based control of the UGV. There are two main types of movement control:

- **Location-Based Movement**: Send the UGV to a specific waypoint by providing GPS coordinates.
- **Velocity-Based Movement**: Control the rover's speed and direction without a specific destination.

Example Python control code can be found in the `scripts/` directory, where you will find scripts for controlling the UGV using both methods.

### Example: Waypoint Navigation
```python
# Example Python script to send the UGV to a GPS waypoint
import mavlink
import time

# Establish connection to Pixhawk
vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=921600)

# Set the waypoint coordinates
waypoint_lat = 40.748817
waypoint_lon = -73.985428

# Send the rover to the waypoint
vehicle.simple_goto(LocationGlobalRelative(waypoint_lat, waypoint_lon, 0))
