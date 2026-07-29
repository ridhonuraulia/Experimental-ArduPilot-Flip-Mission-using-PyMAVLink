# Experimental-ArduPilot-Flip-Mission-using-PyMAVLink
how to using mode flip in otonomus drone in gazebo with plugin ardupilot
Disclaimer

This repository is provided for experimental and educational purposes only. The mission demonstrated here was developed and tested exclusively in a simulation environment. It is not intended for real aircraft operations without extensive validation, safety analysis, and compliance with all applicable regulations.

Parts of the implementation logic and documentation were developed with the assistance of AI and then reviewed and adapted for this project.


Overview

This repository demonstrates an experimental autonomous mission using PyMAVLink, ArduPilot, and Gazebo Harmonic.

The primary objective is to explore how MAVLink commands can be combined to perform a sequence of autonomous maneuvers, including takeoff, altitude changes, and aerial flips.

The project is intended as a reference for researchers, students, and developers interested in autonomous drone programming with ArduPilot.

Environment

The project was tested using the following environment:

Operating System: Ubuntu 22.04
Programming Language: Python
MAVLink Library: PyMAVLink
Simulator: Gazebo Harmonic
Flight Controller: ArduPilot
Gazebo Interface: ArduPilot Gazebo Plugin
Vehicle Model: iris_standoff
Mission Flow

The mission is divided into several phases.

Phase 0 – Initialization & Connection

The script establishes a MAVLink connection with the ArduPilot SITL running in Gazebo.

During initialization it:

Connects to the MAVLink endpoint.
Waits for a heartbeat message.
Verifies that communication with the flight controller is established before sending any commands.
Phase 1 – Arming & Takeoff

The autonomous mission begins with takeoff.

Steps:

Change the flight mode to GUIDED.
Arm the motors.
Send a MAV_CMD_NAV_TAKEOFF command with a target altitude of 20 meters.
Continuously monitor GLOBAL_POSITION_INT.
Continue only after the aircraft reaches approximately 20 meters Above Ground Level (AGL).
Phase 2 – Double Flip

ArduCopter's FLIP mode determines the flip direction from RC inputs.

Without any RC override, the default behavior is a Roll Flip (side flip).

Mission sequence:

Switch to FLIP mode.
Perform the first roll flip.
Wait approximately 3 seconds for stabilization.
Switch to FLIP mode again.
Perform the second roll flip.

After each flip, ArduPilot automatically exits FLIP mode and returns to a stabilized flight mode (typically LOITER or ALTHOLD, depending on configuration).

Phase 3 – Climb to 40 Meters

After completing the flips, the mission resumes autonomous navigation.

Steps:

Return to GUIDED mode.
Send a new position target using SET_POSITION_TARGET_GLOBAL_INT.
Command the vehicle to climb to 40 meters relative altitude.
Monitor telemetry until the target altitude is reached.

Since the vehicle is already airborne, a second TAKEOFF command is not used.

Phase 4 – Backflip

To perform a different flip direction, the script temporarily overrides the RC Pitch channel.

Steps:

Apply an RC Override on Channel 2 (Pitch) with a value of 1000.
Switch to FLIP mode.
ArduPilot interprets the pitch input and performs a Backflip.
Restore the RC channel to its neutral value (1500) after the maneuver.

This prevents the overridden RC input from affecting subsequent flight behavior.

Purpose

The goal of this project is to:

Explore autonomous flight using PyMAVLink.
Demonstrate MAVLink command sequencing.
Study ArduPilot flight modes.
Experiment with autonomous aerobatic maneuvers in simulation.
Provide an open reference implementation for the robotics and drone community.
