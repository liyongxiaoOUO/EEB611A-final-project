## EEB611A-final-project
##Advanced AIoT Fire Patrol Robot

An autonomous patrol robot system built with ROS Noetic and TurtleBot3. This project integrates LiDAR for obstacle avoidance and Computer Vision (OpenCV) for real-time fire detection. It features a Finite State Machine (FSM) to manage autonomous behaviors, including simulated extinguishing, automated evidence logging, and remote operator interaction.

Key Features

Autonomous Patrol: Navigates the environment while avoiding obstacles using LiDAR sensor fusion logic.

Fire Detection: Utilizes HSV color thresholding to detect thermal anomalies (simulated by red objects).

Active Response: Performs a "wiggling" maneuver to simulate an extinguishing action upon detection.

Auto-Evidence Logging: Automatically saves timestamped screenshots and updates a mission log file when a hazard is detected.

Smart Escape Mechanism: Implements a remote "RESUME" command that triggers a blind-turn maneuver to prevent re-detection loops.

Prerequisites

Operating System: Ubuntu 20.04 (LTS) or Windows Subsystem for Linux (WSL2)

ROS Distribution: Noetic Ninjemys

Simulation Platform: Gazebo

Hardware Model: TurtleBot3 Waffle Pi

Python Dependencies:

sudo apt-get install ros-noetic-cv-bridge ros-noetic-opencv-apps
pip install opencv-python numpy


Installation

Clone the repository:

cd ~/catkin_ws/src
git clone <YOUR_GITHUB_REPO_URL_HERE>


(Note: Replace <YOUR_GITHUB_REPO_URL_HERE> with your actual repository link)

Build the workspace:

cd ~/catkin_ws
catkin_make
source devel/setup.bash


How to Run

1. Launch Simulation Environment

Start Gazebo with the Waffle Pi model (required for the camera):

export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch


2. Run the Fire Patrol Node

Open a new terminal and execute the main program:

source ~/catkin_ws/devel/setup.bash
rosrun fire_patrol final_project_v5.py


3. Remote Control (Reset & Escape)

When the robot extinguishes a fire and enters "WAITING" mode, you can command it to resume patrolling. Open a new terminal and send:

rostopic pub -1 /control_center std_msgs/String "data: 'RESUME'"


The robot will perform a 180-degree turn to avoid the hazard and continue its patrol.

Project Structure

src/final_project_v5.py: The main ROS node containing FSM, Vision, and Navigation logic.

mission_log.txt: Automatically generated log file recording detection events.

evidence_*.jpg: Automatically saved screenshots of detected hazards.
