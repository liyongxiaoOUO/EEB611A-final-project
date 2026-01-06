## EEB611A-final-project
#Advanced AIoT Fire Patrol Robot 

  An autonomous patrol robot built with ROS Noetic and TurtleBot3. This system integrates LiDAR for obstacle avoidance and Computer Vision (OpenCV) for real-time fire detection. It features a Finite State Machine (FSM) for managing autonomous behaviors, including         simulated extinguishing, evidence logging, and remote interaction.

##Key Features

 1. Autonomous Patrol: Navigates the environment while avoiding obstacles using LiDAR sensor fusion.

 2. Fire Detection: Uses HSV color thresholding to detect thermal anomalies (red objects).
 
 3. Active Response: Performs a "wiggling" maneuver to simulate extinguishing upon detection.

 4. Auto-Evidence Logging: Automatically saves timestamped images and updates a mission log when a hazard is found.

 5. Smart Escape: Implements a remote "RESUME" command with a blind-turn maneuver to prevent re-detection loops.

#Prerequisites

 OS: Ubuntu 20.04 (LTS) / Windows Subsystem for Linux (WSL2)

 ROS Distro: Noetic Ninjemys

 Hardware/Sim: TurtleBot3 Waffle Pi (Gazebo Simulation)

 Python Dependencies:

  sudo apt-get install ros-noetic-cv-bridge ros-noetic-opencv-apps
  pip install opencv-python numpy


#Installation

  Clone the repository:

   cd ~/catkin_ws/src
   git clone <YOUR_GITHUB_REPO_URL_HERE>


(Note: Replace <YOUR_GITHUB_REPO_URL_HERE> with your actual link)

#Build the workspace:

  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash


#How to Run

 Launch Gazebo Simulation:

  export TURTLEBOT3_MODEL=waffle_pi
  roslaunch turtlebot3_gazebo turtlebot3_world.launch


 Run the Fire Patrol Node:

  # Make sure to source the environment first
  source ~/catkin_ws/devel/setup.bash

  # Run the main program
  rosrun fire_patrol fire_patrol.py


  Remote Control (Reset Robot):
  When the robot enters "WAITING" mode after extinguishing a fire, open a new terminal and send the following command to resume patrol:

  rostopic pub -1 /control_center std_msgs/String "data: 'RESUME'"


#Project Structure

  src/final_project_v5.py: Main logic node (FSM, Vision, Navigation).

  mission_log.txt: Log file recording detection events (generated automatically).

  evidence_*.jpg: Saved screenshots of detected hazards (generated automatically).
