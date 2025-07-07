
# TurtleBot3 Navigation using Custom Planner and Obstacle Avoidance

This project shows how to make a TurtleBot3 robot move through a set of waypoints in a simulated world. It combines simple path planning, motion control, and obstacle avoidance using ROS Noetic and Gazebo.


## Features

- Loads a list of waypoints from a file
- Tracks robot position using odometry (`/odom`)
- Moves the robot using a basic kinematic controller
- Avoids obstacles using LIDAR data and potential fields
- Switches between goal tracking and obstacle avoidance
- Runs entirely in simulation with TurtleBot3



## Project structure

turtlebot3_nav_student/

    Scripts/

    global_planner.py # Saves waypoints to follow

    kinematic_controller.py # Basic motion controller

    potential_fields.py # Obstacle avoidance using LIDAR

    navigator.py # Main node combining all modules

    waypoints.npy # Waypoint list (numpy array)


launch/

    navigation_demo.launch # starts all required notes 

rivz/

    nav.rviz # RViz visualization config

README.md
## Setup

    cd ~/catkin_ws
    catkin build
    source devel/setup.bash

    export TURTLEBOT3_MODEL=burger
## How to run

Step 1 launch

    roscore

step 2 launch # to launch turtlebot3 in gazebo

    roslaunch turtlebot3_gazebo turtlebot3_world.launch

Step 3 # generate path (waypoints)

    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

step 4 # run the navigator node 

    rosrun turtlebot3_nav_student navigator.py

     




## environments

with obstacles 

    roslaunch turtlebot3_gazebo turtlebot3_world.launch

    rosrun turtlebot3_nav_student global_planner.py

    rosrun turtlebot3_nav_student navigator.py

Without obstacles 

    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

    rosrun turtlebot3_nav_student global_planner_without_obstacle.py

    rosrun turtlebot3_nav_student navigator.py
Authors 

    Harsha vardhan reddy Vaddu
    Technische Hochschule Deggendorf
    Matriculation Number: 22311138
    harsha.vaddu@stud.th-deg.de


#Licence 

This project is for academic and educational purposes.
## Requirements

Ubuntu 20.04

ROS Noetic

TurtleBot3 packages

Python 3

install TurtleBot3
## Key compounents 

    A* Algorithm for Path Planning

    Real-Time Obstacle Detection and Avoidance Using LiDAR Sensors

    Kinematic Control Based on the Unicycle Motion Model

    Visualization and Console-Based Feedback via RViz and Terminal

    Capable of Navigating Both Obstacle-Free and Obstacle-Dense Environments

    Analysis of System Performance with Evaluation Metrics and Graphical      Representations
## Goals

>The robot successfully reached the target location in both scenarios.

>In the presence of obstacles: The robot dynamically detected, avoided them, and recalculated its path accordingly.

>In the absence of obstacles: The robot followed a smooth trajectory closely aligned with the originally planned path.