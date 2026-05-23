# MicroSpot: ROS 2 Quadruped Robot (Work in Progress)

**Status:** Active Development   

![MicroSpot Walking Test](assests/microspotai_walking.gif)

## What is this?
This is a custom quadruped robot I am building from scratch using ROS 2 inspired from MicroSpotAi. The goal is to bridge the gap between high-level math (Inverse Kinematics) and low-level physical hardware control. 

I've been working on this for about a month. The core walking mechanics and hardware communication are done, and I'm currently prepping the chassis to add perception sensors for autonomous navigation.

## Current State of the Project

Right now, the robot can calculate its own leg trajectories and move physical motors to match. Here is what is up and running:

* **Simulation & Physics:** Built a custom URDF model for Gazebo/RViz. 
[MicroSpot Walking Test in Rviz](assests/spot_rviz_sim.gif)

[MicroSpot Walking Test in Gazebo](assests/spot_gazebo_sim.gif)
* **Inverse Kinematics (IK):** Wrote a custom Python solver (`leg_ik_solver.py`) that takes 3D target coordinates and calculates the exact joint angles needed for the hip, thigh, and knee using law of cosines and some basic trigonometry.
* **Hardware Bridge:** Created a custom ROS 2 node that listens to the calculated joint angles and translates them into actual PWM signals for the physical servos via an I2C board. It handles all the mechanical zero-offsets and mirrored motor directions.

## What's Next? (Roadmap)

I am transitioning this from a blind walking robot into something that can map a room and avoid obstacles. My next targets are:

- [ ] Mount a Camera and LiDAR scanner to publish `/scan` and `/camera/image_raw` data.
- [ ] Set up `slam_toolbox` to generate a 2D map of my room.
- [ ] Integrate the ROS 2 Nav2 stack so the robot can walk to a target coordinate without running into walls.
- [ ] Add an IMU so the robot can actively balance itself if it gets pushed.

## Running the Simulation Locally

If you want to poke around the math and see the RViz simulation:

```bash
# Create a new directory 
mkdir ~/spot_ws
cd ~/spot_ws

# Clone the repo
git clone [https://github.com/HakimHayate/microspotai.git](https://github.com/HakimHayate/microspotai.git)
mv microspotai/* .
rm -rf microspotai

# Build the workspace
cd ~/spot_ws
colcon build --symlink-install
source install/setup.bash

# Launch the RViz display
First terminal run: ros2 launch microspot_description display.launch.py
Second terminal run: ros2 run spot_controller walking_gait

# Launch gazebo
In a third therminal run: ros2 launch spot_bringup spot.launch.py