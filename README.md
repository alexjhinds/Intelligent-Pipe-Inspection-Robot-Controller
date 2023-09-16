# Intelligent-Pipe-Inspection-Robot-Controller

This repository holds a set of ROS packages that implement a simulated environment for a pipe inspection robot, and provide an intelligent manual controller to allow and operator to control the robot more easily. 

There are three packages: 
1. pipe_inspect_robot_control: This package contains a set of nodes which implement the intelligent controller
2. pipe_inspect_robot_gazebo: This package contains a set of nodes, and Gazebo files needed to simualte the pipe inspection robot in a 3D environment
3. pipe_inspect_robot_description: This package contains the URDF description file and models for the pipe inspection robot platform

A demonstration of what this package implements can be found in the video demo folder. 

# Installing the packages
To install the packages, a ROS environment needs to be set up. 

1. Install ROS Noetic (http://wiki.ros.org/noetic/Installation). Note: A Linux environment is needed. 
2. Create a workspace in ROS
3. Download these packages and place into workspace's src folder. 
4. Run the setup script. This script installs necessary packages for the controller to work, and sets up the Joy node to use a physical controller.
From the workspace src folder: 

`sh install_dependancies_pipe_inspect_robot_controller.sh`

5. Ensure that all the Python Scripts can be run as executable. 
	Navigate to the src folder of each package and type `chmod+x main_robot_controller.py` replacing the python file as necessary. 

6. Build the package:
Navigate to the catkin_ws base directory.
run `catkin_make` in the terminal
If successful the packages should build correctly. 

#Running the simulator and controller
1. Plug in a joystick controller via USB

2. Ensure that you source your setup file in a new terminal in the catkin_ws directory: 
`source devel/setup.bash`

3. Run the simulator by launching the node: 
`roslaunch pipe_inspect_robot_control_demo_world.launch`

4. The simulation is paused on startup by default. In the gazebo simulation window, press the "play" button to start the simulation. 

5. Control the robot
The left joystick controls the robot's axial speed, and the right joystick control the robot's rotational speed. 
Change the chosen pipe direction by pressing the left and right shoulder buttons
The Rviz window shows the GUI

Email if there are any issues setting up. 
