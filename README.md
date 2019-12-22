## Projet - Artificial Potential Field with ArUco Marker

## Usage
### To use UR5 with RVIZ + gazebo

To bring up the simulated robot in Gazebo, run:

`roslaunch ur_gazebo ur5.launch`

For setting up the MoveIt! nodes to allow motion planning run:

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`

In order to start RViz with a configuration including the MoveIt! Motion Planning plugin run:

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

Run the node corresponding to the file UR5_CPA_Gazebo.py
Use --h argument in order to get help

`rosrun custom_codes UR5_CPA_Gazebo.py`

## Required packages

- Moveit Kinetic [https://moveit.ros.org/install/]
- Robotiq Gripper [https://github.com/crigroup/robotiq]
- Universal Robot [https://github.com/ros-industrial/universal_robot]
- Ur_modern_driver [https://github.com/ros-industrial/ur_modern_driver/tree/kinetic-devel]

Install any dependencies you might have missed by using this command in catkin_ws folder
rosdep install --from-paths src --ignore-src -r -y

## Changes in universal_robot pkg

Please substitute the following files into the universal_robot package for compliance with Artificial Potential Field method

Substitute src/files_to_substitute/ur5.urdf.xacro into universal_robot/ur_description/urdf folder
Substitute src/files_to_substitute/ur5gripper_controllers.yaml into universal_robot/ur_gazebo/controller folder
Substitute src/files_to_substitute/ur5.launch into universal_robot/ur_gazebo/launch folder
Substitute src/files_to_substitute/controllers.yaml into universal_robot/ur5_moveit_config/config
Substitute src/files_to_substitute/ur5.srdf into universal_robot/ur5_moveit_config/config

## How to connect to the real UR5 robot

Firstly check the machine IP. The IP configured on the robot must have the last digit different.

`ifconfig`

Disable firewall

`sudo ufw disable`

Set up a static IP on UR5 according to the following figure

![IP Config UR5](https://github.com/caiobarrosv/Projeto_ICAR_2019/blob/master/imgs/config.jpg)

Set up a connection on Ubuntu according to the following figure

![config_ethernet2](https://user-images.githubusercontent.com/28100951/71323962-fe29f880-24b7-11ea-86dc-756729932de4.jpg)

Start ROS with RViz + Gazebo

Bring up - The configured ip must be the same as the robot

`roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=xxx.xxx.xxx.xx`
