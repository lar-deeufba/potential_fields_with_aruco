## Project - Artificial Potential Field with ArUco Marker

## Description

This project is related to the application of the Adaptive Artificial Potential Field with ArUco Marker for grasping and collision avoidance using velocity control of ur_modern_driver.

Please check the paper for further info: [http://proceedings.science/p/111278]

## Required packages

- Moveit Kinetic [https://moveit.ros.org/install/]
- Robotiq Gripper [https://github.com/crigroup/robotiq]
- Universal Robot [https://github.com/ros-industrial/universal_robot]
- Ur_modern_driver [https://github.com/ros-industrial/ur_modern_driver/tree/kinetic-devel]

Install any dependencies you might have missed by using this command in catkin_ws folder
rosdep install --from-paths src --ignore-src -r -y

## Changes in universal_robot pkg

If you prefer to download Universal_robot package from its original repository please substitute the following files for compliance with the proposed Artificial Potential Field method

Substitute src/files_to_substitute/ur5.urdf.xacro into universal_robot/ur_description/urdf folder
Substitute src/files_to_substitute/ur5gripper_controllers.yaml into universal_robot/ur_gazebo/controller folder
Substitute src/files_to_substitute/ur5.launch into universal_robot/ur_gazebo/launch folder
Substitute src/files_to_substitute/controllers.yaml into universal_robot/ur5_moveit_config/config
Substitute src/files_to_substitute/ur5.srdf into universal_robot/ur5_moveit_config/config

## Test Velocity Control (Usage)

The ur5.launch was modified to not start gazebo while velocity command is still being tested.

```
roslaunch ur_gazebo ur5.launch
```

For setting up the MoveIt! nodes to allow motion planning run:

```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```

In order to start RViz with a configuration including the MoveIt! Motion Planning plugin run:

```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

! IMPORTANT ! - Remember to start URSIM before launching ur5_ros_control
Start the ur modern drive to connect with URSIM (3.9.1 version) - Remember to set DHCP and check the ip in the terminal. The IP is 127.0.1.1 as default.

```
roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=127.0.1.1
```

Before running the next command, check if joint_group_vel_controller is running by calling:

```
rosservice call /controller_manager/list_controllers
```

If you don't have the webcam connected please publish a fake ar_marker_0 frame for the robot to reach its pose.

```
roslaunch custom_codes tf_transform fixed:=true
```

Start the command_vel node in order to check if velocity control is working properly (check arguments).
The following command will turn on orientation control (if desired).

```
rosrun custom_codes command_vel.py --armarker --OriON
```

If you want to test velocity control with a dynamic goal published by a node (without Kinect), first run this node before command_vel.py and then run command_vel.py with --dyntest argument.

```
rosrun custom_codes publish_dynamic_goal.py
```

## Test Computer Vision Tools (Kinect)

Launch kinect driver using iai_kinect2 package
Please follow the instruction of iai_kinect2 installation in its default repository [https://github.com/code-iai/iai_kinect2]

```
roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl reg_method:=cpu
```

Launch ar_track_alvar

```
roslaunch ar_track_alvar pr2_indiv_no_kinect_caio.launch
```

Load the Kinect2 TF Frame

```
roslaunch custom_codes tf_transforms.launch kinect2_test:=true
```

Remember to run command_vel node with --armarker argument

## Connecting with real UR5

Firstly check the machine IP. The IP configured on the robot must have the last digit different.

`ifconfig`

Disable firewall

`sudo ufw disable`

Set up a static IP on UR5 according to the following figure

![config](https://user-images.githubusercontent.com/28100951/71323978-2ca7d380-24b8-11ea-954c-940b009cfd93.jpg)

Set up a connection on Ubuntu according to the following figure

![config_ethernet2](https://user-images.githubusercontent.com/28100951/71323962-fe29f880-24b7-11ea-86dc-756729932de4.jpg)

Use the following command in order to connect with real UR5.
If you are using velocity control, do not use bring_up. Use ur5_ros_control instead.

```
roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=192.168.131.12
```

## How to use PC cam to track ar_tracker_alvar markers

Calibrate your PC CAM using this package

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Install kinetic image pipeline

```
sudo apt-get install ros-kinetic-image-pipeline
```

Launch the TF_Broadcaster to transform the camera_link frame relative to the base_link of UR5.
If you don't have a web cam available use the tf_transform launch with argument `fixed:=true` instead.

```
roslaunch custom_codes tf_transforms.launch pc_camera_test:=true
```

Launch the PC CAM node to publish camera_link frame
Change `video_stream_provider` argument to '1' if an external USB CAM is used.

```
roslaunch custom_codes camera_pc.launch
```

Launch the ar_track_alvar node below
Change the marker size (in centimeters) of the marker printed version in the launch file

```
roslaunch custom_codes PC_CAM_alvar.launch
```

## Contributors

This work was made possible by the teamwork of the LaR fellows:
- Caio Viturino
- Henrique Poleselo
- Ubiratan de Melo

Supervised by:
 - Prof. Dr. André Gustavo Scolari Conceição
