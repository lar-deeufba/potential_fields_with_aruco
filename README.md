## Project - Artificial Potential Field with ArUco Marker

## Description

This branch was created to test speed control. According to the ur_modern_driver repository, velocity control reacts much faster than position control and is recommended for visual servoing. The next commands will show you what it takes for velocity control to work.

## Test Velocity Control (Usage)

The ur5.launch was modified to not start gazebo while velocity command is still being tested.

`roslaunch ur_gazebo ur5.launch`

For setting up the MoveIt! nodes to allow motion planning run:

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`

In order to start RViz with a configuration including the MoveIt! Motion Planning plugin run:

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

Start the ur modern drive to connect with URSIM (3.9.1 version) - Remember to set DHCP and check the ip in the terminal. The standard IP is 127.0.1.1 (you should start UR-SIM before launch ur5_ros_contro.launch)

`roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=127.0.1.1`

Before running the next command, check if joint_group_vel_controller is running by calling:

`rosservice call /controller_manager/list_controllers`

Start the command_vel node in order to check if velocity control is working properly (check arguments)

`rosrun custom_codes command_vel.py`

If you want to test velocity control with a dynamic goal published by a node (without Kinect), first run this node before command_vel.py and then run command_vel.py with --dyntest argument.

`rosrun custom_codes publish_dynamic_goal.py`

## Test Computer Vision Tools (Kinect)

Launch kinect driver using iai_kinect2 package
Please follow the instruction of iai_kinect2 installation in its default repository [https://github.com/code-iai/iai_kinect2]

`roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl reg_method:=cpu`

Launch ar_track_alvar

`roslaunch ar_track_alvar pr2_indiv_no_kinect_caio.launch`

Load the Kinect2 TF Frame

`roslaunch custom_codes tf_transforms.launch kinect2_test:=true`

Remember to run command_vel node with --armarker argument

## Connect with real UR5

Use the following command in order to connect with real UR5.
If you are using velocity control, do not use bring_up. Use ur5_ros_control instead.

`roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=192.168.131.12`

## How to use PC cam to track ar_tracker_alvar markers

Calibrate your PC CAM using this package

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Install kinetic image pipeline

`sudo apt-get install ros-kinetic-image-pipeline`

Launch the TF_Broadcaster to transform the camera_link frame relative to the base_link of UR5.
If you don't have a web cam available use the tf_transform launch with argument `fixed:=true` instead.

`roslaunch custom_codes tf_transforms.launch pc_camera_test:=true`

Launch the PC CAM node to publish camera_link frame
Change `video_stream_provider` argument to '1' if an external USB CAM is used.

`roslaunch custom_codes camera_pc.launch`

Launch the ar_track_alvar node below
Change the marker size (in centimeters) of the marker printed version in the launch file

`roslaunch custom_codes PC_CAM_alvar.launch`

### Info

In order see more info please go into the master branch
