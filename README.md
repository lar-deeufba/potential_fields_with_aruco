## Project - Artificial Potential Field with ArUco Marker

## Description

This project is related to the application of the Adaptive Artificial Potential Field with ArUco Marker for grasping and collision avoidance using velocity control of ur_modern_driver.

Please check the paper for further info: [http://proceedings.science/p/111278]

## Required packages - Kinetic Version

- Moveit Kinetic [https://moveit.ros.org/install/]
- Robotiq Gripper [https://github.com/crigroup/robotiq]
- Universal Robot [https://github.com/ros-industrial/universal_robot]
- ur_modern_driver [https://github.com/ros-industrial/ur_modern_driver]

Install any dependencies you might have missed by using this command in catkin_ws folder
rosdep install --from-paths src --ignore-src -r -y

## Test Velocity Control with Gazebo

Gazebo simulation is used if the real robot is not available.

```
roslaunch custom_codes APF_project_gazebo.launch
```

Start the command_vel node in order to check if velocity control is working properly (check arguments).
The following command will turn on orientation control (if desired).

```
rosrun custom_codes command_vel_gazebo.py --armarker --OriON
```

## Test Velocity Control with RVIZ

RVIZ mode is used when the real robot is connected. In this case, Gazebo will not start.
Wait for the message "You can start planning now!" to show before running the next command.

```
roslaunch custom_codes APF_project_rviz.launch
```

! IMPORTANT ! - Remember to start URSIM before launching ur5_ros_control
Start the ur modern drive to connect with URSIM (3.9.1 version) - Remember to set DHCP and check the ip in the terminal. The IP is 127.0.1.1 as default.

```
roslaunch custom_codes ur5_ros_control.launch robot_ip:=127.0.1.1
```

If you don't have a webcam connected, launch this:

```
roslaunch custom_codes tf_transforms.launch
```

Start the command_vel node.
The following command will turn on orientation control (if desired).

```
rosrun custom_codes command_vel_rviz.py --armarker --OriON
```

### Optional commands (Related to Test Velocity Control)

If you want to test velocity control with a dynamic goal published by a node (without Kinect), first run this node before command_vel.py and then run command_vel.py with --dyntest argument.

```
rosrun custom_codes publish_dynamic_goal.py
```

If required, check if joint_group_vel_controller is running by calling:

```
rosservice call /controller_manager/list_controllers
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
## Sending commands through the action server

If you want to test the position controller, sending commands directly to the /'controller_command'/command topic use
the following:

```
rostopic pub -1 /pos_based_pos_traj_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
points:
  - positions: [1.57, 0, 0, 0, 0, 0]
    time_from_start: {secs: 1, nsecs: 0}"
```

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

## Contributors

This work was made possible by the teamwork of the LaR fellows:
* **Caio Viturino** - [caiobarrosv](https://github.com/caiobarrosv)
* **Henrique Poleselo** - [hpoleselo](https://github.com/hpoleselo)
* **Ubiratan de Melo** - [Ubira](https://github.com/Ubira)

Supervised by:
 * **Prof. Dr. André Gustavo Scolari Conceição** - [ascolari](https://github.com/ascolari)
