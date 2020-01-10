## Projet - Artificial Potential Field with ArUco Marker

## Usage
### This branch was created to test speed control. According to the ur_modern_driver repository, velocity control reacts much faster than position control and is recommended for visual servoing. The next commands will show you what it takes for velocity control to work.

The ur5.launch was modified to not start gazebo while velocity command is still being tested.

`roslaunch ur_gazebo ur5.launch`

For setting up the MoveIt! nodes to allow motion planning run:

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`

In order to start RViz with a configuration including the MoveIt! Motion Planning plugin run:

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

Start the ur modern drive with URSIM (3.9.1 version) - Remember to set DHCP and check the ip in the terminal. The standard IP is 127.0.1.1

`roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=127.0.1.1`

Before running the next command, check if joint_group_vel_controller is running by calling:

`rosservice call /controller_manager/list_controllers`

Start the command_vel node in order to check if velocity control is working properly.

`rosrun custom_codes command_vel.py`


## Info

In order see more info please go into the master branch
