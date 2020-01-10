#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class vel_control:
    def __init__(self):
        rospy.init_node('ur5_velocity_control')

        self.pub_vel = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray,  queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position, queue_size=10)

        self.joint_vels = Float64MultiArray()

        self.processing = False
        self.new_msg = False
        self.msg = None

    def ur5_actual_position(self, joint_values_from_ur5):
        # faz a leitura da ultima posicao do robo
        # pode fazer a leitura da ultima posicao do frame
        if not self.processing:
            self.new_msg = True
            # self.actual_position = joint_values_from_ur5.actual.positions
            self.actual_position = joint_values_from_ur5.position

    def stop_robot(self):
        # Set zero velocity in order to keep the robot in last calculated position
        rospy.loginfo("Shutdown time")
        self.joint_vels.data = np.array([0.0, 0.0, 0, 0, 0, 0])
        self.pub_vel.publish(self.joint_vels)

    def velocity_control(self):
        # publishing rate for velocity control
        rate = rospy.Rate(125)

        while not rospy.is_shutdown():
            self.joint_vels.data = np.array([0.0, 0.0, 0.5, 0, 0, 0])
            self.pub_vel.publish(self.joint_vels)
            rospy.sleep(1)

            if self.new_msg:
                #set processing to True
                self.processing = True
                self.new_msg = False
                #simulate a process that take 0.2 seconds
                rospy.loginfo(self.actual_position)
                rate.sleep()
                #set processing to False
                self.processing = False

if __name__ == '__main__':
    try:
        ur5_vel = vel_control()
        rospy.on_shutdown(ur5_vel.stop_robot)
        ur5_vel.velocity_control()

    except rospy.ROSInterruptException:
	    rospy.loginfo("Program interrupted before completion")
