#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

import rospy
import actionlib
import numpy as np
import rosservice

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class vel_control:
    def __init__(self):

        self.processing = False
        self.new_msg = False
        self.msg = None
        self.pub_vel = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray,  queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position, queue_size=10)

        self.client = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server (pos_based_pos_traj_controller)..."
        self.client.wait_for_server()
        print "Connected to server (pos_based_pos_traj_controller)"
        rospy.sleep(2)

        self.joint_vels = Float64MultiArray()
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                            'wrist_3_joint']



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

    def home_pos(self):
        home_pos = [3.14, -1.5707, 0, -1.5707, -1.5707, -1.5707]
        # First point is current position

        try:
            self.goal.trajectory.points.append(JointTrajectoryPoint(positions=home_pos, velocities=[0]*6, time_from_start=rospy.Duration(1.0)))
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

if __name__ == '__main__':
    try:
        rosservice.call_service('/controller_manager/switch_controller', [['pos_based_pos_traj_controller'], ['joint_group_vel_controller'], 1])
        # rospy.init_node('ur5_velocity_control')
        ur5_vel = vel_control()
        rospy.on_shutdown(ur5_vel.stop_robot)
        ur5_vel.home_pos()
        # rospy.sleep(3)

        rosservice.call_service('/controller_manager/switch_controller', [['joint_group_vel_controller'], ['pos_based_pos_traj_controller'], 1])
        ur5_vel.velocity_control()


    except rospy.ROSInterruptException:
	    rospy.loginfo("Program interrupted before completion")
