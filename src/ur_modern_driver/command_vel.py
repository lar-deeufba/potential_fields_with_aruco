#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

vel = [[0.0, 0.05, 0.0, 0.0, 0.0, 0.0],]

def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('joint_group_vel_controller/command',
                          Float64MultiArray,
                          queue_size=10)

    joint_vels = Float64MultiArray()

    # Create the topic message
    # traj = JointTrajectory()
    # traj.header = Header()
    # Joint names for UR5
    # traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        # 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        # 'wrist_3_joint']

    # publishing rate
    rate = rospy.Rate(125)

    # default message for joint_group_vel_controller topic
    joint_vels = Float64MultiArray()
    joint_vels.data = np.array([0.5,0.0,0,0,0,0])

    joint_vels2 = Float64MultiArray()
    joint_vels2.data = np.array([-0.5,0.0,0,0,0,0])


    # pts = JointTrajectoryPoint()
    # traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        # for i in range(0,4):
            # pts.positions = waypoints[i]

            # pts.time_from_start = rospy.Duration(1.0)

            # Set the points to the trajectory
            # traj.points = []
            # traj.points.append(pts)
            # Publish the message
            # pub.publish(traj)
            pub.publish(joint_vels)
            rospy.sleep(1)

            pub.publish(joint_vels2)
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
	print ("Program interrupted before completion")
