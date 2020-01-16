#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

import rospy
import actionlib
import numpy as np
import argparse
import rosservice

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Vector3, Pose

from tf import TransformListener
from tf.transformations import euler_from_quaternion #, quaternion_matrix

# import from moveit
from moveit_python import PlanningSceneInterface

# customized code
from get_geometric_jacobian import *

def parse_args():
    parser = argparse.ArgumentParser(description='AAPF_Orientation')
    # store_false assumes that variable is already true and is only set to false if is given in command terminal
    parser.add_argument('--armarker', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    parser.add_argument('--dyntest', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    # parser.add_argument('--plot', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    parser.add_argument('--plotPath', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    args = parser.parse_args()
    return args

class vel_control:
    def __init__(self, args):
        self.args = args

        # attributes used to receive msgs while publishing new ones
        self.processing = False
        self.new_msg = False
        self.msg = None

        # CPA Parameters
        self.diam_goal = 0.05

        # Topic used to publish vel commands
        self.pub_vel = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray,  queue_size=10)

        # visual tools from moveit
        self.scene = PlanningSceneInterface("base_link")
        self.marker_publisher = rospy.Publisher('visualization_marker2', Marker, queue_size=10)

        # Subscriber used to read joint values
        rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position, queue_size=10)

        # if true, this node receives messages from publish_dynamic_goal.py
        if self.args.dyntest:
            # Subscriber used to receive goal coordinates from publish_dynamic_goal.py
            rospy.Subscriber('/dynamic_goal', Point, self.get_goal_coordinates, queue_size=10)

        # actionClient used to send joint positions
        self.client = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server (pos_based_pos_traj_controller)..."
        self.client.wait_for_server()
        print "Connected to server (pos_based_pos_traj_controller)"
        rospy.sleep(2)

        # Standard attributes used to send joint position commands
        self.joint_vels = Float64MultiArray()
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                            'wrist_3_joint']

        # TF transformations
        self.tf = TransformListener()

        # Data for customized code
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425, 0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15) # d1, SO, EO, a2, a3, d4, d45, d5, d6

    def ur5_actual_position(self, joint_values_from_ur5):
        # It reads the last robot position
        self.th3, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
        self.actual_position = [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6]
        # rospy.loginfo(self.actual_position)

    def get_goal_coordinates(self, goal_coordinates):
        self.ptFinal = [goal_coordinates.x, goal_coordinates.y, goal_coordinates.z]
        self.add_sphere(self.ptFinal, self.diam_goal, ColorRGBA(0.0, 1.0, 0.0, 1.0))

    def stop_robot(self):
        # Set zero velocity in order to keep the robot in last calculated position
        rospy.loginfo("Stopping robot!")
        self.joint_vels.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pub_vel.publish(self.joint_vels)

    def velocity_control_test(self):
        # publishing rate for velocity control
        rate = rospy.Rate(125)

        while not rospy.is_shutdown():
            self.joint_vels.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.pub_vel.publish(self.joint_vels)
            rospy.loginfo(self.actual_position)
            rate.sleep()

    def home_pos(self):
        # home_pos = [2.7, -1.5707, 0.3, -1.5707, -1.5707, -1.5707]
        home_pos = [3.14, -1.5707, 0, -1.5707, -1.5707, -1.5707]
        # home_pos = [0.0, 0.0, 0, 0.0, 0.0, 0.0]

        # First point is current position
        try:
            self.goal.trajectory.points.append(JointTrajectoryPoint(positions=home_pos, velocities=[0]*6, time_from_start=rospy.Duration(4)))
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            rospy.sleep(2)
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                print("The arm is in home position")
            else:
                print("The arm failed to execute the trajectory.")
                print self.client.get_state()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    """
    Adds spheres in RVIZ - Used to plot goals and obstacles
    """

    def add_sphere(self, pose, diam, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = 0
        marker.pose.position = Point(pose[0], pose[1], pose[2])
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale = Vector3(diam, diam, diam)
        marker.color = color
        self.marker_publisher.publish(marker)

    """
    TH Matrix from joint angles
    """

    def matrix_from_joint_angles(self):
            th1, th2, th3, th4, th5, th6 = self.th1, self.th2, self.th3, self.th4, self.th5, self.th6
            d1, SO, EO, a2, a3, d4, d45, d5, d6 = self.ur5_param

            matrix = [[-(sin(th1)*sin(th5) + cos(th1)*cos(th5)*cos(th2 + th3 + th4))*cos(th6) + sin(th6)*sin(th2 + th3 + th4)*cos(th1), (sin(th1)*sin(th5) + cos(th1)*cos(th5)*cos(th2 + th3 + th4))*sin(th6) + sin(th2 + th3 + th4)*cos(th1)*cos(th6), -sin(th1)*cos(th5) + sin(th5)*cos(th1)*cos(th2 + th3 + th4), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1) - d6*(sin(th1)*cos(th5) - sin(th5)*cos(th1)*cos(th2 + th3 + th4))], [-(sin(th1)*cos(th5)*cos(th2 + th3 + th4) - sin(th5)*cos(th1))*cos(th6) + sin(th1)*sin(th6)*sin(th2 + th3 + th4), (sin(th1)*cos(th5)*cos(th2 + th3 + th4) - sin(th5)*cos(th1))*sin(th6) + sin(th1)*sin(th2 + th3 + th4)*cos(th6), sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5), EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2) + a3*sin(th1)*cos(th2 + th3) + d45*cos(th1) - d5*sin(th1)*sin(th2 + th3 + th4) + d6*(sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5))], [sin(th6)*cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*cos(th5)*cos(th6), -sin(th6)*sin(th2 + th3 + th4)*cos(th5) + cos(th6)*cos(th2 + th3 + th4), -sin(th5)*sin(th2 + th3 + th4), -a2*sin(th2) - a3*sin(th2 + th3) + d1 - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4)], [0, 0, 0, 1]]
            return matrix

    """
    Get forces from APF algorithm
    """
    def get_joint_forces(self, ptAtual, ptFinal, dist_EOF_to_Goal, Jacobian, joint_values, ur5_param, zeta, dist_att):

        # Getting attractive forces
        forces_p = np.zeros((3, 1))

        for i in range(3):
            if abs(ptAtual[i] - ptFinal[i]) <= dist_att:
                f_att_l = -zeta[-1]*(ptAtual[i] - ptFinal[i])
            else:
                f_att_l = -dist_att*zeta[-1]*(ptAtual[i] - ptFinal[i])/(dist_EOF_to_Goal)
            forces_p[i, 0] = f_att_l

        forces_p = np.asarray(forces_p)
        JacobianAtt_p = np.asarray(Jacobian[5])
        joint_att_force_p = JacobianAtt_p.dot(forces_p)
        joint_att_force_p = np.multiply(joint_att_force_p, [[1], [1.5], [1.5], [1], [1], [1]])

        return np.transpose(joint_att_force_p)

    def CPA_vel_control(self):

        # CPA Parameters
        zeta = [0.5 for i in range(7)]  # Attractive force gain of the goal
        dist_att = 0.05  # Influence distance in workspace
        alfa = 4  # Grad step of positioning - Default: 0.5

        ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())

        # if true, this node receives messages from publish_dynamic_goal.py
        if self.args.armarker:
            try:
                # used when Ar Marker is ON
                ptFinal, _ = self.tf.lookupTransform("base_link", "ar_marker_0", rospy.Time())
            except:
                if not rospy.is_shutdown():
                    raw_input("Put the marker in front of cam and press enter after it is known!")
                    self.CPA_vel_control()
        elif self.args.dyntest:
            ptFinal = self.ptFinal

        dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal))
        print("Dist to goal: " + str(dist_EOF_to_Goal))

        self.scene.clear()

        rate = rospy.Rate(80)

        raw_input("' =========== Press enter to start APF")

        while not rospy.is_shutdown():

            # Get UR5 Jacobian of each link
            Jacobian = get_geometric_jacobian(self.ur5_param, self.actual_position)

            # Get attractive linear and angular forces and repulsive forces
            joint_att_force_p = \
                self.get_joint_forces(ptAtual, ptFinal,
                                     dist_EOF_to_Goal, Jacobian, self.actual_position, self.ur5_param, zeta, dist_att)

            self.joint_vels.data = np.array(alfa * joint_att_force_p[0])

            self.pub_vel.publish(self.joint_vels)

            if self.args.armarker:
                try:
                    # used when Ar Marker is ON
                    ptFinal, _ = self.tf.lookupTransform("base_link", "ar_marker_0", rospy.Time())
                    self.add_sphere(ptFinal, self.diam_goal, ColorRGBA(0.0, 1.0, 0.0, 1.0))
                except:
                    if not rospy.is_shutdown():
                        self.stop_robot()
                        raw_input("Put the marker in front of cam and press enter after it is known!")
                        self.CPA_vel_control()
            elif self.args.dyntest:
                ptFinal = self.ptFinal

            ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())

            dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal))

            rate.sleep()

if __name__ == '__main__':
    try:
        arg = parse_args()

        '''
        Position Control Test
        '''
        rosservice.call_service('/controller_manager/switch_controller', [['pos_based_pos_traj_controller'], ['joint_group_vel_controller'], 1])
        ur5_vel = vel_control(arg)
        raw_input("Press enter to load home position!")
        ur5_vel.home_pos()

        '''
        Velocity Control Test
        '''
        rosservice.call_service('/controller_manager/switch_controller', [['joint_group_vel_controller'], ['pos_based_pos_traj_controller'], 1])
        rospy.on_shutdown(ur5_vel.stop_robot)
        ur5_vel.CPA_vel_control()

        # '''
        # Velocity Control Test
        # '''
        # raw_input("Press enter to load velocity control!")
        # ur5_vel.velocity_control_test()

    except rospy.ROSInterruptException:
	    rospy.loginfo("Program interrupted before completion")
