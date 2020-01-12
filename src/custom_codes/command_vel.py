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
from tf.transformations import quaternion_from_euler, euler_from_matrix #, quaternion_matrix

# import from moveit
from moveit_python import PlanningSceneInterface

# customized code
from get_geometric_jacobian import *

def parse_args():
    parser = argparse.ArgumentParser(description='AAPF_Orientation')
    # store_false assumes that variable is already true and is only set to false if is given in command terminal
    parser.add_argument('--AAPF', action='store_true', help='Choose AAPF instead of APF')
    parser.add_argument('--APF', action='store_true', help='Choose APF instead of AAPF')
    parser.add_argument('--OriON', action='store_true', help='Activate Orientation Control')
    parser.add_argument('--COMP', action='store_true', help='Compares distance to goal using APF, AAPF w/ and without ori control')
    parser.add_argument('--CSV', action='store_true', help='Write topics into a CSV file')
    # parser.add_argument('--plot', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    parser.add_argument('--plotPath', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    parser.add_argument('--realUR5', action='store_true', help='Enable real UR5 controlling')
    args = parser.parse_args()
    return args

def print_output(n, way_points, wayPointsSmoothed, dist_EOF_to_Goal):
    print("Dados dos CPAAs")
    print("Iterations: ", n)
    print("Way points: ", len(way_points))
    print("Way points smoothed: ", len(wayPointsSmoothed))
    print("Distance to goal: ", dist_EOF_to_Goal)

class vel_control:
    def __init__(self, args):

        self.args = args

        # attributes used to receive msgs while publishing new ones
        self.processing = False
        self.new_msg = False
        self.msg = None

        # Topic used to publish vel commands
        self.pub_vel = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray,  queue_size=10)

        # Topic used to read joint values
        rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position, queue_size=10)

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

        # visual tools from moveit
        self.scene = PlanningSceneInterface("base_link")
        self.marker_publisher = rospy.Publisher('visualization_marker2', Marker, queue_size=10)

        # TF transformations
        self.tf = TransformListener()

        # Data for customized code
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425, 0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15) # d1, SO, EO, a2, a3, d4, d45, d5, d6

    def ur5_actual_position(self, joint_values_from_ur5):
        # It reads the last robot position
        self.th3, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
        self.actual_position = [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6]
        # rospy.loginfo(self.actual_position)

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
        home_pos = [3.14, -1.5707, 0, -1.5707, -1.5707, -1.5707]
        # home_pos = [0.0, 0.0, 0, 0.0, 0.0, 0.0]

        # First point is current position
        try:
            self.goal.trajectory.points.append(JointTrajectoryPoint(positions=home_pos, velocities=[0]*6, time_from_start=rospy.Duration(2)))
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            rospy.sleep(3)
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
    Adds the obstacles and repulsive control points on the robot
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
    def get_joint_forces(self, ptAtual, ptFinal, oriAtual, Displacement, dist_EOF_to_Goal, Jacobian, joint_values, ur5_param, zeta, eta,
    rho_0, dist_att, dist_att_config):
        """
        Get attractive and repulsive forces

        :param CP_dist: [6][13] array vector corresponding to 6 joints and 13 obstacles
        :param CP_pos: [6][3] array vector corresponding to 6 joints and 3 coordinates of each joint
        :param obs_pos: [13][3] array vector corresponding to 13 obstacles and 3 coordinates
        :return: attractive and repulsive forces
        """

        # Getting attractive forces
        forces_p = np.zeros((3, 1))
        forces_w = np.zeros((3, 1))

        for i in range(3):
            if abs(ptAtual[i] - ptFinal[i]) <= dist_att:
                f_att_l = -zeta[-1]*(ptAtual[i] - ptFinal[i])
            else:
                f_att_l = -dist_att*zeta[-1]*(ptAtual[i] - ptFinal[i])/(dist_EOF_to_Goal)

            if abs(oriAtual[i] - Displacement[i]) <= dist_att_config:
                f_att_w = -zeta[-1]*(oriAtual[i] - Displacement[i])
            else:
                f_att_w = -dist_att_config*zeta[-1]*(oriAtual[i] - Displacement[i])/dist_EOF_to_Goal

            forces_p[i, 0] = f_att_l
            forces_w[i, 0] = f_att_w

        forces_p = np.asarray(forces_p)
        JacobianAtt_p = np.asarray(Jacobian[5])
        joint_att_force_p = JacobianAtt_p.dot(forces_p)
        # joint_att_force_p = np.multiply(joint_att_force_p, [[1], [1], [1], [1], [1], [1]])

        forces_w = np.asarray(forces_w)
        JacobianAtt_w = np.asarray(Jacobian[6])
        joint_att_force_w = JacobianAtt_w.dot(forces_w)
        joint_att_force_w = np.multiply(joint_att_force_w, [[0], [0], [0.05], [0.4], [0.4], [0.4]])

        return np.transpose(joint_att_force_p), np.transpose(joint_att_force_w)

    def CPA_vel_control(self, ptFinal, AAPF_COMP = False, ORI_COMP = False):

        # Final position
        Displacement = [0.01, 0.01, 0.01]
        self.ptFinal = ptFinal

        # CPA Parameters
        diam_goal = 0.05
        diam_obs = 0.3 # still need to update this parameter accondinly to real time control
        err = diam_goal / 2  # Max error allowed
        max_iter = 1500  # Max iterations
        zeta = [0.5 for i in range(7)]  # Attractive force gain of each obstacle
        rho_0 = diam_obs / 2  # Influence distance of each obstacle
        dist_att = 0.05  # Influence distance in workspace
        dist_att_config = 0.2  # Influence distance in configuration space
        alfa = 2  # Grad step of positioning - Default: 0.5
        alfa_rot = 0.05  # Grad step of orientation - Default: 0.4
        CP_ur5_rep = [0.15]*6  # Repulsive fields on UR5
        CP_ur5_rep[-2] = 0.15

        # This repulsive gain is used if AAPF is set on with orientation control
        eta = [0.00001 for i in range(6)]  # Repulsive gain of each obstacle default: 0.00006

        # This attractive gain is used if AAPF is set on WITHOUT orientation control
        if self.args.AAPF and not self.args.OriON or (AAPF_COMP and not ORI_COMP):
            eta = [0.0006 for i in range(6)]

        ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())
        print("Grasping link actual position: " + str(ptAtual))

        dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(self.ptFinal))
        print("Dist to goal: " + str(dist_EOF_to_Goal))

        raw_input("' =========== Press enter to load the visual tools")
        self.scene.clear()
        # Final positions
        self.add_sphere(self.ptFinal, diam_goal, ColorRGBA(0.0, 1.0, 0.0, 1.0))

        # Angle correction relative to base_link (from grasping_link)
        R, P, Y = 1.5707, 0, -1.5707

        n = 0
        err_ori = 1
        corr = [R, P, Y]

        joint_attractive_forces = np.zeros(6)
        joint_rep_forces = np.zeros(6)
        ori_atual_vec = np.zeros(3)

        rate = rospy.Rate(90)

        raw_input("' =========== Press enter to start APF")
        while (dist_EOF_to_Goal > err  or abs(err_ori) > 0.02) and not rospy.is_shutdown():

            if dist_EOF_to_Goal > err:
                # Get UR5 Jacobian of each link
                Jacobian = get_geometric_jacobian(self.ur5_param, self.actual_position)

                oriAtual = euler_from_matrix(self.matrix_from_joint_angles())
                oriAtual = [oriAtual[i] + corr[i] for i in range(len(corr))]

                # Get attractive linear and angular forces and repulsive forces
                joint_att_force_p, joint_att_force_w = \
                    self.get_joint_forces(ptAtual, self.ptFinal, oriAtual, Displacement,
                                         dist_EOF_to_Goal, Jacobian, self.actual_position, self.ur5_param, zeta,
                                         eta, rho_0, dist_att, dist_att_config)

                # Joint angles UPDATE - Attractive force
                # self.joint_states.position = self.joint_states.position + \
                #     alfa * joint_att_force_p[0]
                self.joint_vels.data = np.array(alfa * joint_att_force_p[0])
                self.pub_vel.publish(self.joint_vels)

                ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())

                # Get absolute orientation error
                err_ori = np.sum(oriAtual)

                dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(self.ptFinal))
                rospy.loginfo(dist_EOF_to_Goal)

                rate.sleep()
            else:
                ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())
                dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(self.ptFinal))
                rospy.loginfo(dist_EOF_to_Goal)

                self.joint_vels.data = np.array([0, 0, 0, 0, 0, 0])
                self.pub_vel.publish(self.joint_vels)

if __name__ == '__main__':
    try:
        arg = parse_args()

        '''
        Position Control Test
        '''
        rosservice.call_service('/controller_manager/switch_controller', [['pos_based_pos_traj_controller'], ['joint_group_vel_controller'], 1])
        ur5_vel = vel_control(arg)
        ur5_vel.home_pos()

        '''
        APF test
        '''
        rosservice.call_service('/controller_manager/switch_controller', [['joint_group_vel_controller'], ['pos_based_pos_traj_controller'], 1])
        rospy.on_shutdown(ur5_vel.stop_robot)
        ptFinal = [-0.4, 0.2, 0.75]
        ur5_vel.CPA_vel_control(ptFinal)

        '''
        Velocity Control Test
        '''
        raw_input("Press enter to load velocity control!")
        ur5_vel.velocity_control_test()


    except rospy.ROSInterruptException:
	    rospy.loginfo("Program interrupted before completion")
