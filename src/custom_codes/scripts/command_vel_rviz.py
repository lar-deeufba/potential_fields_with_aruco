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

from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_from_matrix, quaternion_multiply

# import from moveit
from moveit_python import PlanningSceneInterface

# customized code
from get_geometric_jacobian import *
from ur_inverse_kinematics import *

from pyquaternion import Quaternion

def parse_args():
    parser = argparse.ArgumentParser(description='AAPF_Orientation')
    # store_false assumes that variable is already true and is only set to false if is given in command terminal
    parser.add_argument('--AAPF', action='store_true', help='Choose AAPF instead of APF')
    parser.add_argument('--armarker', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    parser.add_argument('--dyntest', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    parser.add_argument('--APF', action='store_true', help='Choose APF instead of AAPF')
    parser.add_argument('--OriON', action='store_true', help='Activate Orientation Control')
    parser.add_argument('--COMP', action='store_true', help='Compares distance to goal using APF, AAPF w/ and without ori control')
    parser.add_argument('--CSV', action='store_true', help='Write topics into a CSV file')
    # parser.add_argument('--plot', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    parser.add_argument('--plotPath', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    parser.add_argument('--realUR5', action='store_true', help='Enable real UR5 controlling')
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
        rospy.sleep(1)

        # Standard attributes used to send joint position commands
        self.joint_vels = Float64MultiArray()
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                            'wrist_3_joint']
        self.initial_time = 4

        # Class attribute used to perform TF transformations
        self.tf = TransformListener()

        # Denavit-Hartenberg parameters of UR5
        # The order of the parameters is d1, SO, EO, a2, a3, d4, d45, d5, d6
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425, 0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15)

    """
    This function check if the goal position was reached
    """
    def all_close(self, goal, tolerance = 0.00005):

        angles_difference = [self.actual_position[i] - goal[i] for i in range(6)]
        total_error = np.sum(angles_difference)

        if abs(total_error) > tolerance:
            return False

        print("Total error: ", total_error)
        return True

    """
    This function is responsible for closing the gripper
    """
    def close_gripper(self):
        self.gripper_msg.points = [JointTrajectoryPoint(positions=[0.6], velocities=[0], time_from_start=rospy.Duration(0.1))]
        self.griper_pos.publish(self.gripper_msg)

    """
    This function is responsible for openning the gripper
    """
    def open_gripper(self):
        self.gripper_msg.points = [JointTrajectoryPoint(positions=[0.0], velocities=[0], time_from_start=rospy.Duration(1.0))]
        self.griper_pos.publish(self.gripper_msg)

    """
    The joint states published by /joint_staes of the UR5 robot are in wrong order.
    /joint_states topic normally publishes the joint in the following order:
    [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    But the correct order of the joints that must be sent to the robot is:
    ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    """
    def ur5_actual_position(self, joint_values_from_ur5):
        self.th3, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
        self.actual_position = [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6]

    """
    When to node /dynamic_goal from publish_dynamic_goal.py is used instead of Markers, this
    function is responsible for getting the coordinates published by the node and save it
    as attribute of the class
    """
    def get_goal_coordinates(self, goal_coordinates):
        self.ptFinal = [goal_coordinates.x, goal_coordinates.y, goal_coordinates.z]
        self.add_sphere(self.ptFinal, self.diam_goal, ColorRGBA(0.0, 1.0, 0.0, 1.0))

    """
    Used to test velcoity control under /joint_group_vel_controller/command topic
    """
    def velocity_control_test(self):
        # publishing rate for velocity control
        rate = rospy.Rate(125)

        while not rospy.is_shutdown():
            self.joint_vels.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.pub_vel.publish(self.joint_vels)
            rospy.loginfo(self.actual_position)
            rate.sleep()

    """
    Calculate the initial robot position - Used before CPA application
    """
    def get_ik(self, pose):

        matrix = TransformerROS()

        # The orientation of /tool0 will be constant
        q = quaternion_from_euler(0, 3.14, 1.57)

        matrix2 = matrix.fromTranslationRotation((pose[0]*(-1), pose[1]*(-1), pose[2]), (q[0], q[1], q[2], q[3]))

        th = invKine(matrix2)
        sol1 = th[:, 2].transpose()
        joint_values_from_ik = np.array(sol1)

        joint_values = joint_values_from_ik[0, :]

        return joint_values.tolist()

    """
    Send the HOME position to the robot
    self.client.wait_for_result() does not work well.
    Instead, a while loop has been created to ensure that the robot reaches the
    goal even after the failure.
    """
    def home_pos(self, home_pos = [3.14, -1.5707, 0, -1.5707, -1.5707, -1.5707]):
        home_pos[-1] = 3.14

        # First point is current position
        try:
            self.goal.trajectory.points.append(JointTrajectoryPoint(positions=home_pos, velocities=[0]*6, time_from_start=rospy.Duration(self.initial_time)))
            self.initial_time += 1
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            while not self.all_close(home_pos) and not rospy.is_shutdown():
                self.client.send_goal(self.goal)
                self.client.wait_for_result()
            if self.all_close(home_pos):
                print("The arm is in home position!  \n")
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
    Get forces from APF algorithm
    """
    def get_joint_forces(self, ptAtual, ptFinal, oriAtual, Displacement, dist_EOF_to_Goal, Jacobian,
                         zeta, dist_att, dist_att_config, err_ori):

        # Getting attractive forces
        forces_p = np.zeros((3, 1))
        forces_w = np.zeros((3, 1))

        for i in range(3):
            if abs(ptAtual[i] - ptFinal[i]) <= dist_att:
                f_att_l = -zeta[-1]*(ptAtual[i] - ptFinal[i])
            else:
                f_att_l = -dist_att*zeta[-1]*(ptAtual[i] - ptFinal[i])/dist_EOF_to_Goal

            if abs(oriAtual[i] - Displacement[i]) <= dist_att_config:
                f_att_w = -zeta[-1]*(oriAtual[i] - Displacement[i])
            else:
                f_att_w = -dist_att_config*zeta[-1]*(oriAtual[i] - Displacement[i])/dist_EOF_to_Goal

            forces_p[i, 0] = f_att_l
            forces_w[i, 0] = f_att_w

        forces_p = np.asarray(forces_p)
        JacobianAtt_p = np.asarray(Jacobian[5])
        joint_att_force_p = JacobianAtt_p.dot(forces_p)
        joint_att_force_p = np.multiply(joint_att_force_p, [[0.5], [0.1], [1.5], [1], [1], [1]])

        forces_w = np.asarray(forces_w)
        JacobianAtt_w = np.asarray(Jacobian[6])
        joint_att_force_w = JacobianAtt_w.dot(forces_w)
        joint_att_force_w = np.multiply(joint_att_force_w, [[0], [0.1], [0.1], [0.4], [0.4], [0.4]])

        return np.transpose(joint_att_force_p), np.transpose(joint_att_force_w)

    """
    Stop the robot when the node is killed
    """
    def stop_robot(self):
        # Set zero velocity in order to keep the robot in last calculated position
        rospy.loginfo("Stopping robot!")
        self.joint_vels.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pub_vel.publish(self.joint_vels)

    """
    Function to ensure safety
    """
    def safety_stop(self, ptAtual, wristPt):
        # High limit in meters of the end effector relative to the base_link
        high_limit = 0.03

        # Does not allow wrist_1_link to move above 20 cm relative to base_link
        high_limit_wrist_pt = 0.2

        if ptAtual[-1] < high_limit or wristPt[-1] < high_limit_wrist_pt:
            # Be careful. Only the limit of the end effector is being watched but the other
            # joint can also exceed this limit and need to be carefully watched by the operator
            rospy.loginfo("High limit of " + str(high_limit) + " exceeded!")
            self.stop_robot()
            rosservice.call_service('/controller_manager/switch_controller', [['pos_based_pos_traj_controller'], ['joint_group_vel_controller'], 1])

            raw_input("\n==== Press enter to home the robot again!")
            joint_values = ur5_vel.get_ik([-0.4, -0.1, 0.5 + 0.15])
            ur5_vel.home_pos(joint_values)

            rosservice.call_service('/controller_manager/switch_controller', [['joint_group_vel_controller'], ['pos_based_pos_traj_controller'], 1])
            raw_input("\n==== Press enter to restart APF function!")

    """
    Gets ptFinal and oriAtual
    """
    def get_tf_param(self):
        # Check if a marker is used instead of a dynamic goal published by publish_dynamic_goal.py
        if self.args.armarker:
            # When the marker disappears we get an error and the node is killed. To avoid this
            # we implemented this try function to check if ar_marker_0 frame is available
            try:
                # used when Ar Marker is ON
                ptFinal, oriFinal = self.tf.lookupTransform("base_link", "ar_marker_0", rospy.Time())
                oriFinal = list(euler_from_quaternion(oriFinal))

                # Make YAW Angle goes from 0 to 2*pi
                # Solution proposed by
                # https://answers.ros.org/question/302953/tfquaternion-getangle-eqivalent-for-rospy/
                ptAtual, oriAtual = self.tf.lookupTransform("ar_marker_0", "grasping_link", rospy.Time())
                angle = -1 * 2 * np.arccos(oriAtual[-1])
                oriAtual = list(euler_from_quaternion(oriAtual))
                oriAtual[0] = angle

                # Add a green sphere at ptFinal position
                self.add_sphere(ptFinal, self.diam_goal, ColorRGBA(0.0, 1.0, 0.0, 1.0))
            except:
                if not rospy.is_shutdown():
                    self.stop_robot()
                    raw_input("\nWaiting for /ar_marker_0 frame to be available! Press ENTER after /ar_marker_0 shows up.")
                    self.CPA_vel_control()
        elif self.args.dyntest:
            ptFinal = self.ptFinal

        return ptFinal, oriAtual, oriFinal

    """
    Main function related the Artificial Potential Field method
    """
    def CPA_vel_control(self):

        # At the end, the disciplacement will take place as a final orientation
        Displacement = [0.04, 0.04, 0.04]

        # CPA Parameters
        zeta = [0.5 for i in range(7)]  # Attractive force gain of the goal
        dist_att = 0.1  # Influence distance in workspace
        dist_att_config = 0.2  # Influence distance in configuration space
        alfa_geral = 1.5 # multiply each alfa (position and rotation) equally
        alfa = 4*alfa_geral  # Grad step of positioning - Default: 0.5
        alfa_rot = 4*alfa_geral  # Grad step of orientation - Default: 0.4

        # Return the end effector location relative to the base_link
        ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())

        # Get ptFinal published by ar_marker_0 frame and the orientation from grasping_link to ar_marker_0
        ptFinal, oriAtual, oriFinal = self.get_tf_param()
        print "oriFinal (ar_marker_0 from base_link): ", oriFinal
        print "oriAtual (grasping_link from ar_marker_0): ", oriAtual, " \n"

        # Calculate the correction of the orientation relative to the actual orientation
        R, P, Y = -1*oriAtual[0], -1*oriAtual[1], 0.0
        corr = [R, P, Y]
        print "Correction angle (from quat multiply): ", corr

        # Calculate the distance between end effector and goal
        dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal))

        # Frequency of the velocity controller pubisher
        # Max frequency: 125 Hz
        rate = rospy.Rate(125)

        raw_input("\n==== Press enter to start APF")
        while not rospy.is_shutdown():

            # Get UR5 Jacobian of each link
            Jacobian = get_geometric_jacobian(self.ur5_param, self.actual_position)

            # Get absolute orientation error
            err_ori = np.sum(oriAtual)

            # In order to keep orientation constant, we need to correct the orientation
            # of the end effector in respect to the ar_marker_0
            oriAtual = [oriAtual[i] + corr[i] for i in range(len(corr))]

            print "Angle error (RPY): ", oriAtual

            # Get attractive linear and angular forces and repulsive forces
            joint_att_force_p, joint_att_force_w = \
                self.get_joint_forces(ptAtual, ptFinal, oriAtual, Displacement,
                                     dist_EOF_to_Goal, Jacobian, zeta,
                                     dist_att, dist_att_config, err_ori)

            # Publishes joint valocities related to position only
            self.joint_vels.data = np.array(alfa * joint_att_force_p[0])
            # If orientation control is turned on, sum actual position forces to orientation forces
            if self.args.OriON:
                self.joint_vels.data = self.joint_vels.data + \
                    alfa_rot * joint_att_force_w[0]

            self.pub_vel.publish(self.joint_vels)

            # Return the end effector location relative to the base_link
            ptAtual, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time())

            # Check wrist_1_link position just for safety
            wristPt, _ = self.tf.lookupTransform("base_link", "wrist_1_link", rospy.Time())

            # Function to ensure safety. It does not allow End Effector to move below 20 cm above the desk
            self.safety_stop(ptAtual, wristPt)

            # Get ptFinal published by ar_marker_0 frame and the orientation from grasping_link to ar_marker_0
            # The oriFinal needs to be tracked online because the object will be dynamic
            ptFinal, oriAtual, oriFinal = self.get_tf_param()

            print "oriFinal: ", oriFinal
            print "oriAtual: ", oriAtual, " \n"

            # Calculate the distance between end effector and goal
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
        ur5_vel.scene.clear()
        raw_input("\n==== Press enter to load home position!")

        # Calculate the joint_values equivalent to the initial position before grasping
        joint_values = ur5_vel.get_ik([-0.4, -0.1, 0.5 + 0.15])
        ur5_vel.home_pos(joint_values)

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
