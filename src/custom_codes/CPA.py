from get_ur5_position import *
import numpy as np
from numpy.linalg import norm
import rospy

def get_joint_forces(args, ptAtual, ptFinal, oriAtual, Displacement, dist_EOF_to_Goal, Jacobian, joint_values, ur5_param, zeta, eta,
rho_0, dist_att, dist_att_config, CP_dist, CP_pos, obs_pos, CP_ur5_rep, AAPF_COMP):
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
        if abs(ptAtual[i] - ptFinal[0][i]) <= dist_att:
            f_att_l = -zeta[-1]*(ptAtual[i] - ptFinal[0][i])
        else:
            f_att_l = -dist_att*zeta[-1]*(ptAtual[i] - ptFinal[0][i])/(dist_EOF_to_Goal)

        if abs(oriAtual[i] - Displacement[i]) <= dist_att_config:
            f_att_w = -zeta[-1]*(oriAtual[i] - Displacement[i])
        else:
            f_att_w = -dist_att_config*zeta[-1]*(oriAtual[i] - Displacement[i])/dist_EOF_to_Goal

        forces_p[i, 0] = f_att_l
        forces_w[i, 0] = f_att_w

    forces_p = np.asarray(forces_p)
    forces_w = np.asarray(forces_w)
    JacobianAtt_p = np.asarray(Jacobian[5])
    JacobianAtt_w = np.asarray(Jacobian[6])
    joint_att_force_p = JacobianAtt_p.dot(forces_p)
    joint_att_force_p = np.multiply(joint_att_force_p, [[0.5], [0.1], [1.5], [1], [1], [1]])

    joint_att_force_w = JacobianAtt_w.dot(forces_w)
    # Joint 1, Joint 2, Joint 3, Joint 4, Joint 5, Joint 6
    joint_att_force_w = np.multiply(joint_att_force_w, [[0], [0], [0.05], [0.4], [0.4], [0.4]])

    ### Getting repulsive forces
    forcesRep = np.zeros((len(obs_pos), 6, 3))

    # CP_dist -> [6][13] corresponding to 6 joints and 13 obstacles
    # CP_pos -> [6][3] corresponding to 6 joints and 3 coordinates of each joint
    # obs_pos -> [13][3] corresponding to 13 obstacles and 3 coordinates
    # forcesRep -> [13][6][3] corresponding to 13 obstacles, 6 joints and 3 coordinates

    for j in range(6):
        for w in range(len(obs_pos)): # total number of obstacles - default: 13
            if CP_dist[j][w] < (rho_0[w] + CP_ur5_rep[j]/2):
                if j == 5 and (args.AAPF or AAPF_COMP):# and w == 0:
                    for k in range(3):
                        nor = (CP_pos[j][k] - obs_pos[w][k])/CP_dist[j][w]
                        Frep1 = eta[j] * (1/CP_dist[j][w] - 1/rho_0[w]) * dist_EOF_to_Goal * dist_EOF_to_Goal / (CP_dist[j][w] * CP_dist[j][w] * (1 + dist_EOF_to_Goal*dist_EOF_to_Goal)) * nor

                        nrg = (CP_pos[j][k] - ptFinal[0][k])/dist_EOF_to_Goal
                        Frep2 = eta[j] * (1/CP_dist[j][w] - 1/rho_0[w]) * (1/CP_dist[j][w] - 1/rho_0[w]) * dist_EOF_to_Goal * nrg

                        forcesRep[w][j][k] = Frep1 + Frep2
                else:
                    for k in range (3):
                        f_rep = eta[j] * (1/CP_dist[j][w] - 1/rho_0[w]) * (1 / (CP_dist[j][w]*CP_dist[j][w])) * (CP_pos[j][k] - obs_pos[w][k])/CP_dist[j][w]
                        forcesRep[w][j] = f_rep # forcesRep[13][6][3]
            else:
                forcesRep[w][j] = [0, 0, 0] # forcesRep[13][6]

    joint_rep_force = []
    f = np.zeros((3, 1))
    forcesRep = np.asarray(forcesRep)
    for q in range(6):
        for w in range(len(obs_pos)):
            JacobianRep = np.asarray(Jacobian[q])
            f[0] += forcesRep[w][q][0]
            f[1] += forcesRep[w][q][1]
            f[2] += forcesRep[w][q][2]

        # f retorna primeiro as forcas repulsivas para todos os obstaculos relacionado a mesma junta (coordenadas x, y, z)
        # raw_input(f)
        # f eh uma matriz 3x1 e Jacobian rep eh uma matriz 1x3
        joint_rep_force.append(JacobianRep.dot(f)) # calcula o jacobiano para cada matriz

    return np.transpose(joint_att_force_p), np.transpose(joint_att_force_w), np.transpose(joint_rep_force)
