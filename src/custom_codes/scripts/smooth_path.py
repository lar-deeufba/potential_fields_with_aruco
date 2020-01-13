import csv
import numpy as np
# import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, interp1d

# with open('Joint_states.csv') as csvfile:
#     reader = list(csv.reader(csvfile))

def smooth_path(way_points):
    rownum = 0
    # joint_angles = []
    Joint1, Joint2, Joint3, Joint4, Joint5, Joint6 = [], [], [], [], [], []
    index = []

    for row in way_points:
        # new_row = row[0].split()[1:]
        # raw_input(new_row)
        # joint_angles.append(new_row)
        Joint1.append(row[0])
        Joint2.append(row[1])
        Joint3.append(row[2])
        Joint4.append(row[3])
        Joint5.append(row[4])
        Joint6.append(row[5])
        index.append(rownum)
        rownum += 1

    step = 20
    red_index = index[0:-1:step]
    red_Joint1 = Joint1[0:-1:step]
    red_Joint2 = Joint2[0:-1:step]
    red_Joint3 = Joint3[0:-1:step]
    red_Joint4 = Joint4[0:-1:step]
    red_Joint5 = Joint5[0:-1:step]
    red_Joint6 = Joint6[0:-1:step]

    # Deve incluir sempre o ultimo elemento, ou o robo nao chegara a posicao final
    # if len(index) % step != 0:
    red_index = np.hstack([red_index, index[-1]])
    red_Joint1 = np.hstack([red_Joint1, Joint1[-1]])
    red_Joint2 = np.hstack([red_Joint2, Joint2[-1]])
    red_Joint3 = np.hstack([red_Joint3, Joint3[-1]])
    red_Joint4 = np.hstack([red_Joint4, Joint4[-1]])
    red_Joint5 = np.hstack([red_Joint5, Joint5[-1]])
    red_Joint6 = np.hstack([red_Joint6, Joint6[-1]])

    Joint1_cs = interp1d(red_index, red_Joint1, kind='cubic')
    Joint2_cs = interp1d(red_index, red_Joint2, kind='cubic')
    Joint3_cs = interp1d(red_index, red_Joint3, kind='cubic')
    Joint4_cs = interp1d(red_index, red_Joint4, kind='cubic')
    Joint5_cs = interp1d(red_index, red_Joint5, kind='cubic')
    Joint6_cs = interp1d(red_index, red_Joint6, kind='cubic')

    # Em xnew, pegamos o mesmo intervalo gerado em red_index e a mesma quantidade de pontos
    # da curva original
    # Ex: max(red_index) = 975 -> devido a escolha do step em red_index
    # Agora max(red_index) = 983 devido ao hstack
    # len(index) = 983 -> steps gerados pelos campos potenciais
    xnew = np.linspace(0, max(red_index), num=len(index), endpoint=True)

    # plt.subplot(321)
    # plt.plot(xnew, Joint3_cs(xnew), label='Cubic Spline - S new - Joint 3')
    # plt.plot(index, Joint3, label='Trajectory - ' + str(len(index)) + ' points')
    # plt.plot(red_index, red_Joint3, 's')


    plt.legend(loc='best', ncol=2)
    plt.xlabel("Iterations")
    plt.ylabel("Joint 3 angles [rad]")
    # plt.title("AAPF method with Orientation Control")
    plt.ylim(min(Joint3)-0.1, max(Joint3)+0.1)
    plt.xlim(0, len(index))

    # plt.show()

    wayPointsSmoothed = []
    for idx in index: #index or red_index
        array = np.array([Joint1_cs(xnew)[idx], Joint2_cs(xnew)[idx], Joint3_cs(xnew)[idx],
        Joint4_cs(xnew)[idx], Joint5_cs(xnew)[idx], Joint6_cs(xnew)[idx]])
        wayPointsSmoothed.append(array)

    return wayPointsSmoothed
