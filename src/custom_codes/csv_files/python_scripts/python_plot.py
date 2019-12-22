import csv
import numpy as np
# import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, interp1d

with open('joint_traj_2_APF.csv') as csvfile:
    reader = list(csv.reader(csvfile))

    rownum = 0
    joint_angles = []
    Joint1, Joint2, Joint3, Joint4, Joint5, Joint6 = [], [], [], [], [], []
    index = []

    for row in reader[1:]:
        new_row = row[0].split()[1:]
        # raw_input(new_row)
        joint_angles.append(new_row)
        Joint1.append(new_row[0])
        Joint2.append(new_row[1])
        Joint3.append(new_row[2])
        Joint4.append(new_row[3])
        Joint5.append(new_row[4])
        Joint6.append(new_row[5])
        index.append(rownum)
        rownum += 1

# print(np.asarray(index[:100]))
# print(np.asarray(Joint1[:100]))
index = np.asarray(index)
Joint1 = np.asarray(Joint1)
Joint1 = Joint1.astype(np.float)
Joint2 = np.asarray(Joint2)
Joint2 = Joint2.astype(np.float)
Joint3 = np.asarray(Joint3)
Joint3 = Joint3.astype(np.float)
Joint4 = np.asarray(Joint4)
Joint4 = Joint4.astype(np.float)
Joint5 = np.asarray(Joint5)
Joint5 = Joint5.astype(np.float)
Joint6 = np.asarray(Joint6)
Joint6 = Joint6.astype(np.float)

#
step = 50
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
raw_input(len(xnew))
plt.plot(xnew, Joint1_cs(xnew), label='Cubic Spline - S new')
plt.plot(index, Joint1, label='Trajectory - ' + str(len(index)) + ' points')
plt.plot(red_index, red_Joint1, 's')

# print(Joint1_cs(xnew)[0])

wayPointsSmoothed = []
joint_angles = []
# index
for idx in index:
    array = np.array([Joint1_cs(xnew)[idx], Joint2_cs(xnew)[idx], Joint3_cs(xnew)[idx],
    Joint4_cs(xnew)[idx], Joint5_cs(xnew)[idx], Joint6_cs(xnew)[idx]])
    wayPointsSmoothed.append(array)

print(len(wayPointsSmoothed))

plt.legend(loc='best', ncol=2)
plt.xlabel("Iterations")
plt.ylabel("Joint angles [rad]")
plt.title("AAPF method with Orientation Control")
plt.ylim(min(Joint1)-0.1, max(Joint1)+0.1)
plt.xlim(0, len(index))

plt.show()



### BACKUP

# red_index = index[0:-1:15]
# red_Joint1 = Joint1[0:-1:15]
# plt.plot(red_index, cs(red_index), label='Cubic Spline - S')
# cs = CubicSpline(red_index, red_Joint1)
# plt.plot(red_index, cs(red_index, 1), label='Cubic Spline - S\'')
# plt.plot(red_index, cs(red_index, 2), label='Cubic Spline - S\'\'')

# Temos que pegar alguns pontos da trajetoria
# red_index e red_Joint1 -> 66 pontos

# plt.plot(red_index, red_Joint1, 'o')
# print(max(red_index))
