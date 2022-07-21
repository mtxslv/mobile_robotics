from roboticstoolbox.mobile import DistanceTransformPlanner

import sys
import os


import timeit
import numpy as np
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math

# Conectar no Vrep
clientID = connect_2_sim()
test_connection(clientID)

# Enviar pontos do Caminho Gerado
#send_points_to_sim([p[1:3] for p in points], clientID=clientID)

left_motor_handle, right_motor_handle = get_dr20_motor_handles_(clientID)

return_value_left_motor_control, return_value_right_motor_control = robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)

errorCode, robo = sim.simxGetObjectHandle(clientID=clientID, objectName="./dr20",
                                          operationMode=sim.simx_opmode_blocking)

error_pos, pos_robo = sim.simxGetObjectPosition(clientID, robo, -1, sim.simx_opmode_streaming)
error_ang, ang_robo = sim.simxGetObjectOrientation(clientID, robo, -1, sim.simx_opmode_streaming)

print(robo)
print(pos_robo)
print(ang_robo)

#PONTOS FINAIS
'''
lista_pontos = [[1.5,-1.5], [1.5,0], [1.5,1.5], 
                [-1.5,-1.5], [-1.5,0], [-1.5,1.5],
                [0,0]]
'''
"""
lista_pontos = [[-2.02499950075732, 0.997989014727446],
 [-1.97499978701219, 1.08165714350794],
 [-1.47499978701219, 0.947755848346502],
 [-1.02499950075732, 0.154285298001702],
 [-0.850000118019009, -0.241338562783844],
 [-0.399999502791271, -1.00226402234716],
 [0.100000497208729, -0.762012722182138],
 [0.149999881980991, -0.672365149298713],
 [0.899999970683028, -1.21394604254210],
 [1.3249999284744263, -1.6999998092651367]]
"""
lista_pontos = [[-1.51047619,  1.51047619],
       [-1.46095238,  1.46095238],
       [-1.41142857,  1.41142857],
       [-1.36190476,  1.36190476],
       [-1.31238095,  1.31238095],
       [-1.26285714,  1.26285714],
       [-1.21333333,  1.21333333],
       [-1.16380952,  1.16380952],
       [-1.11428571,  1.11428571],
       [-1.0647619 ,  1.0647619 ],
       [-1.0152381 ,  1.0152381 ],
       [-0.96571429,  0.96571429],
       [-0.91619048,  0.91619048],
       [-0.86666667,  0.86666667],
       [-0.81714286,  0.81714286],
       [-0.76761905,  0.76761905],
       [-0.71809524,  0.71809524],
       [-0.66857143,  0.66857143],
       [-0.61904762,  0.61904762],
       [-0.56952381,  0.56952381],
       [-0.52      ,  0.52      ],
       [-0.47047619,  0.47047619],
       [-0.42095238,  0.42095238],
       [-0.37142857,  0.37142857],
       [-0.32190476,  0.32190476],
       [-0.27238095,  0.27238095],
       [-0.22285714,  0.22285714],
       [-0.17333333,  0.17333333],
       [-0.12380952,  0.12380952],
       [-0.07428571,  0.07428571],
       [-0.0247619 ,  0.0247619 ],
       [ 0.0247619 , -0.0247619 ],
       [ 0.07428571, -0.07428571],
       [ 0.12380952, -0.12380952],
       [ 0.17333333, -0.17333333],
       [ 0.22285714, -0.22285714],
       [ 0.27238095, -0.27238095],
       [ 0.32190476, -0.32190476],
       [ 0.37142857, -0.37142857],
       [ 0.42095238, -0.42095238],
       [ 0.47047619, -0.47047619],
       [ 0.52      , -0.52      ],
       [ 0.56952381, -0.56952381],
       [ 0.61904762, -0.61904762],
       [ 0.66857143, -0.66857143],
       [ 0.71809524, -0.71809524],
       [ 0.76761905, -0.76761905],
       [ 0.81714286, -0.81714286],
       [ 0.86666667, -0.86666667],
       [ 0.91619048, -0.91619048],
       [ 0.96571429, -0.96571429]]

lista_pontos.reverse()

#Ganhos do controlador
k_theta = 1
k_l = 0.07

#Dados do robô
rd = 0.084601/2
re = 0.084601/2
B = 0.214



for ponto in lista_pontos:
    xf = ponto[0]
    yf = ponto[1]

    p = np.array([[xf,yf]])
    send_points_to_sim(p, clientID=clientID)
    print(f'\nGoing to point {ponto}')
    start = timeit.default_timer()
    while True:
        error_pos, pos_robo = sim.simxGetObjectPosition(clientID, robo, -1, sim.simx_opmode_buffer)
        error_ang, ang_robo = sim.simxGetObjectOrientation(clientID, robo, -1, sim.simx_opmode_buffer)

        theta_robo = ang_robo[2] #+ np.pi/2

        #Calculo dos Delta x e y 
        delta_x = xf - pos_robo[0]
        delta_y = yf - pos_robo[1]

        #Calculo do theta estrela/Referencial
        theta_ref = np.arctan2(delta_y,delta_x)     


        #Calculo do delta l do referencial e  delta theta
        delta_l_ref = np.sqrt((delta_x)**2 + (delta_y)**2)     
        delta_theta = theta_ref - theta_robo

        if(delta_l_ref <= 0.1):
            break

        #Caluclo do delta L
        delta_l = delta_l_ref * np.cos(delta_theta)

        #Calculo da velocidade linear e angular
        v = k_l * delta_l
        w = k_theta * delta_theta

        #velocidades das juntas
        wd = (v/rd) + (B/(2*rd))*w
        we = (v/re) - (B/(2*re))*w

        #txt = [np.array(pos_robo).round(2), np.array(ang_robo).round(2), delta_l.round(2), we.round(2), wd.round(2)]
        robot_run(clientID, left_motor_handle, right_motor_handle, we, wd)
        #print(txt, ret)
            
    stop = timeit.default_timer()
    robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)
    print(f'Arrived at point {ponto} with pos {[px.round(4) for px in np.array(pos_robo)]}')
    print(f'Time: {round(stop - start, 3)}s\n')
