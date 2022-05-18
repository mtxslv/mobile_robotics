import sys
import os


import timeit
import numpy as np
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math

# Gerar Caminho
#pos_ini = [0,0,0]
#pos_fin = [2.5,2.5,math.pi/4]

#path = CubicPolynomials(pos_ini, pos_fin)
#points = path.get_curve_points(20)

# Conectar no Vrep
clientID = connect_2_sim()
test_connection(clientID)

# Enviar pontos do Caminho Gerado
#send_points_to_sim([p[1:3] for p in points], clientID=clientID)

left_motor_handle, right_motor_handle = get_pioneer3DX_motor_handles_(clientID)


return_value_left_motor_control, return_value_right_motor_control = robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)

errorCode, robo = sim.simxGetObjectHandle(clientID=clientID, objectName="./PioneerP3DX",
                                          operationMode=sim.simx_opmode_blocking)

error_pos, pos_robo = sim.simxGetObjectPosition(clientID, robo, -1, sim.simx_opmode_streaming)
error_ang, ang_robo = sim.simxGetObjectOrientation(clientID, robo, -1, sim.simx_opmode_streaming)

print(robo)
print(pos_robo)
print(ang_robo)

#PONTOS FINAIS
lista_pontos = [[1.5,-1.5], [1.5,0], [1.5,1.5], 
                [-1.5,-1.5], [-1.5,0], [-1.5,1.5],
                [0,0]]

#Ganhos do controlador
k_theta = 0.25
k_l = 0.1

#Dados do robô
rd = 0.195/2
re = 0.195/2
B = 0.381




for ponto in lista_pontos:
    xf = ponto[0]
    yf = ponto[1]

    p = np.array([[xf,yf]])
    send_points_to_sim(p, clientID=clientID)
    print(f'Going to point {ponto} ')
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
    print(f'Time: {(stop - start).round(3)}s')
    robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)
    print(f'Arrived at point {ponto} with pos {[px.round(4) for px in np.array(pos_robo)]}')
