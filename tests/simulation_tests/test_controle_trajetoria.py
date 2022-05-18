import sys
import os

import timeit
import numpy as np
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math
from scipy import optimize

# Conectar no Vrep
clientID = connect_2_sim()
test_connection(clientID)

left_motor_handle, right_motor_handle = get_pioneer3DX_motor_handles_(clientID)

return_value_left_motor_control, return_value_right_motor_control = robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)

errorCode, robo = sim.simxGetObjectHandle(clientID=clientID, objectName="./PioneerP3DX",
                                          operationMode=sim.simx_opmode_blocking)

error_pos, pos_robo = sim.simxGetObjectPosition(clientID, robo, -1, sim.simx_opmode_streaming)
error_ang, ang_robo = sim.simxGetObjectOrientation(clientID, robo, -1, sim.simx_opmode_streaming)

print(f'{robo}\n{pos_robo}\n{ang_robo}')

# Ganhos do controlador
k_theta = 0.6
k_l = 50

# Dados do robô
rd = 0.195/2
re = 0.195/2
B = 0.381

# Velocidade 
v = 0.05

pos_ini = [0, 0, 0] 
pos_fin = [1.7,2.0,math.pi/4] 

# Gerar Caminho
path = CubicPolynomials(pos_ini, pos_fin)
points = path.get_curve_points(20)

a, b = path.x_curve_parameters, path.y_curve_parameters

# Enviar pontos do Caminho Gerado
send_points_to_sim([p[1:3] for p in points], clientID=clientID)

# Função de minimização de distancia
def f(lmb, path, point):
    return path.get_distance_between_points(lmb,point)

ErroL = []
while True:
    error_pos, pos_robo = sim.simxGetObjectPosition(clientID, robo, -1, sim.simx_opmode_buffer)
    error_ang, ang_robo = sim.simxGetObjectOrientation(clientID, robo, -1, sim.simx_opmode_buffer)

    lmb = optimize.fminbound(f, 0, 1, args=(path, pos_robo[0:2]))
    ponto_curva = path.get_point(lmb)
    Delta_l = path.get_distance_between_points(lmb, pos_robo[0:2])
    lmb = round(lmb, 4)

    Theta_robo = ang_robo[2]
    
    #raio de giro
    dx = a[1] + 2*a[2]*lmb + 3*a[3]*(lmb**2)
    dy = b[1] + 2*b[2]*lmb + 3*b[3]*(lmb**2)
    d2x = 2*a[2] + 6*a[3]*lmb
    d2y = 2*b[2] + 6*b[3]*lmb
    r = ((((dx**2)+(dy**2))**1.5)/((d2y*dx)-(d2x*dy)))              
    K = (1/r)

    #delta theta
    theta_SF = np.arctan(dy/dx)
    Delta_theta = Theta_robo - theta_SF

    #Garantir o sinal correto de Delta L 
    theta_posicao_robo = np.arctan2(pos_robo[1],pos_robo[0]) #angulo de posicao do robo
    theta_ref = np.arctan2(ponto_curva[1],ponto_curva[0]) #angulo da curva que está mais próxima ao robo

    if(theta_ref>theta_posicao_robo): #Sinal do delta L
        Delta_l = -Delta_l
    
    print(round(Delta_l,3), round(Delta_theta,3))

    #Controle
    u = -(k_theta*Delta_theta + (k_l*Delta_l*v*np.sin(Delta_theta)/Delta_theta))

    #Velocidade Angular
    w = u + ((K*v*np.cos(Delta_theta))/(1-(K*Delta_l)))

    #Velocidade das juntas
    wd = (v/rd) + (B/(2*rd))*w
    we = (v/re) - (B/(2*re))*w

    robot_run(clientID, left_motor_handle, right_motor_handle, we, wd)

    if lmb == 1:
        break 
    
                
robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)