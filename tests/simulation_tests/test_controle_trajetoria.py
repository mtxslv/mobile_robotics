from cmath import cos
import sys
import os

import time
import timeit
import numpy as np
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from lemniscate import *
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
error_ang, v_robo, w_robo = sim.simxGetObjectVelocity(clientID, robo, sim.simx_opmode_streaming)

print(f'{robo}\n{pos_robo}\n{ang_robo}')

# Gerar Caminho
r_x=2
r_y=2
number_of_points=50
lem = Lemniscate(r_x,r_y,1/60,1/60)
lem_position, lem_velocity, lem_acceleration, lem_orientation = lem.get_curve_points(number_of_points)

send_points_to_sim(lem_position, clientID=clientID)

# Ganhos do controlador
k_d, k_p = 0.001, 0.01

# Dados do robô
rd = 0.195/2
re = 0.195/2
B = 0.381

# Velocidade 
vc = 0.05

start_t = time.time()
current_t = time.time() - start_t
print(current_t)
#print( lem.get_point(current_t) )
tmax = 20


while current_t <= tmax:
    error_pos, pos_robo = sim.simxGetObjectPosition(clientID, robo, -1, sim.simx_opmode_buffer)
    error_ang, ang_robo = sim.simxGetObjectOrientation(clientID, robo, -1, sim.simx_opmode_buffer)
    error_ang, v_robo, w_robo = sim.simxGetObjectVelocity(clientID, robo, sim.simx_opmode_buffer)
    

    current_t = time.time() - start_t
    
    lem_p, lem_v, lem_a, lem_orientation = lem.get_point(current_t)
    print([i.round(3) for i in lem.get_point(current_t)])

    delta_vx = lem_v[0] - v_robo[0]
    delta_vy = lem_v[1] - v_robo[1]
    
    delta_px = lem_p[0] - pos_robo[0]
    delta_py = lem_p[1] - pos_robo[1]
    
    # Realimentação PD
    x2c = lem_a[0] + k_d*delta_vx + k_p*delta_px
    y2c = lem_a[1] + k_d*delta_vy + k_p*delta_py

    theta = ang_robo[2]
    v_r = np.sqrt((v_robo[0]**2) + (v_robo[1]**2))

    # Compensação do modelo não linear
    v1c = np.cos(theta)*x2c + np.sin(theta)*y2c
    # = -(np.sin(theta)/v_r)*x2c + (np.cos(theta)/v_r)*y2c
    wc = (y2c * np.cos(theta) - x2c * np.sin(theta)) / (v_r + 0.001)
    

    # Integração
    vc = vc + v1c

    #Velocidade das juntas
    wd = (vc/rd) + (B/(2*rd))*wc
    we = (vc/re) - (B/(2*re))*wc

    
    print(round(x2c,3), round(y2c,3), round(v_r,3), round(theta,3))
    print(round(v1c,3), round(vc,3) , round(wc,3))
    print(round(we,3), round(wd,3))
    print('\n')

    robot_run(clientID, left_motor_handle, right_motor_handle, we, wd)


robot_run(clientID, left_motor_handle, right_motor_handle, 0, 0)