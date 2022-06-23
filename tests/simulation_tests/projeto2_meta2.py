# esse arquivo faz o robô se mexer pelo parte do caminho que está ok
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


def test_magic_move():

    # MANDAR O ROBÔ MAGICAMENTE PROS PONTOS ACHADOS
    for point in lista_pontos:
        ponto_pra_ir = [ point[0], point[1], position_vector[2] ]
        sim.simxSetObjectPosition(clientID, robo, -1, ponto_pra_ir, sim.simx_opmode_blocking)
        print(f' chegou em {ponto_pra_ir}')

def test_move_controlled():

    #MANDAR O ROBÔ PRO PONTO INICIAL DA SEQUÊNCIA
    ponto_pra_ir = [ lista_pontos[0][0], lista_pontos[0][1], position_vector[2] ]
    sim.simxSetObjectPosition(clientID,robo, -1, ponto_pra_ir, sim.simx_opmode_blocking)
    print(f'robo colocado em {ponto_pra_ir}')

    #Ganhos do controlador
    k_theta = 0.1
    k_l = 0.05

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


if __name__ == "__main__":
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

    pontos_caminho = [[-2.26249975032426, 0],
     [-2.02499950064851, 0.887500227864903],
     [-1.99999964384828, 0.887500227864903],
     [-1.72499978704806, -0.362499591606807], 
     [-1.24999964384828, 0.887500227466864], # mandar o robô pra cá e começar a partir daqui, i=4
     [-1.02499950064851, 0],
     [-0.937499808777136, 0],
     [-0.850000116905762, -0.750000307530566],
     [-0.624999807956711, -0.750000307530566],
     [-0.399999499007659, -1.42499978773592],
     [-0.149999499007659, -1.42499978773592],
     [0.100000500992341, -0.750000308306786],
     [0.125000192043289, -0.750000308306786],
     [0.149999883094238, 0],
     [0.524999928677615, 0],
     [0.899999974260992, -1.57499994848484]]

    lista_pontos = pontos_caminho[4:]

    #lista_pontos = [[1.5,-1.5], [1.5,0], [1.5,1.5], 
    #                [-1.5,-1.5], [-1.5,0], [-1.5,1.5],
    #                [0,0]]

    position_error, position_vector = sim.simxGetObjectPosition(clientID,robo,-1, sim.simx_opmode_blocking)
    #print(position_vector)

    #MANDAR O ROBÔ PRO PONTO INICIAL DA SEQUÊNCIA
    #ponto_pra_ir = [ lista_pontos[4][0], lista_pontos[4][1], position_vector[2] ]
    #sim.simxSetObjectPosition(clientID,robo, -1, ponto_pra_ir, sim.simx_opmode_blocking)

    teste_a_ser_realizado = 'magic' # MODIFICAR AQUI

    if teste_a_ser_realizado == 'magic':
        test_magic_move()
    elif teste_a_ser_realizado == 'control':
        test_move_controlled()