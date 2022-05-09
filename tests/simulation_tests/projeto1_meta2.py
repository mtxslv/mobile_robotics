import sys
import os

import numpy
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math
import ctypes

def send_points_to_sim(points, clientID, sleep_time = 0.07):
    #the bigger the sleep time the more accurate the points are 
    #placed but you have to be very patient :D
    for p in points:
        packedData=sim.simxPackFloats(p.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        
        returnCode = sim.simxWriteStringStream(clientID, "point_coord", raw_bytes, sim.simx_opmode_oneshot)
        if returnCode != 0 and returnCode != 1:
            print(f'Point {p.round(3)} not sent. Error {returnCode}')
        else:
            print(p)
        time.sleep(sleep_time)

# Gerar Caminho
pos_ini = [0,0,0]
pos_fin = [2.5,2.5,math.pi/4]

path = CubicPolynomials(pos_ini, pos_fin)
points = path.get_curve_points(200)

# Conectar no Vrep
clientID = connect_2_sim()
test_connection(clientID)

# Enviar pontos do Caminho Gerado
send_points_to_sim([p[1:3] for p in points], clientID=clientID)