import sys
import os

import numpy
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math
import ctypes

def send_points_to_sim(points, clientID, sleep_time = 0.07, ):
    #the bigger the sleep time the more accurate the points are 
    #placed but you have to be very patient :D
    #for i in [numpy.array([1.02121,1.12356])]:
    for i in points:
        i = i[1:3]
        print(i)
        packedData=sim.simxPackFloats(i.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        
        returnCode = sim.simxWriteStringStream(clientID, "point_coord", raw_bytes, sim.simx_opmode_oneshot)
        if returnCode != 0 and returnCode != 1:
            print(f'Point {i.round(3)} not sent. Error {returnCode}')
        time.sleep(sleep_time)

# Gerar Caminho
pos_ini = [0,0,0]
pos_fin = [2.5,2.5,math.pi/4]

path = CubicPolynomials(pos_ini, pos_fin)
points = path.get_curve_points(200)

# Conectar no Vrep
clientID = connect_2_sim()
test_connection(clientID)

send_points_to_sim(points, clientID=clientID)