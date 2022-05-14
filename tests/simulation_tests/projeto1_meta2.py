import sys
import os

import numpy
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math

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