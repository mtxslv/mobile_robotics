import sys
import os

import timeit
import numpy as np
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *
from trajectory import CubicPolynomials
import math
import shapely.geometry as geom
from scipy import optimize

# Gerar Caminho
pos_ini = [0,0,0]
pos_fin = [2.5,2.5,math.pi/4]

path = CubicPolynomials(pos_ini, pos_fin)
points = path.get_curve_points(20)

a, b = path.x_curve_parameters, path.y_curve_parameters

line = geom.LineString([p[1:3] for p in points])
point = geom.Point(10, 10)

# Note that "line.distance(point)" would be identical
print(point.distance(line))
print(path.get_point(1))

def f(lmb, path, point):
    return path.get_distance_between_points(lmb,point)

m_lmb = optimize.fminbound(f, 0, 1, args=(path, [10,10]))
print(m_lmb)

print(path.get_point(m_lmb))

print(path.get_distance_between_points(m_lmb,[10,10]))
