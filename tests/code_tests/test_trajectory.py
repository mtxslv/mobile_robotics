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
import math
from scipy import optimize


r_x=2
r_y=2
number_of_points=50
lem = Lemniscate(r_x,r_y,1/600,1/600)
lem_position, lem_velocity, lem_acceleration, lem_orientation = lem.get_curve_points(number_of_points)

start_t = time.time()
current_t = time.time() - start_t
print(current_t)
print( lem.get_point(current_t) )

while current_t <= 2:
    current_t = time.time() - start_t
    lem_p, lem_v, lem_a, lem_orientation = lem.get_point(current_t)
    print(current_t, [i.round(2) for i in lem.get_point(current_t)])
    time.sleep(.2)