# this file contains the code needed for generating trajectories using Cubic Polynomials

import math

class CubicPolynomials:
    def __init__(self, initial_config, final_config):
        self.initial_x = initial_config[0]
        self.initial_y = initial_config[1]
        self.initial_theta = initial_config[2]
        
        self.final_x = final_config[0]
        self.final_y = final_config[1]
        self.final_theta = final_config[2]

        self.generate_curve_parameters()
    
    def generate_curve_parameters(self):
        delta_x = self.final_x - self.initial_x
        delta_y = self.final_y - self.initial_y
        small_delta = 0.0174533 # rad, approximatelly 1 deg

        if((self.initial_theta>math.pi/2-small_delta and self.initial_theta<math.pi/2+small_delta)
            and(self.final_theta>math.pi/2-small_delta and self.final_theta<math.pi/2+small_delta)):
            b1 = delta_y
            b2 = 0
            a0 = self.initial_x
            a1 = 0
            a2 = 3*delta_x
            a3 = -2*delta_x
            b0 = self.initial_y
            b3 = 0 # delta_y - b1 - b2 = delta_y - delta_y - 0
            print('first option')
        elif((self.initial_theta>math.pi/2-small_delta)
             and(self.initial_theta<math.pi/2+small_delta)):
            a3 = -delta_x/2
            b3 = 0 # qualquer valor
            a0 = self.initial_x
            a1 = 0
            a2 = 1.5*delta_x # delta_x - a3 = delta_x - - delta_x/2
            b0 = self.initial_y

            final_alpha = (b1+2*b2+3*b3)/(a1+2*a2+3*a3) #(b1+2*b2+3*b3)/(a1+2*a2+3*a3)

            print('second option')
        elif((self.final_theta>math.pi/2-small_delta)
             and(self.final_theta<math.pi/2+small_delta)):
            print('third option')
        else:
            print('fourth option')
            