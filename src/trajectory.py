# this file contains the code needed for generating trajectories using Cubic Polynomials

import math
import numpy as np

class CubicPolynomials:
    def __init__(self, initial_config, final_config):
        """This class computes cubic polynomials coefficients for trajectory generation. The parameters are the initial and final configuration, with the angle in radians.

        Args:
            initial_config (list): configuration in the initial position (x_i,y_i,theta_i)
            final_config (list): configuration in the final position (x_f,y_f,theta_f)
        """
        self.initial_x = initial_config[0]
        self.initial_y = initial_config[1]
        self.initial_theta = initial_config[2]
        
        self.final_x = final_config[0]
        self.final_y = final_config[1]
        self.final_theta = final_config[2]

        self._compute_curve_parameters()
    
    def _compute_curve_parameters(self):
        """This method computes the curve parameters themselves, following the algorithm showed by Pablo.
        """

        delta_x = self.final_x - self.initial_x
        delta_y = self.final_y - self.initial_y
        small_delta = 0.0174533 # rad, approximatelly 1 deg
        final_alpha = math.tan(self.final_theta)
        initial_alpha = math.tan(self.initial_theta)

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
            print('both provided points near singularity')
        elif((self.initial_theta>math.pi/2-small_delta)
             and(self.initial_theta<math.pi/2+small_delta)):
            a3 = -delta_x/2
            b3 = 0 # qualquer valor
            a0 = self.initial_x
            a1 = 0
            a2 = 1.5*delta_x # delta_x - a3 = delta_x - - delta_x/2
            b0 = self.initial_y
            b1 = 2*(delta_y-final_alpha*delta_x) - final_alpha*a3 # + b3 = + 0
            b2 = 2*final_alpha*delta_x - delta_y + final_alpha*a3 # - 2*b3 = 0

            print('initial point near singularity')
        elif((self.final_theta>math.pi/2-small_delta)
             and(self.final_theta<math.pi/2+small_delta)):
            a1 = 1.5*delta_x
            b2 = 0
            a0 = self.initial_x
            a2 = 3*delta_x - 2*a1
            a3 = a1 - 2*delta_x
            b0 = self.initial_y
            b1 = initial_alpha*a1
            b3 = delta_y - initial_alpha*a1-b2
            print('final point near singularity')
        else:
            a1 = delta_x
            a2 = 0
            a0 = self.initial_x
            a3 = delta_x - a1 - a2
            b0 = self.initial_y
            b1 = initial_alpha*a1
            b2 = 3*(delta_y - final_alpha*delta_x) + 2*(final_alpha-initial_alpha)*a1 + final_alpha*a2
            b3 = 3*final_alpha*delta_x - 2*delta_y - (2*final_alpha-initial_alpha)*a1 - final_alpha*a2
            print('no point near singularity')

        self.x_curve_parameters = [a0,a1,a2,a3]
        self.y_curve_parameters = [b0,b1,b2,b3]

        print(f'x(lmb) = {a0} + {a1}*lmb + {a2}*lmb² + {a3}*lmb³')
        print(f'y(lmb) = {b0} + {b1}*lmb + {b2}*lmb² + {b3}*lmb³')
        print(f'theta(lmb) = atan( ({b1} + {2*b2}*lmb + {3*b3}*lmb²) / ({a1} + {2*a2}*lmb + {3*a3}*lmb²) )')

    def get_point(self, lambda_parameter):
        """Generates a point over the given trajectory.

        Args:
            lambda_parameter (float): polynomials' input, ranging from 0 to 1

        Returns:
            list: configuration [x,y,theta]
        """
        if lambda_parameter <= 1 and lambda_parameter >= 0:

            lambda_squared = lambda_parameter*lambda_parameter
            lambda_cubed = lambda_parameter*lambda_parameter*lambda_parameter
            
            x_lambda = self.x_curve_parameters[0] + self.x_curve_parameters[1]*lambda_parameter + self.x_curve_parameters[2]*lambda_squared+self.x_curve_parameters[3]*lambda_cubed
            y_lambda = self.y_curve_parameters[0] + self.y_curve_parameters[1]*lambda_parameter + self.y_curve_parameters[2]*lambda_squared+self.y_curve_parameters[3]*lambda_cubed

            alpha_denominator = self.x_curve_parameters[1] + 2*self.x_curve_parameters[2]*lambda_parameter+3*self.x_curve_parameters[3]*lambda_squared
            alpha_numerator = self.y_curve_parameters[1] + 2*self.y_curve_parameters[2]*lambda_parameter+3*self.y_curve_parameters[3]*lambda_squared

            if alpha_denominator == 0:
                if alpha_numerator<0:
                    theta_lambda = -math.pi/2
                else:
                    theta_lambda = math.pi/2
            else:
                alpha_lambda = alpha_numerator/alpha_denominator
                theta_lambda = math.atan(alpha_lambda)
        else:
            raise ValueError('lambda must be between 0 and 1')

        return [x_lambda, y_lambda, theta_lambda]

    def get_curve_points(self, how_many_points=10):
        # this function returns how_many_points in configuration form (position and orientation)
        # in a 2D numpy array
        """This method is used to compute the points' configuration over the polynomial curve. The amount
            of points over the polynomial curve is defined with how_many_points.

        Args:
            how_many_points (int, optional): How many points over the curve is required. Defaults to 10.

        Returns:
            points (numpy array): the configurations over the line. A numpy array, whose rows follow the
                                  pattern: [lambda, x, y, theta]
        """
        lambdas = np.linspace(0,1,how_many_points)
        points = []
        for lmb in lambdas:
            current_point = self.get_point(lmb)
            current_point.insert(0,lmb)
            points.append(current_point)

        return np.array(points)

    
