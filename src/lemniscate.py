# this class create a lemniscate curve

import math
import numpy as np

class Lemniscate:
    def __init__(self, a_x, a_y):
        """This class is used to compute a lemniscate curve.

        Args:
            a_x (float): The X amplitude (how far the curve goes horizontally)
            a_y (float): The Y amplitude (how far the curve goes vertically)

        Raises:
            ValueError: raised when a_x is not inside the interval (0 , 2.5]
            ValueError: raised when a_y is not inside the interval (0 , 2.5]
        """
        if a_x > 0 and a_x <= 2.5:
            self.a_x = a_x
        else:
            raise ValueError('r_x must be greater than 0 and less than 2.5')
        if a_y > 0 and a_y <= 2.5:
            self.a_y = a_y
        else:
            raise ValueError('r_y must be greater than 0 and less than 2.5')            
    
    def get_curve_points(self, how_many_points=20):
        """Return the position, velocity, acceleration and orientation over the lemniscate curve.

        Args:
            how_many_points (int, optional): How many points over the curve will be generated. Defaults to 20.

        Returns:
            tuple: a tuple of numpy arrays, regarding (in this order) the:  position, velocity, acceleration and 
            orientation over the lemniscate curve. For position, velocity and acceleration, each array row is a X,Y point.
            For orientation, each row represents the position's orientation (in rad).
        """

        domain = np.linspace(0,2*math.pi,how_many_points)

        lemniscate = np.array( (self.r_x*np.cos(domain), self.r_y*np.sin(2*domain)) ).T
        lemniscate_velocity = np.array( (-self.r_x*np.sin(domain), 2*self.r_y*np.cos(2*domain)) ).T
        lemniscate_acceleration = np.array( (-self.r_x*np.cos(domain), -4*self.r_y*np.sin(2*domain)) ).T

        lemniscate_orientation = np.arctan2(lemniscate_velocity[:,1],lemniscate_velocity[:,0])

        return lemniscate, lemniscate_velocity, lemniscate_acceleration, lemniscate_orientation