# this class create a lemniscate curve

import math
import numpy as np

class Lemniscate:
    def __init__(self, a_x, a_y, f_x=1/30, f_y=1/30):
        """This class is used to compute a lemniscate curve. It ranges from -a_x to a_x horizontally, 
            and from -a_y to a_y vertically. The movement of a particle over the curve is such that 
            it returns to its original horizontal coordinate in 1/f_x units of time, and to its original
            vetical coordinate in 1/f_y units of time.

        Args:
            a_x (float): The X amplitude (how far the curve goes horizontally)
            a_y (float): The Y amplitude (how far the curve goes vertically)
            f_x (float): The frequency (cicles/second) of the X component. Ideally it equals f_y. (Default:1/30)
            f_y (float): The frequency (cicles/second) of the Y component. Ideally it equals f_x. (Default:1/30)

        Raises:
            ValueError: raised when a_x is not inside the interval (0 , 2.5]
            ValueError: raised when a_y is not inside the interval (0 , 2.5]
        """
        
        if a_x > 0 and a_x <= 2.5:
            self.a_x = a_x
        else:
            raise ValueError('a_x must be greater than 0 and less than 2.5')
        if a_y > 0 and a_y <= 2.5:
            self.a_y = a_y
        else:
            raise ValueError('a_y must be greater than 0 and less than 2.5')    
        if f_x <=0:        
            raise ValueError('f_x must be greater than 0')
        else:
            self.f_x = f_x
        if f_y <=0:
            raise ValueError('f_y must be greater than 0')
        else:
            self.f_y = f_y
                
    def get_curve_points(self, how_many_points=50):
        """Return the position, velocity, acceleration and orientation over the lemniscate curve.

        Args:
            how_many_points (int, optional): How many points over the curve will be generated. Defaults to 20.

        Returns:
            tuple: a tuple of numpy arrays, regarding (in this order) the:  position, velocity, acceleration and 
            orientation over the lemniscate curve. For position, velocity and acceleration, each array row is a X,Y point.
            For orientation, each row represents the position's orientation (in rad).
        """
        stop_value = min([1/self.f_x,1/self.f_y])
        domain = np.linspace(0,stop_value,how_many_points)

        lemniscate = np.array( (self.a_x*np.cos(2*np.pi*self.f_x*domain), self.a_y*np.sin(2*2*np.pi*self.f_y*domain)) ).T
        lemniscate_velocity = np.array( (-self.a_x*2*np.pi*self.f_x*np.sin(2*np.pi*self.f_x*domain), 2*self.a_y*2*np.pi*self.f_y*np.cos(2*2*np.pi*self.f_y*domain)) ).T
        lemniscate_acceleration = np.array( (-self.a_x*4*np.power(np.pi*self.f_x,2)*np.cos(2*np.pi*self.f_x*domain), -self.a_y*16*np.power(np.pi*self.f_x,2)*np.sin(2*2*np.pi*self.f_y*domain)) ).T

        lemniscate_orientation = np.arctan2(lemniscate_velocity[:,1],lemniscate_velocity[:,0])

        return lemniscate, lemniscate_velocity, lemniscate_acceleration, lemniscate_orientation