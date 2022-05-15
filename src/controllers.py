# this file will contain the controllers
import numpy as np
from pid import *

# POSITION CONTROLLER
class UncoupledController:
    def __init__(self, delL_controller_params, delTH_controller_params, uncoupling_matrix):
        """Creates the Delta-L and Delta-Theta controllers.

        Args:
            delL_controller_params (np.NDarray): [kp,ki,kd] parameters for L controller
            delTH_controller_params (np.NDarray): [kp,ki,kd] parameters for Theta controller
            uncoupling_matrix (np.NDarray): the uncoupling matrix, that maps [u_v, u_omega] to [omega_r, omega_l]
        """

        self.delL_controller_params = delL_controller_params
        self.delTH_controller_params = delTH_controller_params
        self.uncoupling_matrix = uncoupling_matrix

        self.delta_L_controller = pid(self.delL_controller_params[0],
                                      self.delL_controller_params[1],
                                      self.delL_controller_params[2])
        
        self.delta_Th_controller = pid(self.delTH_controller_params[0],
                                      self.delTH_controller_params[1],
                                      self.delTH_controller_params[2])
   
    def control(self, delta_L, delta_Theta):
        """Updates the inner controllers, and maps the virtual outputs to wheels speed.

        Args:
            delta_L (float): linear error delta L
            delta_Theta (float): angular error delta theta

        Returns:
            np.NDArray: Wheels speed. First row: right wheel. Second row: left wheel.
        """
        u_v = self.delta_L_controller.control(delta_L)
        u_omega = self.delta_Th_controller.control(delta_Theta)
        self.virtual_inputs = np.array([[u_v],[u_omega]])
        self.wheels_speed = np.matmul(self.uncoupling_matrix,self.virtual_inputs) 
        return self.wheels_speed