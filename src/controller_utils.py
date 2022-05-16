# this file contains all utilities related to controllers
import numpy as np

def get_metaparameters(goal_config, initial_config):
    vector_difference = goal_config - initial_config
    del_L_star_squared = np.power(vector_difference[:2],2)
    del_theta = vector_difference[2]
    del_L_star = np.sqrt(np.sum(del_L_star_squared))
    del_L = del_L_star*np.cos(del_theta)
    
    return del_L, del_theta