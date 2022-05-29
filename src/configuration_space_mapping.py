import numpy as np
from utils import *

def get_normals_orientation(euler_z_angle, polygon_type = 'cubic'):
    """This method returns the normals on a polygon's horizontal faces, considering one of the faces is aligned with the X axis.

    Args:
        euler_z_angle (float): the angle between the polygon's X axis and the global X axis.
        polygon_type (str, optional): the type of polygon to be considered. Defaults to 'cubic'.

    Raises:
        ValueError: raised when an unsupported polygon is requested
        ValueError: raised when the Z euler angle is not within [-2pi, 2pi]

    Returns:
        tuple: x_angle, y_angle, negative_x_angle, and negative_y_angle. 
    """
    if polygon_type != 'cubic':
        raise ValueError("only cubic polygons supported so far")
    
    pi = 3.14159265359
    if euler_z_angle > 2*pi or euler_z_angle < -2*pi:
        raise ValueError("Euler angle is out of limits")
    else:
        x_angle = euler_z_angle
        y_angle = euler_z_angle + pi/2
        negative_x_angle = euler_z_angle + pi
        negative_y_angle = euler_z_angle + pi/2 + pi

    return x_angle, y_angle, negative_x_angle, negative_y_angle

def rotation_matrix_from_euler_angles(phi=0, theta=0, psi=0, inverse = False): #angles in radians
    """Computes the transformation matrix R = R_z(phi)*R_y(theta)*R_x(psi) or inv(R). R is the rotation matrix associated with the euler angles phi, theta and psi. This matrix maps the object frame to the global frame.

    Args:
        phi (int, optional): rotation, in rad, around Z axis. Defaults to 0.
        theta (int, optional): rotation, in rad, around Y axis. Defaults to 0.
        psi (int, optional): rotation, in rad, around X axis. Defaults to 0.
        inverse (bool, optional): if the inverse matrix should be returned instead of the regular matrix. Defaults to False.

    Raises:
        RuntimeError: raised when the inverse matrix is chosen, but it has no inverse (it is calculated numerically).

    Returns:
        matrix (numpy NDarray): the rotation matrix R or its inverse (depending on the boolean inverse parameter)
    """
    r_11 = np.cos(theta)*np.cos(phi)
    r_12 = np.sin(psi)*np.sin(theta)*np.cos(phi)-np.cos(psi)*np.sin(phi)
    r_13 = np.cos(psi)*np.sin(theta)*np.cos(phi)+np.sin(psi)*np.sin(phi)
    r_21 = np.cos(theta)*np.sin(phi)
    r_22 = np.sin(psi)*np.sin(theta)*np.sin(phi)+np.cos(psi)*np.cos(phi)
    r_23 = np.cos(psi)*np.sin(theta)*np.sin(phi)-np.sin(psi)*np.cos(phi)
    r_31 = -np.sin(theta)
    r_32 = np.sin(psi)*np.cos(theta)
    r_33 = np.cos(psi)*np.cos(theta)
    matrix_list = [[r_11, r_12, r_13],
                   [r_21, r_22, r_23],
                   [r_31, r_32, r_33]]
    rotation_matrix = np.array(matrix_list)
    if inverse == True:
        if np.linalg.det(rotation_matrix) == 0:
            raise RuntimeError("Null Determinant")
        else:
            inverse_rotation_matrix = np.linalg.inv(rotation_matrix)
            return inverse_rotation_matrix
    else:
        return rotation_matrix
    
def get_global_position(point_in_robot_frame, euler_angles, frame_origin_position):
    """This function returns the global coordinates of a given point (described in object's reference frame). 

    Args:
        point_in_robot_frame (numpy ndarray): a given point in the objects's reference frame (same shape as frame_origin_position)
        euler_angles (list): the phi, theta, and psi (in this order) euler angles that map object's frame to global frame.
        frame_origin_position (numpy ndarray): the origin of object's reference frame (same shape as point_in_robot_frame), described in global frame.

    Returns:
        point_position (numpy ndarray): the given point, but in global reference frame (same shape as point_in_robot_frame and frame_origin_position).
    """
    phi = euler_angles[0]
    theta = euler_angles[1]
    psi = euler_angles[2]

    rotation_matrix = rotation_matrix_from_euler_angles(phi,theta,psi)

    point_in_global_frame = np.matmul(rotation_matrix,point_in_robot_frame)
    point_position = point_in_global_frame + frame_origin_position

    return point_position

def get_bounding_box_corners_local_coordinates(client_id, object_handle, parameter_id_type='model', consider_up_layer = False):

    """This method returns the coordinates of the bounding box corners, in the local reference frame. The order starts at the top right point and goes anti-clockwise until the down right point.

    Returns:
        tuple: vectors from point 1 to point 8
    """
    # the idea is to get the coordinates and return it anti-clockwise (to match mapping algorithm)
    x_min, y_min, z_min, x_max, y_max, z_max = get_bounding_box_corners_positions(client_id,object_handle,parameter_id_type)

    # down layer points
    point_1 = np.array([x_max,y_max,z_min])
    point_2 = np.array([x_min,y_max,z_min])
    point_3 = np.array([x_min,y_min,z_min])
    point_4 = np.array([x_max,y_min,z_min])
    
    if consider_up_layer:
        point_5 = np.array([x_max,y_max,z_max])
        point_6 = np.array([x_min,y_max,z_max])
        point_7 = np.array([x_min,y_min,z_max])
        point_8 = np.array([x_max,y_min,z_max])
        return point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8
    
    else:
        return point_1, point_2, point_3, point_4
