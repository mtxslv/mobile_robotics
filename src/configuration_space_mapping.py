import numpy as np
import pandas as pd
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
        tuple: y_angle, negative_x_angle, negative_y_angle, and x_angle . 
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

    return y_angle, negative_x_angle, negative_y_angle, x_angle

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

    Args:
        client_id (int): an ID related to the running simulation
        object_handle (int): an ID related to the simulated object whose bounding box's corners will be retrieved 
        parameter_id_type (str, optional): can be "model" or "object". Defaults to 'model'.
        consider_up_layer (boolean, optional): this indicates if the up layer points will be returned. Defaults to "False".

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

def map_local_coordinates_to_global_coordinates(local_coordinates, euler_angles, frame_origin_position):
    """This method takes coordinates in a local reference frame (robot, i.e.) and map them to the global reference frame.

    Args:
        local_coordinates (tuple): points in a local reference frame 
        euler_angles (list): the phi, theta, and psi (in this order) euler angles that map local frame to global frame.
        frame_origin_position (numpy ndarray): the origin of local reference frame (same shape as point_in_robot_frame), described in global frame.

    Returns:
        global_points (list): a list containing the points in the global frame. The order of points follow the same order as local_coordinates. 
    """
    global_points = []
    for local_point in local_coordinates:
        global_point = get_global_position(local_point,euler_angles,frame_origin_position)
        global_points.append(global_point)
    return global_points

def convert_angle_to_0_2pi_interval(angle): # https://stackoverflow.com/questions/58627711/get-angle-into-range-0-2pi-python
    new_angle = np.arctan2(np.sin(angle), np.cos(angle))
    if new_angle < 0:
        new_angle = abs(new_angle) + 2 * (np.pi - abs(new_angle))
    return new_angle

def is_angle_between(v_1,v,v_2):
    v_1 = convert_angle_to_0_2pi_interval(v_1)
    v = convert_angle_to_0_2pi_interval(v)
    v_2 = convert_angle_to_0_2pi_interval(v_2)

    if (v_1 < v and v < v_2) or (v_2 < v and v < v_1):  
        return True
    else:
        return False

def mapping_loop(dframe, corners_robot, corners_obstacle):
    vertices_list = []
    for it in range(0,dframe.shape[0]-1):
        next_different = dframe.iloc[it+1:,-1].ne(dframe.iloc[it,-1]).idxmax()
        #print(f'it = {it} | next dif = {next_different}')
        current_owner = dframe.iloc[it,-1]
        next_different_owner = dframe.iloc[next_different,-1]
        if current_owner != next_different_owner:
            if current_owner == 'r':
                a_index = dframe.iloc[it,1]
                b_index = dframe.iloc[next_different,1]
            else:
                a_index = dframe.iloc[next_different,1]
                b_index = dframe.iloc[it,1]
            print(f'{current_owner} -> {next_different_owner} | b{b_index}-a{a_index}')                
            new_point = corners_obstacle[b_index-1] - corners_robot[a_index-1]
            # vertices_list.append(np.round(new_point,2))
            vertices_list.append(new_point)

    # this accounts for the circle ending
    current_owner = dframe.iloc[-1,-1]
    next_different = dframe.iloc[0:-1,-1].ne(dframe.iloc[-1,-1]).idxmax()
    next_different_owner = dframe.iloc[next_different,-1]
    if current_owner != next_different_owner:
        if current_owner == 'r':
            a_index = dframe.iloc[-1,1]
            b_index = dframe.iloc[next_different,1]
        else:
            a_index = dframe.iloc[next_different,1]
            b_index = dframe.iloc[-1,1]
        print(f'{current_owner} -> {next_different_owner} | b{b_index}-a{a_index}')                
        new_point = corners_obstacle[b_index-1] - corners_robot[a_index-1]
        # vertices_list.append(np.round(new_point,2))
        vertices_list.append(new_point)
    return vertices_list


def mapping(client_id, scene_objects, robot_name):

    listazinha = get_scene_objects_info(client_id, scene_objects)
    info_list, robot_information = split_robot_from_info_list(listazinha,robot_name)


    # ROBOT STUFF
    # Now that I have the objects handlers, I need to get the robot handler so I can retrieve its global coordinates and normals
    local_c_robot = get_bounding_box_corners_local_coordinates(client_id, robot_information['object_handler'])

    position_robot = robot_information['object_position']
    orientation_robot = robot_information['object_orientation']
    orientation_robot.reverse() # notice that the orientation vector is reversed (theta is naturally the last, but we need to have it as the first value)

    global_coordin_corners_robot = map_local_coordinates_to_global_coordinates(local_c_robot, orientation_robot, position_robot)

    robot_handle = sim.simxGetObjectHandle(client_id, robot_name,sim.simx_opmode_blocking)
    normals_robot = get_normals_orientation(sim.simxGetObjectOrientation(client_id,robot_handle[1],-1,sim.simx_opmode_blocking)[1][2])

    
    temporary_normals_robot = []
    for normal_robot_element in normals_robot:
        temporary_normals_robot.append(convert_angle_to_0_2pi_interval(normal_robot_element))

    normals_robot = temporary_normals_robot
    
    print(f'points bounding box around robot: {global_coordin_corners_robot}')
    print(f'normals bounding box around robot: {normals_robot}')


    # mirroring the normal vectors
    negative_delocation = (np.pi,np.pi,np.pi,np.pi)
    normals_robot = [sum(x) for x in zip(normals_robot,negative_delocation)]

    temporary_normals_robot = []
    for normal_robot_element in normals_robot:
        temporary_normals_robot.append(convert_angle_to_0_2pi_interval(normal_robot_element))

    normals_robot = temporary_normals_robot

    print(f'inverted normals bounding box around robot: {normals_robot}')
    print(" ")
    # OBSTACLE STUFF, FOR 1 OBSTACLE ONLY
    # I need to retrieve the global coordinates and normals of an obstacle. Let's suppose the Cuboid_0 for this test

    cuboid_number = 0

    # OBSTACLE

    local_c_cuboid = get_bounding_box_corners_local_coordinates(client_id, listazinha[cuboid_number]['object_handler'])

    orientation_cuboid = listazinha[cuboid_number]['object_orientation']
    orientation_cuboid.reverse() # notice that the orientation vector is reversed (theta is naturally the last, but we need to have it as the first value)
    position_cuboid = listazinha[cuboid_number]['object_position']

    global_coordinates_cuboid = map_local_coordinates_to_global_coordinates(local_c_cuboid, orientation_cuboid, position_cuboid)

    normals_cuboid = get_normals_orientation(listazinha[cuboid_number]['object_orientation'][2])

    temporary_normals_cuboid = []
    for normal_cuboid_element in normals_cuboid:
        temporary_normals_cuboid.append(convert_angle_to_0_2pi_interval(normal_cuboid_element))

    normals_cuboid = temporary_normals_cuboid

    print(f'points bounding box around cuboid0: {global_coordinates_cuboid}')
    print(f'normals bounding box around cuboid0: {normals_cuboid}')

    # I need to check if a given normal is between other two, right? Then I can use a table or something to do so
    # How? On one column I gonna put the normals of the robot and of the obstacle.
    # On other column I'll indicate the owner of the normal: robot or obstacle
    # On the third column I'll indicate the normal order (first normal: right face, second one: up face, etc)
    # Now I can order the rows according to the normal order: increasing
    # Then I will scan the lines to see if a given normal lies between other two
    normal_robot_series = pd.Series(normals_robot, name='normals')
    normal_robot_name_series = pd.Series([1,2,3,4], name= 'normal_order')
    normal_cuboid_series = pd.Series(normals_cuboid, name='normals')
    normal_cuboid_name_series = pd.Series([1,2,3,4], name= 'normal_order')

    normals_series = pd.concat([normal_robot_series, normal_cuboid_series], ignore_index=True)
    normals_order = pd.concat([normal_robot_name_series, normal_cuboid_name_series], ignore_index=True)

    dframe_normals = pd.DataFrame({'normals': normals_series, 'normal_order': normals_order, 'owner':['r','r','r','r','o','o','o','o']})
    dframe_normals.sort_values(by='normals', inplace=True)
    print(" ")
    print(dframe_normals)
    print(" ")
    
    vertices_list = mapping_loop(dframe=dframe_normals, corners_robot=global_coordin_corners_robot,corners_obstacle=global_coordinates_cuboid)
    return vertices_list