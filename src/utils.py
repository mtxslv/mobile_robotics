"""
This file will contain all functions needed for handling the robot, 
such that sim.py will not be imported directly. Furthermore, these 
utilities aim to ease the development process once the project's 
goals advance.
"""

# python native libs
import time
import sys
import ctypes

# 3rd party software
import sim # simulation lib

def connect_2_sim():
    sim.simxFinish(-1) # just in case, close all opened connections

    clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

    if clientID != -1:
        print('Connected to remote API server')
    else:
        print("damn it boi... didnt work :( ")
        sys.exit("Could not connect")
    return clientID

def test_connection(clientID):
    # this method aims to check if the connection is ok. It shows how many objects are on the scene
    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

def get_pioneer3DX_motor_handles_(clientID):
    # let's take the robot handles
    error_code_left_motor, left_motor_handle =  sim.simxGetObjectHandle(clientID= clientID, 
                                                                            objectName="./leftMotor",
                                                                            operationMode=sim.simx_opmode_blocking)
    error_code_right_motor, right_motor_handle =  sim.simxGetObjectHandle(clientID= clientID, 
                                                                            objectName="./rightMotor",
                                                                            operationMode=sim.simx_opmode_blocking)                                                                            
    if error_code_left_motor!=0 or error_code_right_motor!=0:
        print("dude! You've got an error on the motors!")
        if error_code_left_motor == 8 or error_code_right_motor==8:
            print("pal, I believe you only wrote the element address. Did you put ./ at the beginning? ")
    else:
        print("motor handles successfully retrieved!")
    return left_motor_handle,right_motor_handle

def get_robot_motor_handles_(clientID, left_motor_path, right_motor_path):
    # let's take the robot handles
    error_code_left_motor, left_motor_handle =  sim.simxGetObjectHandle(clientID= clientID, 
                                                                            objectName=left_motor_path,
                                                                            operationMode=sim.simx_opmode_blocking)
    error_code_right_motor, right_motor_handle =  sim.simxGetObjectHandle(clientID= clientID, 
                                                                            objectName=right_motor_path,
                                                                            operationMode=sim.simx_opmode_blocking)                                                                            
    if error_code_left_motor!=0 or error_code_right_motor!=0:
        print("dude! You've got an error on the motors!")
        if error_code_left_motor == 8 or error_code_right_motor==8:
            print("pal, I believe you only wrote the element address. Did you put ./ at the beginning? ")
    else:
        print("motor handles successfully retrieved!")
    return left_motor_handle,right_motor_handle

def robot_run(clientID, left_motor_handle, right_motor_handle, left_target_velocity, right_target_velocity):
    return_value_left_motor_control = sim.simxSetJointTargetVelocity(clientID=clientID,jointHandle=left_motor_handle,
                                                                targetVelocity=left_target_velocity,operationMode=sim.simx_opmode_blocking)
    return_value_right_motor_control = sim.simxSetJointTargetVelocity(clientID=clientID,jointHandle=right_motor_handle,
                                                                targetVelocity=right_target_velocity,operationMode=sim.simx_opmode_blocking)
                                                                    
    return return_value_left_motor_control,return_value_right_motor_control

def send_points_to_sim(points, clientID, sleep_time = 0.07):
    #the bigger the sleep time the more accurate the points are 
    #placed but you have to be very patient :D
    i = 0
    print('Sending Points ...')
    for p in points:
        packedData=sim.simxPackFloats(p.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        
        returnCode = sim.simxWriteStringStream(clientID, "point_coord", raw_bytes, sim.simx_opmode_oneshot)
        if returnCode != 0 and returnCode != 1:
            print(f'Point {p.round(3)} not sent. Error {returnCode}')
        else:
            i = i + 1
        time.sleep(sleep_time)
    print(f'Points sent: {i}')

def get_bounding_box_corners_positions(client_id, object_handle, parameter_id_type='model'):
    """This method returns the coordinates (on the robot's reference frame) of the two extreme corners of the bounding box.

    Args:
        client_id (int): an ID related to the running simulation
        object_handle (int): an ID related to the simulated object whose bounding box's corners will be retrieved 
        parameter_id_type (str, optional): can be "model" or "object". Defaults to 'model'.

    Raises:
        RuntimeError: this error is raised when any of the error codes is not zero (returned when the corners are retrieved).
        RuntimeError: this error is raised when any of the error codes is not zero (returned when the corners are retrieved). 
        ValueError: this error is raised when the parameter_id_type is not one of the two types showed.

    Returns:
        tuple: the coordinates of the two extreme corners of the bounding box (min_x, min_y, min_z, max_x, max_y, max_z).
    """
    if parameter_id_type == 'model':
        error_min_x, min_x = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_modelbbox_min_x,
                                operationMode=sim.simx_opmode_blocking)
        error_min_y, min_y = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_modelbbox_min_y,
                                operationMode=sim.simx_opmode_blocking)
        error_min_z, min_z =  sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_modelbbox_min_z,
                                operationMode=sim.simx_opmode_blocking)
        error_max_x, max_x = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_modelbbox_max_x,
                                operationMode=sim.simx_opmode_blocking)
        error_max_y, max_y = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_modelbbox_max_y,
                                operationMode=sim.simx_opmode_blocking)
        error_max_z, max_z = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_modelbbox_max_z,
                                operationMode=sim.simx_opmode_blocking)
        if error_min_x+error_min_y+error_min_z+error_max_x+error_max_y+error_max_z != 0:
            raise RuntimeError('an inexpected error occurred when corners were retrieved')
    elif parameter_id_type == 'object':
        error_min_x, min_x = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_objbbox_min_x,
                                operationMode=sim.simx_opmode_blocking)
        error_min_y, min_y = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_objbbox_min_y,
                                operationMode=sim.simx_opmode_blocking)
        error_min_z, min_z =  sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_objbbox_min_z,
                                operationMode=sim.simx_opmode_blocking)
        error_max_x, max_x = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_objbbox_max_x,
                                operationMode=sim.simx_opmode_blocking)
        error_max_y, max_y = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_objbbox_max_y,
                                operationMode=sim.simx_opmode_blocking)
        error_max_z, max_z = sim.simxGetObjectFloatParameter(clientID=client_id,
                                objectHandle= object_handle,
                                parameterID=sim.sim_objfloatparam_objbbox_max_z,
                                operationMode=sim.simx_opmode_blocking)
        if error_min_x+error_min_y+error_min_z+error_max_x+error_max_y+error_max_z != 0:
            raise RuntimeError('an inexpected error occurred when corners were retrieved')
    else:
        raise ValueError('Invalid parameter')
    return min_x, min_y, min_z, max_x, max_y, max_z