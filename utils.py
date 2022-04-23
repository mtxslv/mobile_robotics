"""
This file will contain all functions needed for handling the robot, 
such that sim.py will not be imported directly. Furthermore, these 
utilities aim to ease the development process once the project's 
goals advance.
"""

# python native libs
import time
import sys

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
            print("pal, I believe you wrote the element address. Did you put ./ at the beginning? ")
    else:
        print("motor handles successfully retrieved!")
    return left_motor_handle,right_motor_handle