import sys
import os

import numpy as np
from pynput import keyboard 

import time

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))

from utils import *

# Conectar no Vrep
clientID = connect_2_sim()
test_connection(clientID)

# Recuperar handlers do dr20
left_motor_handle, right_motor_handle = get_dr20_motor_handles_(clientID)

# getting sensor handle
sensor_reference = './dr20/ultrasonicSensorJoint_/ultrasonicSensor_'
err_code,ultras1 = sim.simxGetObjectHandle(clientID=clientID,
                                            objectName=sensor_reference,
                                            operationMode=sim.simx_opmode_blocking)

def move_safe(direction, clientID, left_motor_handle, right_motor_handle):
    if direction == 'f':
        print('Moving forward')
        robot_run(clientID, left_motor_handle, right_motor_handle,1,1)
    if direction == 'b':
        print('Moving backward')
        robot_run(clientID, left_motor_handle, right_motor_handle,-1,-1)
    if direction == 'l':
        print('Moving left')
        robot_run(clientID, left_motor_handle, right_motor_handle,-1,1)
    if direction == 'r':
        print('Moving right')
        robot_run(clientID, left_motor_handle, right_motor_handle,1,-1)
    if direction == 's':
        print("Stopping")
        robot_run(clientID, left_motor_handle, right_motor_handle,0,0)


# callback function when the key is pressed
def on_press(key):
    _, _, detected_point, _, _ = sim.simxReadProximitySensor(clientID=clientID, sensorHandle=ultras1, operationMode=sim.simx_opmode_blocking)
    distance = np.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
    print(f'distance = {distance}')
    if distance < 0.4:
        print(f'obstacle found ')
        move_safe('r', clientID, left_motor_handle, right_motor_handle)
        time.sleep(1)
        move_safe('s', clientID, left_motor_handle, right_motor_handle)
    try:
        if key.char == 'a':
            move_safe('l', clientID, left_motor_handle, right_motor_handle)
            print('Moving left')
        if key.char == 'd':
            move_safe('r', clientID, left_motor_handle, right_motor_handle)
            print('Moving right')
        if key.char == 'w':
            move_safe('f', clientID, left_motor_handle, right_motor_handle)
            print('Moving forward')
        if key.char == 's':
            move_safe('b', clientID, left_motor_handle, right_motor_handle)
            print('Moving backward')
        if key.char == 'q':
            move_safe('s', clientID, left_motor_handle, right_motor_handle)
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

# callback function when the key is released
def on_release(key):
    if key.char == 'a' or key.char == 'w' or key.char == 's' or key.char == 'd':
        move_safe('s', clientID, left_motor_handle, right_motor_handle)
    if key.char == 'q':
        stop_sim = True
        return False
    if key == keyboard.Key.esc:
        print(type(key))
        # Stop listener
        return False

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()