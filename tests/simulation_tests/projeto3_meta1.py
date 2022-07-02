import sys
import os

from pynput import keyboard 

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


# callback function when the key is pressed
def on_press(key):
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
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

# callback function when the key is released
def on_release(key):
    if key == keyboard.Key.esc:
        print(type(key))
        # Stop listener
        return False

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()


while True:
    detection_state, detection_state_bool, detected_point, distance, normal_surface = sim.simxReadProximitySensor(clientID=clientID, sensorHandle=ultras1, operationMode=sim.simx_opmode_blocking)
    print(distance)
    if detection_state_bool:
        move_safe('r', clientID, left_motor_handle, right_motor_handle)
        print(f'obstacle found')

