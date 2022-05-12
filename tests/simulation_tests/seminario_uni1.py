# this code aims to control some robot movement.

import sys
import os
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))
from utils import *

clientID = connect_2_sim()
test_connection(clientID)
left_motor_handle, right_motor_handle = get_pioneer3DX_motor_handles_(clientID)

# getting sensor handle
err_code,ultras1 = sim.simxGetObjectHandle(clientID=clientID,
                                            objectName='./visible/ultrasonicSensor[14]',
                                            operationMode=sim.simx_opmode_blocking)

print(f'ultrassonic sensor error code: {err_code}')

# first sensoring 
detection_state, detection_state_bool, detected_point, distance, normal_surface = sim.simxReadProximitySensor(clientID=clientID, sensorHandle=ultras1, operationMode=sim.simx_opmode_blocking)
print(f'Has detected something? {detection_state_bool}')
if detection_state_bool:
    print(f'distance={distance}')


# moving the robot onward and backward

return_value_left_motor_control, return_value_right_motor_control = robot_run(
    clientID, left_motor_handle, right_motor_handle, 1, 1)
print(
    f'return_value_left_motor_control == {return_value_left_motor_control} and return_value_right_motor_control == {return_value_right_motor_control}')
time.sleep(5)
return_value_left_motor_control, return_value_right_motor_control = robot_run(
    clientID, left_motor_handle, right_motor_handle, -1, -1)
print(
    f'return_value_left_motor_control == {return_value_left_motor_control} and return_value_right_motor_control == {return_value_right_motor_control}')
time.sleep(5)
return_value_left_motor_control, return_value_right_motor_control = robot_run(
    clientID, left_motor_handle, right_motor_handle, 0, 0)
print(
    f'return_value_left_motor_control == {return_value_left_motor_control} and return_value_right_motor_control == {return_value_right_motor_control}')
time.sleep(5)
return_value_left_motor_control, return_value_right_motor_control = robot_run(
    clientID, left_motor_handle, right_motor_handle, -1, 1)
time.sleep(5)
return_value_left_motor_control, return_value_right_motor_control = robot_run(
    clientID, left_motor_handle, right_motor_handle, 0, 0)

# second sensoring 
detection_state, detection_state_bool, detected_point, distance, normal_surface = sim.simxReadProximitySensor(clientID=clientID, sensorHandle=ultras1, operationMode=sim.simx_opmode_blocking)
print(f'Has detected something? {detection_state_bool}')
if detection_state_bool:
    print(f'distance={distance}')