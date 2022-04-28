# this code aims to control some robot movement.

import sys
import os
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../src/')))

clientID = connect_2_sim()
test_connection(clientID)
left_motor_handle, right_motor_handle = get_pioneer3DX_motor_handles_(clientID)

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
