# this code aims to control some robot movement.

from utils import *


clientID = connect_2_sim()
test_connection(clientID)
left_motor_handle, right_motor_handle = get_pioneer3DX_motor_handles_(clientID)

# moving the robot ahead

t = time.time()
current_time = t
while(current_time-t < 5):
    current_time = time.time()
    print(f'it has passed = {current_time-t} secs')
    return_value_left_motor_control = sim.simxSetJointTargetVelocity(clientID=clientID,jointHandle=left_motor_handle,
                                                                    targetVelocity=1,operationMode=sim.simx_opmode_streaming)
    return_value_right_motor_control = sim.simxSetJointTargetVelocity(clientID=clientID,jointHandle=right_motor_handle,
                                                                    targetVelocity=1,operationMode=sim.simx_opmode_streaming)