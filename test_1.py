# this code aims to control some robot movement.

import time
import sim 
import sys

sim.simxFinish(-1) # just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')
else:
    print("damn it boi... didnt work :( ")
    sys.exit("Could not connect")

# Now try to retrieve data in a blocking fashion (i.e. a service call):
res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
if res==sim.simx_return_ok:
    print ('Number of objects in the scene: ',len(objs))
else:
    print ('Remote API function call returned with error code: ',res)
    
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