import time
import math
import sys
sys.path.append('./VREP_remoteAPIs')

''' Inverse kinematics '''
def chassisInverseKinematics(vx, vy, omega, wheel_R, a, b):
    omega_1 = (vy - vx + (a+b)*omega)/wheel_R
    omega_2 = (vy + vx - (a+b)*omega)/wheel_R
    omega_3 = (vy - vx - (a+b)*omega)/wheel_R
    omega_4 = (vy + vx + (a+b)*omega)/wheel_R
    
    # set the direction for each wheel
    v_wheel = [0,0,0,0]
    v_wheel[0] = -omega_1
    v_wheel[1] = -omega_2
    v_wheel[2] = -omega_3
    v_wheel[3] = -omega_4

    return v_wheel

''' Arm control function of youBot '''
def VREP_armControl(vrep_sim, clientID, arm_joints_handle, desired_arm_joint_angles):
    for i in range(0,5):
        vrep_sim.simxSetJointPosition(clientID, arm_joints_handle[i], desired_arm_joint_angles[i], vrep_sim.simx_opmode_blocking)

''' Wheels control function of youBot '''
def VREP_wheelsControl(vrep_sim, clientID, wheel_joints_handle, desired_wheel_velocities):
    for i in range(0,4):
        vrep_sim.simxSetJointTargetVelocity(clientID, wheel_joints_handle[i], desired_wheel_velocities[i], vrep_sim.simx_opmode_blocking)


if __name__ == '__main__':
    try:
        import sim as vrep_sim
    except:
        print ('--------------------------------------------------------------')
        print ('"sim.py" could not be imported. This means very probably that')
        print ('either "sim.py" or the remoteApi library could not be found.')
        print ('Make sure both are in the same folder as this file,')
        print ('or appropriately adjust the file "sim.py"')
        print ('--------------------------------------------------------------')
        print ('')

    ''' Initialization '''
    print ('Program started')
    vrep_sim.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep_sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID != -1:
        print ('Connected to remote API server')

        return_code, youBot_handle = vrep_sim.simxGetObjectHandle(clientID, 'youBot', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object youBot ok.')

        return_code, youBot_dummy_handle = vrep_sim.simxGetObjectHandle(clientID, 'youBotDummy', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object youBotDummy ok.')

        # Prepare initial values for four wheels
        wheel_joints_handle = [-1,-1,-1,-1]; # front left, rear left, rear right, front right
        return_code, wheel_joints_handle[0] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object youBot rollingJoint_fl ok.')

        return_code, wheel_joints_handle[1] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object youBot rollingJoint_rl ok.')

        return_code, wheel_joints_handle[2] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object youBot rollingJoint_rr ok.')

        return_code, wheel_joints_handle[3] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object youBot rollingJoint_fr ok.')
        
        # Prepare initial values for five arm joints
        arm_joints_handle = [-1,-1,-1,-1,-1]
        for i in range(0,4):
            return_code, arm_joints_handle[i] = vrep_sim.simxGetObjectHandle(clientID, 'youBotArmJoint' + str(i), vrep_sim.simx_opmode_blocking)
            if (return_code == vrep_sim.simx_return_ok):
                print('get object arm joint ' + str(i) + ' ok.')
        
        # Desired joint positions for initialization
        desired_arm_joint_angles = [180*math.pi/180, 30.91*math.pi/180, 52.42*math.pi/180, 72.68*math.pi/180, 0]
        
        # Initialization all arm joints
        for i in range(0,4):
            vrep_sim.simxSetJointPosition(clientID, arm_joints_handle[i], desired_arm_joint_angles[i], vrep_sim.simx_opmode_blocking)
    else:
        print ('Failed connecting to remote API server')


    # Size information for youBot
    wheel_R = 0.05
    a = 0.165
    b = 0.228

    # User variables
    simu_time = 0
    center_velocity = [0,0,0]
    desired_wheel_velocities = [0,0,0,0]

    ''' Main control loop '''
    print('begin main control loop ...')
    while True:
        # Motion planning
        simu_time = simu_time + 0.05
        
        for i in range(0,5):
            if int(simu_time) % 2 == 0:
                desired_arm_joint_angles[i] = desired_arm_joint_angles[i] - 0.04 # rad
            else:
                desired_arm_joint_angles[i] = desired_arm_joint_angles[i] + 0.04 # rad

        # Control the youBot robot
        VREP_armControl(vrep_sim, clientID, arm_joints_handle, desired_arm_joint_angles)
        VREP_wheelsControl(vrep_sim, clientID, wheel_joints_handle, desired_wheel_velocities)


    # Now send some data to CoppeliaSim in a non-blocking fashion:
    vrep_sim.simxAddStatusbarMessage(clientID,'Over!',vrep_sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep_sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    vrep_sim.simxFinish(clientID)
    print ('Program ended')
