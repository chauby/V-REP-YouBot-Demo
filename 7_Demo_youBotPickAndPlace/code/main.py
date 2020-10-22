import numpy as np
from TrajectoryGenerator import trajectoryGenerator
from NextState import nextState
from FeedbackControl import feedbackControl

import time
import sys
sys.path.append('./vrep/VREP_remoteAPIs')

#%% VREP control function definition
''' Arm control function of youBot '''
def VREP_armControl(vrep_sim, clientID, arm_joints_handle, desired_arm_joint_angles):
    for i in range(0,5):
        vrep_sim.simxSetJointPosition(clientID, arm_joints_handle[i], desired_arm_joint_angles[i], vrep_sim.simx_opmode_oneshot)

''' Wheels control function of youBot '''
def VREP_wheelsControl(vrep_sim, clientID, wheel_joints_handle, desired_wheel_positions):
    for i in range(0,4):
        vrep_sim.simxSetJointPosition(clientID, wheel_joints_handle[i], desired_wheel_positions[i], vrep_sim.simx_opmode_oneshot)

''' World frame control function of youBot '''
def VREP_WorldFrameControl(vrep_sim, clientID, world_joints_handle, desired_world_positions):
    for i in range(0,3):
        vrep_sim.simxSetJointPosition(clientID, world_joints_handle[i], desired_world_positions[i], vrep_sim.simx_opmode_oneshot)


#%% VREP initialization
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
    return_code, wheel_joints_handle[0] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object youBot rollingJoint_rl ok.')

    return_code, wheel_joints_handle[1] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object youBot rollingJoint_fl ok.')

    return_code, wheel_joints_handle[2] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object youBot rollingJoint_fr ok.')
    
    return_code, wheel_joints_handle[3] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object youBot rollingJoint_rr ok.')

    # Prepare initial values for five arm joints
    arm_joints_handle = [-1,-1,-1,-1,-1]
    for i in range(0,5):
        return_code, arm_joints_handle[i] = vrep_sim.simxGetObjectHandle(clientID, 'Joint' + str(i+1), vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object arm joint ' + str(i+1) + ' ok.')
    
    return_code, gripper_joint_1_handle = vrep_sim.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object gripper joint 1 ok')

    return_code, gripper_joint_2_handle = vrep_sim.simxGetObjectHandle(clientID, 'youBotGripperJoint2', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object gripper joint 2 ok')

    return_code, gripper_tip_handle = vrep_sim.simxGetObjectHandle(clientID, 'youBot_positionTip', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object gripper position tip ok')

    # Desired joint positions for initialization
    desired_arm_joint_angles = [0, 0, 0, 0, 0]
    
    # Initialization all arm joints
    for i in range(0,5):
        vrep_sim.simxSetJointPosition(clientID, arm_joints_handle[i], desired_arm_joint_angles[i], vrep_sim.simx_opmode_blocking)

    # Get world joints handles
    world_joints_handle = [-1, -1, -1]
    return_code, world_joints_handle[0] = vrep_sim.simxGetObjectHandle(clientID, 'World_X_Joint', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object world joint X ok.')

    return_code, world_joints_handle[1] = vrep_sim.simxGetObjectHandle(clientID, 'World_Y_Joint', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object world joint Y ok.')

    return_code, world_joints_handle[2] = vrep_sim.simxGetObjectHandle(clientID, 'World_Th_Joint', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object world joint Th ok.')

    # Get initial and goal cube handles
    return_code, cube_1_handle = vrep_sim.simxGetObjectHandle(clientID, 'Cube_1', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object cube_1 ok.')

    return_code, cube_2_handle = vrep_sim.simxGetObjectHandle(clientID, 'Cube_2', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object cube_2 ok.')

    return_code, cube_goal_handle = vrep_sim.simxGetObjectHandle(clientID, 'Cube_goal', vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get object cube_goal_1 ok.')
else:
    print ('Failed connecting to remote API server')


#%% ---------------------- generate trajectory and control the robot ---------------------------
# Get initial and goal cube positions
_, cube_1_position = vrep_sim.simxGetObjectPosition(clientID, cube_1_handle, -1, vrep_sim.simx_opmode_blocking)
_, cube_2_position = vrep_sim.simxGetObjectPosition(clientID, cube_2_handle, -1, vrep_sim.simx_opmode_blocking)
_, cube_goal_position = vrep_sim.simxGetObjectPosition(clientID, cube_goal_handle, -1, vrep_sim.simx_opmode_blocking)


for i in range (0,2):
    # Get youBot position
    _, youBot_position = vrep_sim.simxGetObjectPosition(clientID, youBot_handle, -1, vrep_sim.simx_opmode_blocking)
    _, youBot_x = vrep_sim.simxGetJointPosition(clientID, world_joints_handle[0], vrep_sim.simx_opmode_blocking)
    _, youBot_y = vrep_sim.simxGetJointPosition(clientID, world_joints_handle[1], vrep_sim.simx_opmode_blocking)
    _, youBot_phi = vrep_sim.simxGetJointPosition(clientID, world_joints_handle[2], vrep_sim.simx_opmode_blocking)

    # Get youBot arm joint angles
    youBot_arm_q = [0, 0, 0, 0, 0]
    for j in range(0, 5):
        _, youBot_arm_q[j] = vrep_sim.simxGetJointPosition(clientID, arm_joints_handle[j], vrep_sim.simx_opmode_blocking)

    # Get youBot wheel angles
    youBot_wheel_angles = [0, 0, 0, 0]
    for j in range(0, 4):
        _, youBot_wheel_angles[j] = vrep_sim.simxGetJointPosition(clientID, wheel_joints_handle[j], vrep_sim.simx_opmode_blocking)

    # Get youBot gripper position
    _, youBot_gripper_position = vrep_sim.simxGetObjectPosition(clientID, gripper_tip_handle, -1, vrep_sim.simx_opmode_blocking)

    # Initialize the robot's configuration
    #                         phi x  y  J1 J2 J3 J4 J5 w1 w2 w3 w4 gripper
    config_initial = np.array([youBot_phi, youBot_x, youBot_y, youBot_arm_q[0], youBot_arm_q[1], youBot_arm_q[2], youBot_arm_q[3], youBot_arm_q[4], youBot_wheel_angles[0], youBot_wheel_angles[1], youBot_wheel_angles[2], youBot_wheel_angles[3], 0]).reshape(1,13)

    # Generate trajectory
    T_se_initial = np.array([[0,0,1,youBot_gripper_position[0]],[0,1,0,youBot_gripper_position[1]],[-1,0,0,youBot_gripper_position[2]],[0,0,0,1]])
    if i == 0:
        T_sc_initial = np.array([[1,0,0,cube_1_position[0]],[0,1,0,cube_1_position[1]],[0,0,1,cube_1_position[2]],[0,0,0,1]])
        T_sc_final = np.array([[0,1,0,cube_goal_position[0]],[-1,0,0,cube_goal_position[1]],[0,0,1, cube_goal_position[2]],[0,0,0,1]])
    else:
        T_sc_initial = np.array([[1,0,0,cube_2_position[0]],[0,1,0,cube_2_position[1]],[0,0,1,cube_2_position[2]],[0,0,0,1]])
        T_sc_final = np.array([[0,1,0,cube_goal_position[0]],[-1,0,0,cube_goal_position[1]],[0,0,1, cube_goal_position[2] + 0.05],[0,0,0,1]])
    gripper_pose = np.pi/3 # change orientation of the gripper, default value is pi/4
    T_ce_grasp = np.array([[-np.sin(gripper_pose), 0, np.cos(gripper_pose),0],[0,1,0,0],[-np.cos(gripper_pose), 0, -np.sin(gripper_pose),0],[0,0,0,1]])
    T_ce_standoff = np.array([[-np.sin(gripper_pose), 0, np.cos(gripper_pose),0],[0,1,0,0],[-np.cos(gripper_pose), 0, -np.sin(gripper_pose),0.1],[0,0,0,1]])

    k = 1
    T_se_post = trajectoryGenerator(T_se_initial,T_sc_initial,T_sc_final,T_ce_grasp,T_ce_standoff,k)

    # set to use joint limits
    joint_limit = 0

    # set Kp, Ki and dt
    Kp = 2.2
    Ki = 0
    dt = 0.01

    #%% ------------------------------------- main loop --------------------------------------
    desired_wheel_positions = [0,0,0,0]
    desired_world_positions = [0,0,0]
    reference_trajectory = []
    reference_trajectory.append(config_initial)
    new_config = config_initial

    print('begin main control loop ...')
    for i in np.arange(np.shape(T_se_post)[0]-1):
        # write down Xd
        Xd = np.array([[T_se_post[i,0],T_se_post[i,1],T_se_post[i,2],T_se_post[i,9]],[T_se_post[i,3],T_se_post[i,4],T_se_post[i,5],T_se_post[i,10]],[T_se_post[i,6],T_se_post[i,7],T_se_post[i,8],T_se_post[i,11]],[0,0,0,1]])

        # write down Xd_next
        Xd_next = np.array([[T_se_post[i+1,0],T_se_post[i+1,1],T_se_post[i+1,2],T_se_post[i+1,9]],[T_se_post[i+1,3],T_se_post[i+1,4],T_se_post[i+1,5],T_se_post[i+1,10]],[T_se_post[i+1,6],T_se_post[i+1,7],T_se_post[i+1,8],T_se_post[i+1,11]],[0,0,0,1]])

        # compute speeds and X_err
        V, speeds, X_err = feedbackControl(new_config, Xd, Xd_next, Kp, Ki, dt, joint_limit)

        # adjust the order of wheels speeds and joints speeds
        speeds = np.array([speeds[4], speeds[5], speeds[6], speeds[7], speeds[8], speeds[0], speeds[1], speeds[2], speeds[3]]).reshape(1,9)
        speeds_max = 1000

        # compute new configuration
        new_config = nextState(new_config,speeds,dt,speeds_max)
        new_config = np.append(new_config,[[T_se_post[i,12]]],axis=1)
        reference_trajectory.append(new_config) # record the trajectory

        # set commands for the youBot model in VREP
        desired_world_positions[0] = new_config[0, 1]
        desired_world_positions[1] = new_config[0, 2]
        desired_world_positions[2] = new_config[0, 0]
        
        desired_arm_joint_angles[0] = new_config[0, 3]
        desired_arm_joint_angles[1] = new_config[0, 4]
        desired_arm_joint_angles[2] = new_config[0, 5]
        desired_arm_joint_angles[3] = new_config[0, 6]
        desired_arm_joint_angles[4] = new_config[0, 7]

        desired_wheel_positions[0] = new_config[0, 8]
        desired_wheel_positions[1] = new_config[0, 9]
        desired_wheel_positions[2] = new_config[0, 10]
        desired_wheel_positions[3] = new_config[0, 11]

        # send commands to VREP
        VREP_armControl(vrep_sim, clientID, arm_joints_handle, desired_arm_joint_angles)
        VREP_wheelsControl(vrep_sim, clientID, wheel_joints_handle, desired_wheel_positions)
        VREP_WorldFrameControl(vrep_sim, clientID, world_joints_handle, desired_world_positions)

        # set gripper state
        if new_config[0, 12] == 0:
            vrep_sim.simxSetJointTargetVelocity(clientID, gripper_joint_2_handle, -0.04, vrep_sim.simx_opmode_oneshot) # open
        else:
            vrep_sim.simxSetJointTargetVelocity(clientID, gripper_joint_2_handle, 0.04, vrep_sim.simx_opmode_oneshot) # close
        return_code, gripper_joint_2_position = vrep_sim.simxGetJointPosition(clientID, gripper_joint_2_handle, vrep_sim.simx_opmode_oneshot)
        vrep_sim.simxSetJointTargetPosition(clientID, gripper_joint_1_handle, -gripper_joint_2_position, vrep_sim.simx_opmode_oneshot)
        time.sleep(0.002)

    # Squeeze reference trajectory
    reference_trajectory = np.squeeze(reference_trajectory)

# Stop VREP simulation
#print ('Stop VREP simulation')
#vrep_sim.simxStopSimulation(clientID, vrep_sim.simx_opmode_blocking)
time.sleep(0.1)

# Now close the connection to VREP
vrep_sim.simxFinish(clientID)
print ('Program ended')
