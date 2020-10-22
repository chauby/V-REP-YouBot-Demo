import numpy as np
import modern_robotics as mr

def trajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k):
    # ---------------------------------
    # segment 1: A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block
    # represent the frame of end-effector when standoff in space frame
    T_se_standoff_initial = T_sc_initial.dot(T_ce_standoff)
    # generate trajectory when approaching the standoff position in segment 1
    T_se_segment_1 = mr.CartesianTrajectory(T_se_initial,T_se_standoff_initial,5,5/0.01,3)
    
    # ---------------------------------
    # segment 2: A trajectory to move the gripper down to the grasp position
    # represent the frame of end-effecto    r in space frame in segment 2
    T_se_grasp = T_sc_initial.dot(T_ce_grasp)
    # generate trajectory when approaching the grasping position in segment 2
    T_se_seg2 = mr.CartesianTrajectory(T_se_segment_1[-1], T_se_grasp, 2, 2/0.01, 3)
    # append the trajectory of segment 2 after segment 1
    T_se_before = np.append(T_se_segment_1, T_se_seg2, axis=0)

    # ---------------------------------
    # segment 3: Closing of the gripper
    # append the trajectory of segment 3 by 63 times
    for i in np.arange(64):
        T_se_before = np.append(T_se_before,np.array([T_se_before[-1]]),axis=0)
    
    # ---------------------------------
    # segment 4: A trajectory to move the gripper back up to the "standoff" configuration
    # generate trajectory when back on the standoff position in segment 4
    T_se_segment_4 = mr.CartesianTrajectory(T_se_grasp, T_se_standoff_initial, 2, 2/0.01, 3)
    # append the trajectory of segment 4
    T_se_before = np.append(T_se_before,T_se_segment_4,axis=0)

    # ---------------------------------
    # segment 5: A trajectory to move the gripper to a "standoff" configuration above the final configuration
    # generate trajectory when moving to the final standoff position in segment 5
    T_se_standoff_final = T_sc_final.dot(T_ce_standoff)
    T_se_segment_5 = mr.CartesianTrajectory(T_se_standoff_initial, T_se_standoff_final, 8, 8/0.01, 3)
    # append the trajectory of segment 5
    T_se_before = np.append(T_se_before, T_se_segment_5, axis=0)

    # ---------------------------------
    # segment 6: A trajectory to move the gripper to the final configuration of the object
    # generate the end-effector configuration when losing
    T_se_lose = T_sc_final.dot(T_ce_grasp)
    # generate trajectory when moving to the final cube position in segment 6
    T_se_segment_6 = mr.CartesianTrajectory(T_se_standoff_final, T_se_lose, 2, 2/0.01, 3)
    # append the trajectory of segment 6
    T_se_before = np.append(T_se_before, T_se_segment_6, axis=0)

    # ---------------------------------
    # segment 7: Opening of the gripper
    # append the trajectory of segment 7 by 63 times
    for i in np.arange(64):
        T_se_before = np.append(T_se_before, np.array([T_se_before[-1]]), axis=0)
    
    # ---------------------------------
    # segment 8: A trajectory to move the gripper back to the "standoff" configuration
    # generate trajectory when moving to the final standoff position in segment 8
    T_se_segment_8 = mr.CartesianTrajectory(T_se_before[-1], T_se_standoff_final, 2, 2/0.01, 3)
    # append the trajectory of segment 8
    T_se_before = np.append(T_se_before, T_se_segment_8, axis=0)

    # ---------------------------------
    # generate a matrix which is n by 13
    T_se_post = np.zeros([int(k*21/0.01+64*2),13])
    # put the configuration, position and gripper state in matrix which is n by 13
    for i in np.arange(int(k*21/0.01+64*2)):
        T_se_post[i,0] = T_se_before[i,0,0]
        T_se_post[i,1] = T_se_before[i,0,1]
        T_se_post[i,2] = T_se_before[i,0,2]
        T_se_post[i,3] = T_se_before[i,1,0]
        T_se_post[i,4] = T_se_before[i,1,1]
        T_se_post[i,5] = T_se_before[i,1,2]
        T_se_post[i,6] = T_se_before[i,2,0]
        T_se_post[i,7] = T_se_before[i,2,1]
        T_se_post[i,8] = T_se_before[i,2,2]
        T_se_post[i,9] = T_se_before[i,0,3]
        T_se_post[i,10] = T_se_before[i,1,3]
        T_se_post[i,11] = T_se_before[i,2,3]
        T_se_post[i,12] = 0
    # amend the gripper state in segment 3, 4, 5, 6
    for i in np.arange(int(k*7/0.01), int(k*19/0.01+64)):
        T_se_post[i, 12] = 1

    return T_se_post