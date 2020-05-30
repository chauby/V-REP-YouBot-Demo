function VREP_armControl(vrep_sim, clientID, arm_joints_handle, desired_arm_joint_angles)
    for i = 1:5
        vrep_sim.simxSetJointPosition(clientID, arm_joints_handle(i), desired_arm_joint_angles(i), vrep_sim.simx_opmode_blocking);
    end
end