function VREP_wheelsControl(vrep_sim, clientID, wheel_joints_handle, desired_wheel_velocities)
    for i = 1:4
	    vrep_sim.simxSetJointTargetVelocity(clientID, wheel_joints_handle(i), desired_wheel_velocities(i), vrep_sim.simx_opmode_blocking);
	end
end