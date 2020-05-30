clear

%% Initialization
systemInitialization;

% Size information for youBot
wheel_R = 0.05;
a = 0.165;
b = 0.228;

% User variables
simu_time = 0;
center_velocity = [0,0,0];
desired_wheel_velocities = [0,0,0,0];

%% Main control loop
disp('begin main control loop ...');
while true
    % Motion planning
    simu_time = simu_time + 0.05;
    
    if simu_time < 1
        center_velocity = [0, 0.1, 0];
    elseif simu_time < 2
        center_velocity = [0, -0.1, 0];
    elseif simu_time < 3
        center_velocity = [0.1, 0, 0];
    elseif simu_time < 4
        center_velocity = [-0.1, 0, 0];
    elseif simu_time < 5
        center_velocity = [0.1, 0.1, 0];
    elseif simu_time < 6
        center_velocity = [-0.1, -0.1, 0];
    elseif simu_time < 7
        center_velocity = [0, 0, pi/10];
    elseif simu_time < 8
        center_velocity = [0, 0, -pi/10];
    elseif simu_time < 9
        center_velocity = [0, 0, -pi/10];
    elseif simu_time < 10
        center_velocity = [0, 0, pi/10];
    elseif simu_time < 11
        center_velocity = [0, 0, 0];
    else
        break;
    end

    % Inverse kinematics calculation
    desired_wheel_velocities = chassisInverseKinematics(center_velocity(1), center_velocity(2), center_velocity(3), wheel_R, a, b);

    % Control the youBot robot
    % VREP_armControl(vrep_sim, clientID, arm_joints_handle, desired_arm_joint_angles);
    VREP_wheelsControl(vrep_sim, clientID, wheel_joints_handle, desired_wheel_velocities);
    pause(0.001);
end

% Now send some data to CoppeliaSim in a non-blocking fashion:
vrep_sim.simxAddStatusbarMessage(clientID,'Over!',vrep_sim.simx_opmode_oneshot);

% Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep_sim.simxGetPingTime(clientID);

% Now close the connection to CoppeliaSim:
vrep_sim.simxFinish(clientID);

% call the destructor
vrep_sim.delete();
disp('Program ended');
