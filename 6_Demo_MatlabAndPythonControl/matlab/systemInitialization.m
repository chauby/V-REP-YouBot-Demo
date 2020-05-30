%% System Initialization
disp('System Initialization');

vrep_sim=remApi('remoteApi');
vrep_sim.simxFinish(-1); % just in case, close all opened connections
clientID=vrep_sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');

    [return_code, youBot_handle] = vrep_sim.simxGetObjectHandle(clientID, 'youBot', vrep_sim.simx_opmode_blocking);
    if (return_code == vrep_sim.simx_return_ok)
        disp('get object youBot ok.');
    end

    [return_code, youBot_dummy_handle] = vrep_sim.simxGetObjectHandle(clientID, 'youBotDummy', vrep_sim.simx_opmode_blocking);
    if (return_code == vrep_sim.simx_return_ok)
        disp('get object youBotDummy ok.');
    end

    % Prepare initial values for four wheels
    wheel_joints_handle = [-1,-1,-1,-1]; % front left, rear left, rear right, front right
    [return_code, wheel_joints_handle(1)] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep_sim.simx_opmode_blocking);
    if (return_code == vrep_sim.simx_return_ok)
        disp('get object youBot rollingJoint_fl ok.');
    end

    [return_code, wheel_joints_handle(2)] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep_sim.simx_opmode_blocking);
    if (return_code == vrep_sim.simx_return_ok)
        disp('get object youBot rollingJoint_rl ok.');
    end

    [return_code, wheel_joints_handle(3)] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep_sim.simx_opmode_blocking);
    if (return_code == vrep_sim.simx_return_ok)
        disp('get object youBot rollingJoint_rr ok.');
    end

    [return_code, wheel_joints_handle(4)] = vrep_sim.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep_sim.simx_opmode_blocking);
    if (return_code == vrep_sim.simx_return_ok)
        disp('get object youBot rollingJoint_fr ok.');
    end
    
    % Prepare initial values for five arm joints
    arm_joints_handle = [-1,-1,-1,-1,-1];
    for i=0:4
        [return_code, arm_joints_handle(i+1)] = vrep_sim.simxGetObjectHandle(clientID, strcat('youBotArmJoint', num2str(i)), vrep_sim.simx_opmode_blocking);
        if (return_code == vrep_sim.simx_return_ok)
            disp(strcat('get object arm joint ', num2str(i), ' ok.'));
        end
    end
    
    % Desired joint positions for initialization
    desired_arm_joint_angles = [180*pi/180, 30.91*pi/180, 52.42*pi/180, 72.68*pi/180, 0];
    
    % Initialization all arm joints
    for i = 1:5
        vrep_sim.simxSetJointPosition(clientID, arm_joints_handle(i), desired_arm_joint_angles(i), vrep_sim.simx_opmode_blocking);
    end
else
    disp('Failed connecting to remote API server');
    pause();
end
