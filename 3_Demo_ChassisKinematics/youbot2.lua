-- Initialization for YouBot
function sysCall_init()
    -- Get YouBot Handle
    you_bot_2 = sim.getObjectHandle('youBot_2')
    you_bot_dummy_2 = sim.getObjectHandle('youBotDummy_2')
    dummy_guider_handle_2 = sim.getObjectHandle('DummyGuider_2')
    
    -- Prepare initial values for four wheels
    wheel_joints_2 = {-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheel_joints_2[1] = sim.getObjectHandle('rollingJoint_fr_2')
    wheel_joints_2[2] = sim.getObjectHandle('rollingJoint_fl_2')
    wheel_joints_2[3] = sim.getObjectHandle('rollingJoint_rl_2')
    wheel_joints_2[4] = sim.getObjectHandle('rollingJoint_rr_2')
    
    -- Prepare initial values for five arm joints
    arm_joints_2 = {-1,-1,-1,-1,-1} -- set default values
    arm_joints_2[1] = sim.getObjectHandle('youBotArmJoint0_2')
    arm_joints_2[2] = sim.getObjectHandle('youBotArmJoint1_2')
    arm_joints_2[3] = sim.getObjectHandle('youBotArmJoint2_2')
    arm_joints_2[4] = sim.getObjectHandle('youBotArmJoint3_2')
    arm_joints_2[5] = sim.getObjectHandle('youBotArmJoint4_2')
    
    -- Desired joint positions for initialization
    desired_joint_angles_2 = {180*math.pi/180, 30.91*math.pi/180, 52.42*math.pi/180, 72.68*math.pi/180, 0}
    
    -- Initialization all arm joints
    for i = 1,5,1 do
        sim.setJointPosition(arm_joints_2[i], desired_joint_angles_2[i])
    end

    -- Set the maximum joint velocities
    max_joint_velocity = 100*math.pi/180
    
    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    omega_4 = 0

    -- youBot size parameters
    wheel_R = 0.05
    a = 0.165
    b = 0.228
    
    dummy_guider_position_2 = {0, 0, 0}
    dummy_guider_orientation_2 = {0, 0, 0}
    circle_path_R = 1
    target_pos_2 = {0, 0, 0}
    
    -- initialize youBot orientation
    youbot_init_orientation_2 = {-1.57, 0, -1.57}
    sim.setObjectOrientation(you_bot_2, -1, youbot_init_orientation_2)

    -- Initialization for two circle paths
    youbot_init_position_2 = {2, -2, 9.5341e-02}
    sim.setObjectPosition(you_bot_2, -1, youbot_init_position_2)
end


function sysCall_cleanup()
 
end

function sysCall_sensing()
    -- put your sensing code here
    dummy_guider_position_2 = sim.getObjectPosition(dummy_guider_handle_2, -1)
    --print('pos:', dummy_guider_position_2)
    dummy_guider_orientation_2 = sim.getObjectOrientation(dummy_guider_handle_2, -1)
    -- print('ori:', dummy_guider_orientation_2)
end

-- Inverse kinematics
function chassisInverseKinematics(vx, vy, omega, wheel_R, a, b)
    omega_1 = (vy - vx + (a+b)*omega)/wheel_R
    omega_2 = (vy + vx - (a+b)*omega)/wheel_R
    omega_3 = (vy - vx - (a+b)*omega)/wheel_R
    omega_4 = (vy + vx + (a+b)*omega)/wheel_R
    
    -- set the right direction for each wheel
    v_wheel_1 = -omega_1
    v_wheel_2 = -omega_2
    v_wheel_3 = -omega_3
    v_wheel_4 = -omega_4
end


-- Control joints of YouBot
function sysCall_actuation()
    -- Keep the arm status
    for i = 1,5,1 do
        sim.setJointPosition(arm_joints_2[i], desired_joint_angles_2[i])
    end

    simu_time = sim.getSimulationTime()

    -- dummy velocity
    dummy_guider_velocity_2 = 0.1


    -- Path 2: two circles
    vx = 0
    vy = dummy_guider_velocity_2
    target_pos_2  = sim.getObjectPosition(dummy_guider_handle_2, -1)
    if target_pos_2[1] > 0 then
        omega = dummy_guider_velocity_2 / circle_path_R 
    else
        omega = -dummy_guider_velocity_2 / circle_path_R 
    end


    center_velocity = {vx, vy, omega}
    chassisInverseKinematics(center_velocity[1], center_velocity[2], center_velocity[3], wheel_R, a, b)
    -- print('time=', simu_time, 'target_pos_2[1]=', target_pos_2[1], 'center_velocity=', center_velocity)
    
    -- Apply the desired wheel velocities
    sim.setJointTargetVelocity(wheel_joints_2[1], v_wheel_1)
    sim.setJointTargetVelocity(wheel_joints_2[2], v_wheel_2)
    sim.setJointTargetVelocity(wheel_joints_2[3], v_wheel_3)
    sim.setJointTargetVelocity(wheel_joints_2[4], v_wheel_4)
end
