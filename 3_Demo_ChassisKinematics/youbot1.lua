-- Initialization for YouBot
function sysCall_init()
    -- Get YouBot Handle
    you_bot = sim.getObjectHandle('youBot')
    you_bot_dummy = sim.getObjectHandle('youBotDummy')
    dummy_guider_handle = sim.getObjectHandle('DummyGuider')
    
    -- Prepare initial values for four wheels
    wheel_joints = {-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheel_joints[1] = sim.getObjectHandle('rollingJoint_fr')
    wheel_joints[2] = sim.getObjectHandle('rollingJoint_fl')
    wheel_joints[3] = sim.getObjectHandle('rollingJoint_rl')
    wheel_joints[4] = sim.getObjectHandle('rollingJoint_rr')
    
    -- Prepare initial values for five arm joints
    arm_joints = {-1,-1,-1,-1,-1} -- set default values
    for i=0,4,1 do
        arm_joints[i+1] = sim.getObjectHandle('youBotArmJoint'..i)
    end
    
    -- Desired joint positions for initialization
    desired_joint_angles = {180*math.pi/180, 30.91*math.pi/180, 52.42*math.pi/180, 72.68*math.pi/180, 0}
    
    -- Initialization all arm joints
    for i = 1,5,1 do
        sim.setJointPosition(arm_joints[i], desired_joint_angles[i])
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
    
    dummy_guider_position = {0, 0, 0}
    dummy_guider_orientation = {0, 0, 0}
    circle_path_R = 1
    target_pos = {0, 0, 0}
    
    -- initialize youBot orientation
    youbot_init_orientation = {-1.57, 0, -1.57}
    sim.setObjectOrientation(you_bot, -1, youbot_init_orientation)

    -- Initialization for  one circle path
    youbot_init_position = {1, 0, 9.5341e-02}
end


function sysCall_cleanup()
 
end

function sysCall_sensing()
    -- put your sensing code here
    dummy_guider_position = sim.getObjectPosition(dummy_guider_handle, -1)
    --print('pos:', dummy_guider_position)
    dummy_guider_orientation = sim.getObjectOrientation(dummy_guider_handle, -1)
    -- print('ori:', dummy_guider_orientation)
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
        sim.setJointPosition(arm_joints[i], desired_joint_angles[i])
    end

    simu_time = sim.getSimulationTime()

    -- dummy velocity
    dummy_guider_velocity = 0.1

    -- Path 1: one circle
    vx = 0
    vy = dummy_guider_velocity
    omega = dummy_guider_velocity / circle_path_R 

    center_velocity = {vx, vy, omega}
    chassisInverseKinematics(center_velocity[1], center_velocity[2], center_velocity[3], wheel_R, a, b)
    -- print('time=', simu_time, 'target_pos[1]=', target_pos[1], 'center_velocity=', center_velocity)
    
    -- Apply the desired wheel velocities
    sim.setJointTargetVelocity(wheel_joints[1], v_wheel_1)
    sim.setJointTargetVelocity(wheel_joints[2], v_wheel_2)
    sim.setJointTargetVelocity(wheel_joints[3], v_wheel_3)
    sim.setJointTargetVelocity(wheel_joints[4], v_wheel_4)
end
