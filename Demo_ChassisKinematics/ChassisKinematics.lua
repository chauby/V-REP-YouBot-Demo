-- Initialization for YouBot
function sysCall_init()
    -- Get YouBot Handle
    you_bot = sim.getObjectHandle('youBot')
    
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
    
    -- user customized variables, default as global variables
    move_direction = 1 -- 1:forward, 2:backward, 3:left, 4:right
    wheel_velocity = 0 -- Initialized velocity with zero value
    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    omega_4 = 0

    wheel_R = 0.05 -- 0.05 m
    a = 0.228 -- 0.228 m
    b = 0.25 -- 0.25 m
    
    current_pos = {0, 0, 0}
    target_pos = {0, 0, 0}
    theta = 0
    delta_theta = 0.01
    circle_R = 1
    circle_pos = {circle_R, 0}
    
    youbot_init_position = {0, 0, 9.5341e-02}
    sim.setObjectPosition(you_bot, -1, youbot_init_position)

    InitializeTwoCirclesPathPlanning()
end


function sysCall_cleanup()
 
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

-- Single circle path
function circlePathPlanning()
    theta = theta + delta_theta
    target_pos[1] = circle_pos[1] - circle_R*math.cos(theta)
    target_pos[2] = circle_pos[2] + circle_R*math.sin(theta)
    target_pos[3] = 0
end

-- Two circles path
function InitializeTwoCirclesPathPlanning()
    t = 0
    t1 = 1
    t2 = 3
    t3 = 4
    time_step = 0.002

    circle_R1 = 1
    circle_R2 = 1

    theta_1 = 0
    theta_2 = 0
    pos_x = 0
    pos_y = 0
end

function twoCirclesPathPlanning()
    if t <= t1 then
        theta_1 = theta_1 + math.pi*time_step/t1
    elseif t <= t2 then
        theta_2 = theta_2 + 2*math.pi - 2*math.pi*time_step/(t2-t1)
    else
        theta_1 = theta_1 + math.pi*time_step/(t3-t2)
    end
    
    pos_x = circle_R1 - circle_R1*math.cos(theta_1) + circle_R2 - circle_R2*math.cos(theta_2)
    pos_y = circle_R1*math.sin(theta_1) + circle_R2*math.sin(theta_2)
    
    target_pos[1] = pos_x
    target_pos[2] = pos_y
    target_pos[3] = 0
end


function circlePathPlanning()
    theta = theta + delta_theta
    target_pos[1] = circle_pos[1] - circle_R*math.cos(theta)
    target_pos[2] = circle_pos[2] + circle_R*math.sin(theta)
    target_pos[3] = 0
end

-- Control joints of YouBot
function sysCall_actuation()
    -- Keep the arm status
    for i = 1,5,1 do
        sim.setJointPosition(arm_joints[i], desired_joint_angles[i])
    end
    
    -- Demo control
    -- vx = 0.1
    -- vy = 0
    -- omega = 0
    -- chassisInverseKinematics(vx, vy, omega, wheel_R, a, b)

    -- Plan the path for YouBot
    current_pos = sim.getObjectPosition(you_bot,-1)
    current_orientation = sim.getObjectOrientation(you_bot, -1, opMode)
    -- print('current_ori=', current_orientation)
    current_pos[3] = current_orientation[3]
    
    -- target_pos = {0, 0, 0.1}
    -- circlePathPlanning() -- update target position
    -- print('target_pos=', target_pos)
    -- print('current_pos=', current_pos) -- error

    if t == 0 then
        print('initialize path')
        InitializeTwoCirclesPathPlanning()
        t = t + time_step
    elseif t > t3 then
        print('end')
        t = 0
    else
        print('following path')
        twoCirclesPathPlanning()
        t = t + time_step
    end
    
    -- Simple PID control
    KP_pos = 0.5
    KP_omega = 0
    center_velocity = {KP_pos*(target_pos[1]  - current_pos[1]), KP_pos*(target_pos[2] - current_pos[2]), KP_omega*(target_pos[3] - current_pos[3])}
    chassisInverseKinematics(center_velocity[1], center_velocity[2], center_velocity[3], wheel_R, a, b)
    
    
    -- Apply the desired wheel velocities
    sim.setJointTargetVelocity(wheel_joints[1], v_wheel_1)
    sim.setJointTargetVelocity(wheel_joints[2], v_wheel_2)
    sim.setJointTargetVelocity(wheel_joints[3], v_wheel_3)
    sim.setJointTargetVelocity(wheel_joints[4], v_wheel_4)
end
