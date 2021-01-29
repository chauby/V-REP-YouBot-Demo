-- Initialization for YouBot
function sysCall_init()
    -- Get YouBot Handle
    youBot_handle = sim.getObjectHandle('youBot')
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
    wheel_R = 0.0475
    a = 0.15
    b = 0.235
    
    -- initialize youBot orientation
    youbot_init_orientation = {-math.pi/2, 0, -math.pi/2}
    sim.setObjectOrientation(youBot_handle, -1, youbot_init_orientation)
end


function sysCall_cleanup()
 
end

function sysCall_sensing()

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
        
    -- get the relative positions
    delta_pos = sim.getObjectPosition(dummy_guider_handle, youBot_handle)
    delta_ori = sim.getObjectOrientation(dummy_guider_handle, youBot_handle)
    
    Kp = 1.0
    vx = Kp*delta_pos[2] / 0.05
    vy = Kp*delta_pos[3] / 0.05
    omega = Kp*delta_ori[1] / 0.05

    center_velocity = {vx, vy, omega}
    chassisInverseKinematics(center_velocity[1], center_velocity[2], center_velocity[3], wheel_R, a, b)
    
    -- Apply the desired wheel velocities
    sim.setJointTargetVelocity(wheel_joints[1], v_wheel_1)
    sim.setJointTargetVelocity(wheel_joints[2], v_wheel_2)
    sim.setJointTargetVelocity(wheel_joints[3], v_wheel_3)
    sim.setJointTargetVelocity(wheel_joints[4], v_wheel_4)
end
