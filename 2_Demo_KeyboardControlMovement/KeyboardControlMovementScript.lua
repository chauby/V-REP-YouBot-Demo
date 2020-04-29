-- Initialization for YouBot
function sysCall_init()
    -- Get YouBot Handle
    you_bot = sim.getObjectHandle('youBot')
    
    -- Prepare initial values for four wheels
    wheel_joints = {-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheel_joints[1] = sim.getObjectHandle('rollingJoint_fl')
    wheel_joints[2] = sim.getObjectHandle('rollingJoint_rl')
    wheel_joints[3] = sim.getObjectHandle('rollingJoint_rr')
    wheel_joints[4] = sim.getObjectHandle('rollingJoint_fr')
    
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
    max_joint_velocity = 50*math.pi/180
    
    -- user customized variables, default as global variables
    move_direction = 1 -- 1:forward, 2:backward, 3:left, 4:right
    wheel_velocity = 0 -- Initialized velocity with zero value
end


function sysCall_cleanup()
 
end

function getKeyboardStatus()
    message,data,data2=simGetSimulatorMessage()
    if (message==sim_message_keypress) then
        if (data[1]==2007) then -- up key
            print("up")
            move_direction = 1
        elseif (data[1]==2008) then -- down key
            print("down")
            move_direction = 2
        elseif (data[1]==2009) then -- left key
            print("left")
            move_direction = 3
        elseif (data[1]==2010) then -- right key
            print("right")
            move_direction = 4
        elseif (data[1]==string.byte('q')) then -- q key
            print("q")
            move_direction = 5
        elseif (data[1]==string.byte('w')) then -- w key
            print("w")
            move_direction = 6
        end

        if (data[1]==string.byte(' ')) then -- space key
            wheel_velocity = 0
            print("stop")
        else
            wheel_velocity = 1
        end
    end
end

function setYouBotMovementDirection()
    -- Check movement direction
    if (move_direction == 1) then -- go forward
        lf_dire = -1
        rf_dire = -1
        lr_dire = -1
        rr_dire = -1
    elseif (move_direction == 2) then -- go backward
        lf_dire = 1
        rf_dire = 1
        lr_dire = 1
        rr_dire = 1
    elseif (move_direction == 3) then -- go left
        lf_dire = 1
        rf_dire = -1
        lr_dire = -1
        rr_dire = 1
    elseif (move_direction == 4) then -- go right
        lf_dire = -1
        rf_dire = 1
        lr_dire = 1
        rr_dire = -1
    elseif (move_direction == 5) then -- turn left
        lf_dire = 1
        rf_dire = -1
        lr_dire = 1
        rr_dire = -1
    elseif (move_direction == 6) then -- turn right
        lf_dire = -1
        rf_dire = 1
        lr_dire = -1
        rr_dire = 1
    end
end

-- Control joints of YouBot
function sysCall_actuation()
    -- Check keyboard press events
    getKeyboardStatus()
    setYouBotMovementDirection()

    -- Keep the arm status
    for i = 1,5,1 do
        sim.setJointPosition(arm_joints[i], desired_joint_angles[i])
    end
    
    -- Apply the desired wheel velocities
    sim.setJointTargetVelocity(wheel_joints[1], lf_dire*wheel_velocity)
    sim.setJointTargetVelocity(wheel_joints[2], lr_dire*wheel_velocity)
    sim.setJointTargetVelocity(wheel_joints[3], rr_dire*wheel_velocity)
    sim.setJointTargetVelocity(wheel_joints[4], rf_dire*wheel_velocity)
end 
