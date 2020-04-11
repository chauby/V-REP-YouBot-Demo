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
    desired_joint_angles = {0,0,0,0,0}
    
    -- Initialization all arm joints
    for i = 1,5,1 do
        sim.setJointPosition(arm_joints[i], desired_joint_angles[i])
    end

    -- Set the maximum joint velocities
    max_joint_velocity = 50*math.pi/180 
    
    -- user customized variables, default as global variables
    switch_pos_flag = 0
    go_forward_flag = 1 -- set YouBot to move forward
end


function sysCall_cleanup()
 
end 

-- Control joints of YouBot
function sysCall_actuation() 
    current_joint_angles = {0,0,0,0,0}
    for i=1,5,1 do
        current_joint_angles[i] = sim.getJointPosition(arm_joints[i])
    end
    max_variation_allowed = max_joint_velocity*sim.getSimulationTimeStep()
    
    simu_time = sim.getSimulationTime()
    
    
    -- set the desired values for every 10 seconds simulation time, ps: not real time
    if (simu_time % 10 == 0) then
        print('\nSimulation time = '..simu_time)
        
        -- Change the move direction for every 20 seonds simulation time
        if (simu_time % 20 == 0) then
            if go_forward_flag == 1 then
                go_forward_flag = -1 -- go backward
            else
                go_forward_flag = 1 -- go forward
            end
        end
        
        -- Set the desired joint angles of YouBot Arm
        if switch_pos_flag == 0 then
            print("Switch to position 1")
            desired_joint_angles = {180,0,0,0,0}
            switch_pos_flag = 1
        elseif switch_pos_flag == 1 then
            print("Switch to position 2")
            desired_joint_angles = {120*math.pi/180, 30.91*math.pi/180, 52.42*math.pi/180, 72.68*math.pi/180, 0}
            switch_pos_flag = 2
        else
            print("Switch to position 3")
            desired_joint_angles = {-120*math.pi/180, 30.91*math.pi/180, 52.42*math.pi/180, 72.68*math.pi/180, 0}
            switch_pos_flag = 0
        end   
    end
  
    -- Apply the desired wheel velocities
    wheel_velocity = go_forward_flag*0.8;
    sim.setJointTargetVelocity(wheel_joints[1], wheel_velocity)
    sim.setJointTargetVelocity(wheel_joints[2], wheel_velocity)
    sim.setJointTargetVelocity(wheel_joints[3], wheel_velocity)
    sim.setJointTargetVelocity(wheel_joints[4], wheel_velocity)
      
    -- Apply the desired joint angles for each arm joint
    for i=1,5,1 do
        delta = desired_joint_angles[i] - current_joint_angles[i]
        if (math.abs(delta) > max_variation_allowed) then
            delta = max_variation_allowed*delta/math.abs(delta) -- limit the variation of each joint
        end
        
        sim.setJointPosition(arm_joints[i], current_joint_angles[i] + delta)
    end
end 
