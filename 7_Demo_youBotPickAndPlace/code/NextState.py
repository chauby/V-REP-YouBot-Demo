import numpy as np

def nextState(config,speed,delta_t,speed_max):
    # here we limit speed
    for j in np.arange(np.shape(speed)[1]):
        if speed[0,j]>speed_max:
            speed[0,j] = speed_max
        elif speed[0,j]<-speed_max:
            speed[0,j] = -speed_max

    # here we get the new arm and wheels configurations
    new_angle_config = config[0,3:12]+speed*delta_t

    # here we start to compute the configuration of chassis using odometry
    # step1: compute delta theta, which is the change of wheels rotation
    delta_theta = (speed[0,5:].reshape(1,4)).T*delta_t
    
    # step2: compute the wheels velocities
    theta_dot = delta_theta

    # step3: compute chassis planar twist Vb
    # here we write down some necessary parameters for step3
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    Vb = (r/4)*np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]]).dot(theta_dot)

    # step4: calculate new chassis config
    # here we write down dqb
    if Vb[0,0] == 0:
        dqb = np.array([[0],[Vb[1,0]],[Vb[2,0]]])
    elif Vb[0,0] != 0:
        dqb = np.array([[Vb[0,0]],[(Vb[1,0]*np.sin(Vb[0,0])+Vb[2,0]*(np.cos(Vb[0,0])-1))/Vb[0,0]],[(Vb[2,0]*np.sin(Vb[0,0])+Vb[1,0]*(1-np.cos(Vb[0,0])))/Vb[0,0]]])
    # here we write down dq
    dq = np.array([[1,0,0],[0,np.cos(config[0,0]),-np.sin(config[0,0])],[0,np.sin(config[0,0]),np.cos(config[0,0])]]).dot(dqb)
    # here we write down new chassis config
    new_chassis_config = config[0,0:3].reshape(1,3)+dq.reshape(1,3)

    # here we put the angle and chassis configuration together
    new_config = np.hstack((new_chassis_config,new_angle_config))
    
    return new_config