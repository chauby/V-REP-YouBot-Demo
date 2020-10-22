import numpy as np
import modern_robotics as mr
from JointLimits import jointLimits

def feedbackControl(config, Xd, Xd_next, Kp, Ki, dt, jointlimit):
    # here we compute X
    M = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
    Blist = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0],[0,-1,0,-0.3526,0,0],[0,-1,0,-0.2176,0,0],[0,0,1,0,0,0]]).T
    Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
    thetalist_initial = np.array([config[0,3],config[0,4],config[0,5],config[0,6],config[0,7]])
    T_sb_initial = np.array([[np.cos(config[0,0]),-np.sin(config[0,0]),0,config[0,1]],[np.sin(config[0,0]),np.cos(config[0,0]),0,config[0,2]],[0,0,1,0.0963],[0,0,0,1]])
    X = T_sb_initial.dot(Tb0).dot(mr.FKinBody(M,Blist,thetalist_initial))

    # here we write down Vd
    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(np.linalg.inv(Xd).dot(Xd_next)))

    # here we write down Vb = Ad*Vd
    Vb = mr.Adjoint(np.linalg.inv(X).dot(Xd)).dot(Vd)

    # here we write down X_err
    X_err = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X).dot(Xd)))
    
    # here we write down commanded twist
    V = Vb+Kp*X_err+Ki*(X_err+X_err*dt)

    # here we compute J_e
    # first we write down J_arm
    Blist = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0],[0,-1,0,-0.3526,0,0],[0,-1,0,-0.2176,0,0],[0,0,1,0,0,0]]).T
    thetalist = np.array([config[0,3],config[0,4],config[0,5],config[0,6],config[0,7]])
    J_arm = mr.JacobianBody(Blist,thetalist)
    # second we write down J_base
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    gamma1 = -np.pi/4
    gamma2 = np.pi/4
    gamma3 = -np.pi/4
    gamma4 = np.pi/4
    beta = 0
    x1 = l
    y1 = w
    x2 = l
    y2 = -w
    x3 = -l
    y3 = -w
    x4 = -l
    y4 = w
    # here we write down F6
    a1 = np.array([1,np.tan(gamma1)])
    a2 = np.array([1,np.tan(gamma2)])
    a3 = np.array([1,np.tan(gamma3)])
    a4 = np.array([1,np.tan(gamma4)])
    b = np.array([[np.cos(beta),np.sin(beta)],[-np.sin(beta),np.cos(beta)]])
    c1 = np.array([[-y1,1,0],[x1,0,1]])
    c2 = np.array([[-y2,1,0],[x2,0,1]])
    c3 = np.array([[-y3,1,0],[x3,0,1]])
    c4 = np.array([[-y4,1,0],[x4,0,1]])
    h1 = (((1/r)*a1).dot(b)).dot(c1)
    h2 = (((1/r)*a2).dot(b)).dot(c2)
    h3 = (((1/r)*a3).dot(b)).dot(c3)
    h4 = (((1/r)*a4).dot(b)).dot(c4)
    H0 = np.vstack((h1,h2,h3,h4))
    F = np.linalg.pinv(H0)
    F6 = np.array([[0,0,0,0],[0,0,0,0],[F[0,0],F[0,1],F[0,2],F[0,3]],[F[1,0],F[1,1],F[1,2],F[1,3]],[F[2,0],F[2,1],F[2,2],F[2,3]],[0,0,0,0]])

    # then write down J_base
    T_sb = np.array([[np.cos(config[0,0]),-np.sin(config[0,0]),0,config[0,1]],[np.sin(config[0,0]),np.cos(config[0,0]),0,config[0,2]],[0,0,1,0.0963],[0,0,0,1]])
    T_eb = np.linalg.inv(X).dot(T_sb)
    J_base = mr.Adjoint(T_eb).dot(F6)

    # here we test joint limits
    if jointlimit == 1:
        jointLimits(config,J_arm)

    # finally we write down J_e
    J_e = np.hstack((J_base,J_arm))

    # here we write down speeds (u,thetadot)
    speeds = np.linalg.pinv(J_e,rcond=1e-2).dot(V)

    return V, speeds, X_err