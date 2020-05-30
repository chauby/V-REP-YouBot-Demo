function [v_wheel] = chassisInverseKinematics(vx, vy, omega, wheel_R, a, b)
    omega_1 = (vy - vx + (a+b)*omega)/wheel_R;
    omega_2 = (vy + vx - (a+b)*omega)/wheel_R;
    omega_3 = (vy - vx - (a+b)*omega)/wheel_R;
    omega_4 = (vy + vx + (a+b)*omega)/wheel_R;
    
    % set the direction for each wheel
    v_wheel = [0,0,0,0];
    v_wheel(1) = -omega_1;
    v_wheel(2) = -omega_2;
    v_wheel(3) = -omega_3;
    v_wheel(4) = -omega_4;
end
