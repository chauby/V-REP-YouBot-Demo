clear
close all

t1 = 5;
t2 = 15;
t3 = 20;

R1 = 1;
R2 = 1;

time_step = 0.01;

theta_1 = 0;
theta_2 = 0;
delta_theta = 0;
x = 0;
y = 0;
time = 0;

figure()
for t=time_step:time_step:t3
    if t <= t1
        tmp_theta_1 = theta_1(end) + pi*time_step/t1;
        tmp_theta_2 = 0;
    elseif t <= t2
        tmp_theta_1 = theta_1(end);
        tmp_theta_2 = theta_2(end) - 2*pi*time_step/(t2-t1);
    else
        tmp_theta_1 = theta_1(end) + pi*time_step/(t3-t2);
        tmp_theta_2 = theta_2(end);
    end
    
    theta_1 = [theta_1; tmp_theta_1];
    theta_2 = [theta_2; tmp_theta_2];
    
    x = [x; R1 - R1*cos(tmp_theta_1) + R2 - R2*cos(tmp_theta_2)];
    y = [y; R1*sin(tmp_theta_1) + R2*sin(tmp_theta_2)];
    
    x1 = x(end-1);
    y1 = y(end-1);
    x2 = x(end);
    y2 = y(end);
    delta_theta = [delta_theta; acos((x1*x2 + y1*y2)/(sqrt(x1^2 + y1^2)*sqrt(x2^2 + y2^2)))];
    
    time = [time, t];
    
    cla
    plot(x, y, '.-')
    xlim([0, 4])
    ylim([-2, 2])
    axis equal
    pause(time_step)
end

%% calculate velocity
vx = 0;
vy = 0;
omega = 0;

delta_theta(end) = delta_theta(end-1); % avoid NaN

for i =2:length(x)
    vx = [vx; (x(i) - x(i-1))/time_step];
    vy = [vy; (y(i) - y(i-1))/time_step];
    omega = [omega; delta_theta(i)/time_step];
end


%% plot
figure()
subplot(2,1,1)
plot(theta_1)
title('theta 1')

subplot(2,1,2)
plot(theta_2)
title('theta 2')

figure()
subplot(3,1,1)
hold on
plot(time, x)
plot(time, y)
title('x, y, theta')
xlabel('time')
ylabel('pos')
legend('x', 'y')
hold off

subplot(3,1,2)
hold on
plot(time, vx)
plot(time, vy)
title('vx and vy')
xlabel('time')
ylabel('vel')
legend('vx', 'vy')
hold off

subplot(3,1,3)
plot(time, omega)
title('omega')
xlabel('time')
ylabel('vel')
legend('omega')

v = [vx,vy,omega];

%%
csvwrite('center_velocity.csv',v);
