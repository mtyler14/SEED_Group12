%% EENG350 Demo 2 
% Rachel Lumnitzer
close all; clear;

% This program uses the experimental data from Arduino for the motor
% voltage and angular velocities to determine the tuned system transfer 
% functions and PID controllers for each motor. It also compares the 
% responses based on applying a faster speed.

%% Angular Velocity 1
time = [ % time in seconds
0.25
0.3
0.35
0.4
0.45
0.5
0.55
0.6
0.65
0.7
0.75
0.8
0.85
0.9
0.95
1
1.05
1.1
1.15
1.2
1.25
1.3
1.35
1.4
1.45
1.5
1.55
1.6
1.65
1.7
1.75
1.8
1.85
1.9
1.95
2
2.05
2.1
2.15
2.2
2.25
2.3
2.35
2.4
2.45
2.5
2.55
2.61
2.66
2.71
2.75
2.8
2.85
2.91
2.96
3.01
3.06
3.11
3.16
3.2
3.25
3.3
3.35
3.4
3.45
3.5
3.55
3.6
3.65
3.7
3.75
3.8
3.85
3.9
3.95
4
4.05
4.1
4.15
4.2
4.25
4.3
4.35
4.4
4.45
4.5
4.55
4.6
4.66
4.7
4.75
4.8
4.85
4.9
4.96
5.01
5.05
5.11
5.16
5.2
5.25
5.3
5.35
5.4
5.45
5.51
5.55];
time = time-.25; % shift time to 0

ang_velocity1 = [ % angular velocity in rad/s for 0.5V for each motor
0.02
0.24
0.64
0.48
0.56
0.72
0.72
0.63
0.64
0.65
0.72
0.8
0.88
0.94
1.04
1.04
0.72
0.64
0.71
0.67
0.65
0.72
0.64
0.56
0.63
0.64
0.64
0.56
0.64
0.67
0.72
0.72
0.72
0.72
0.72
0.64
0.63
0.64
0.64
0.64
0.58
0.64
0.56
0.64
0.56
0.64
0.64
0.63
0.64
0.64
0.67
0.65
0.64
0.55
0.56
0.48
0.56
0.56
0.48
0.51
0.49
0.56
0.56
0.56
0.56
0.65
0.64
0.64
0.64
0.67
0.57
0.57
0.56
0.48
0.56
0.48
0.55
0.56
0.48
0.5
0.49
0.48
0.55
0.64
0.56
0.64
0.56
0.64
0.71
0.67
0.57
0.56
0.48
0.64
0.55
0.48
0.48
0.48
0.48
0.58
0.58
0.56
0.56
0.56
0.56
0.63
0.64];


%% Angular Velocity 2
time2 = [ % time in seconds
0.25
0.3
0.35
0.4
0.45
0.5
0.55
0.6
0.65
0.7
0.75
0.8
0.85
0.9
0.95
1
1.05
1.1
1.15
1.2
1.25
1.3
1.35
1.4
1.45
1.5
1.55
1.6
1.65
1.7
1.75
1.8
1.85
1.9
1.95
2
2.05
2.1
2.15
2.2
2.25
2.3
2.35
2.4
2.45
2.5
2.55
2.61
2.66
2.71
2.75
2.8
2.85
2.91
2.96
3.01
3.06
3.11
3.16
3.2
3.25
3.3
3.35
3.4
3.45
3.5
3.55
3.6
3.65
3.7
3.75
3.8
3.85
3.9
3.95
4
4.05
4.1
4.15
4.2
4.25
4.3
4.35
4.4
4.45
4.5
4.55
4.6
4.66
4.7
4.75
4.8
4.85
4.9
4.96
5.01
5.05
5.11
5.16
5.2
5.25
5.3
5.35
5.4
5.45
5.51
5.55];
time2 = time2-0.25;

ang_velocity2 = [ % angular velocity in rad/s for 0.5V for each motor
0
0.39
0.64
0.4
0.64
0.56
0.64
0.63
0.64
0.57
0.64
0.56
0.64
0.55
0.48
0.72
0.96
0.8
0.86
0.84
0.74
0.8
0.72
0.64
0.55
0.64
0.64
0.64
0.64
0.67
0.72
0.8
0.72
0.72
0.72
0.8
0.71
0.72
0.64
0.64
0.67
0.56
0.56
0.64
0.64
0.64
0.72
0.63
0.72
0.72
0.75
0.65
0.64
0.71
0.72
0.64
0.56
0.48
0.64
0.51
0.49
0.56
0.56
0.56
0.72
0.65
0.64
0.64
0.72
0.75
0.65
0.74
0.56
0.56
0.64
0.56
0.55
0.48
0.56
0.58
0.57
0.56
0.47
0.56
0.56
0.72
0.64
0.64
0.55
0.67
0.74
0.72
0.64
0.64
0.63
0.64
0.56
0.56
0.48
0.58
0.67
0.56
0.56
0.56
0.64
0.71
0.72];

%% Experimental Data- Step Responses
figure(1)
plot(time,ang_velocity1);
title('Motor 1- Exp Angular Velocity');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');

figure(2)
plot(time2,ang_velocity2);
title('Motor 2- Exp Angular Velocity');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');

%% Simulated Data- Transfer Functions
K = 0.65; % settling time
sigma = 10; % time constant

K1 = 0.63; % settling time
sigma1 = 12; % time constant

figure(3)
sys = tf((K*sigma),[1 sigma]); % transfer function
[y,t] = step(sys);
plot(t,y);
title('Motor 1- Simulated Angular Velocity');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');

figure(4)
sys2 = tf((K1*sigma1),[1 sigma1]); % transfer function
[y2,t2] = step(sys2);
plot(t2,y2);
title('Motor 2- Simulated Angular Velocity');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');

%% Experimental vs. Simulated Angular Velocity
figure(5)
plot(time,ang_velocity1,t,y);
xlim([0 2]);
title('Motor 1-Step Response Comparison');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');

figure(6)
plot(time2,ang_velocity2,t2,y2);
xlim([0 2]);
title('Motor 2-Step Response Comparison');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');

%% Experimental vs. Simulated Angular Velocity
sys_1 = tf((K*sigma),[1 sigma 0]); % transfer function
[y3,t3] = step(sys_1);
figure(7)
plot(t3,y3, time,ang_velocity1.*time);
legend('Simulated','Experimental');
xlim([0 5]);
xlabel('time (seconds)');
ylabel('Position (radians)');
title('Motor 1- Angular Position Comparison');

sys_2 = tf((K1*sigma1),[1 sigma1 0]); % transfer function
[y4,t4] = step(sys_2);
figure(8)
plot(t4,y4, time2,ang_velocity2.*time2);
legend('Simulated','Experimental');
xlim([0 5]);
xlabel('time (seconds)');
ylabel('Position (radians)');
title('Motor 2- Angular Position Comparison');

%% PID Controllers- Angular Velocity
% Process
out = sim('Demo_2_controller1'); % assign Simulink file to plot
                                           % results from tuned controller
% Plot
figure(9)
plot(out.output1);
title('Motor 1- P Controller');
ylabel('Velocity (rad/s)');
xlim([1 2]);
hold on

out = sim('Demo_2_controller1_speed'); % assign Simulink file to plot
plot(out.outputs);
legend('Reference','Faster Speed');
hold off

figure(10)
out = sim('Demo_2_controller2'); % assign Simulink file to plot
plot(out.output2);
title('Motor 2- P Controller');
ylabel('Velocity (rad/s)');
xlim([1 2]);
hold on

out = sim('Demo_2_controller2_speed'); % assign Simulink file to plot
plot(out.outputss);
legend('Reference','Faster Speed');
hold off

%% Simulated Faster Angular Velocity- Motor 1
sys_speed = sys*4; % scale to 2 rad/s for each motor
[y_s,t_s] = step(sys_speed); % assign faster simulated step response

figure(11)
plot(t,y,t_s,y_s);
legend('Reference','Faster Speed');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');
title('Motor 1 Faster Speed');

%% Simulated Faster Angular Velocity- Motor 2
sys_speed2 = sys2*4; % scale to 2 rad/s for each motor
[y_s2,t_s2] = step(sys_speed2); % assign faster simulated step response

figure(12)
plot(t2,y2,t_s2,y_s2);
legend('Reference','Faster Speed');
xlabel('time (seconds)');
ylabel('Velocity (rad/s)');
title('Motor 2 Faster Speed');

%% Simulated Faster Angular Position- Motor 1
sys_pos = tf((K*sigma),[1 sigma 0]);
[yp,tp] = step(sys_pos);

sys_speed_pos = tf((K*4*sigma),[1 sigma 0]); % add integrator
[ysp,tsp] = step(sys_speed_pos);

figure(13)
plot(tp,yp,tsp,ysp);
legend('Reference','Faster Speed');
xlabel('time (seconds)');
ylabel('Position (radians)');
xlim([0 10]);
title('Motor 1 Faster Speed - Position');

%% Simulated Faster Angular Position- Motor 2
sys_pos2 = tf((K1*sigma1),[1 sigma1 0]);
[yp2,tp2] = step(sys_pos2);

sys_speed_pos2 = tf((K1*4*sigma1),[1 sigma1 0]); % add integrator
[ysp2,tsp2] = step(sys_speed_pos2);

figure(14)
plot(tp2,yp2,tsp2,ysp2);
legend('Reference','Faster Speed');
xlabel('time (seconds)');
ylabel('Position (radians)');
xlim([0 10]);
title('Motor 2 Faster Speed - Position');