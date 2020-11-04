%% EENG350 Demo 1 Angular Velocity Step Response Experiments
% Rachel Lumnitzer
close all; clear;

% This program uses the experimental data from Arduino for the motor
% voltage and angular velocities to determine the tuned system transfer 
% functions and PID controllers for each motor.

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

ang_velocity1 = [ % angular velocity in rad/s
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

ang_velocity2 = [ % angular velocity in rad/s
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

%% Experimental vs. Simulated
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

%% PID Controllers
% Process
out = sim('Demo_1_controller1_1'); % assign Simulink file to plot
                                           % results from tuned controller
% Plot
figure(7)
plot(out.output);
title('Motor 1- P Controller');
ylabel('Velocity (rad/s)');
xlim([1 2]);

% Process
out = sim('Demo_1_controller2_1'); % assign Simulink file to plot
                                           % results from tuned controller                           
% Plot
figure(8)
plot(out.output);
title('Motor 2- P Controller');
ylabel('Velocity (rad/s)');
xlim([1 2]);