%% EENG350 Mini Project Model Development
% Rachel Lumnitzer
clear; close all;

% This program uses the experimental data from Arduino for the motor
% voltage, angular position, and angular velocity used to
% determine the tuned system transfer function and PID controller.

%% Transfer Function
% Experimental Data from Motor Encoder using Arduino
Va = (100/255)*7.5; % scaled motor voltage using PWM value from Arduino

time = [0
0.05
0.1
0.15
0.2
0.25
0.301
0.351
0.401
0.451
0.501
0.551
0.602
0.652
0.702
0.752
0.802
0.852
0.902
0.952
1.002
1.052
1.102
1.153
1.203
1.253
1.303
1.353
1.403
]; % time in seconds

Position = [0
0.06
0.19
0.38
0.58
0.79
1.02
1.24
1.48
1.71
1.94
2.18
2.41
2.65
2.88
3.12
3.36
3.59
3.82
4.05
4.29
4.53
4.76
5
5.24
5.47
5.71
5.94
6.18
]; % Position in radians

Velocity = [0
1.2
2.857142857
3.673469388
4.081632653
4.375
4.489795918
4.693877551
4.693877551
4.693877551
4.897959184
4.693877551
4.897959184
4.693877551
4.897959184
4.897959184
4.897959184
4.897959184
4.791666667
4.791666667
4.897959184
4.897959184
4.897959184
4.897959184
4.897959184
4.897959184
4.693877551
4.897959184
4.791666667
]; % Velocity in radians per second

Velocity = Velocity/Va; % scale velocity and position by voltage to 
Position = Position/Va; % achieve unit step response

% Plots
figure(1)
plot(time,Position);
title('Experimental Angular Position');
xlabel('time(s)');
ylabel('radians');

figure(2)
plot(time,Velocity);
title('Experimental Angular Velocity Step Response');
xlabel('time(s)');
ylabel('rad/s');
xlim([0 1]); % adjust x axis limits
%% Simulated transfer function
% Variables
% find K from settling time
K = 1.65;
% find sigma from time at 0.64(K), 1/time
sigma = 9.5; % tuned value

% Simulated angular velocity
sys = tf((K*sigma),[1 sigma]); % transfer function
[y,t] = step(sys);

% Simulated angular position
sys2 = tf((K*sigma),[1 sigma 0]); % add integrator to transfer function
[y2,t2] = step(sys2);

% Plots
figure(4)
plot(t,y); 
title('Simulated Angular Velocity ');
xlim([0 1]);
xlabel('time(s)');
ylabel('rad/s');

figure(5)
plot(t2,y2);
title('Simulated Angular Position');
xlim([0 1]);
xlabel('time(s)');
ylabel('radians');
%% Transfer Function
% Plot angular velocity and position for both
figure(6)
plot(time,Velocity,t,y);
title('Simulated vs. Experimental Velocity');
xlim([0 1]);
xlabel('time(s)');
ylabel('rad/s');
legend('Experimental','Simulated');

figure(7)
plot(time,Position,t2,y2);
title('Simulated vs. Experimental Position');
xlim([0 1]);
xlabel('time(s)');
ylabel('rad');
legend('Experimental','Simulated');
%% Simulated PID Controller
% Process
out = sim('mini_project_controller_PID_final'); % assign Simulink file to plot
                                           % results from tuned PID
                                           % controller
% Plot
figure(8)
hold on
plot(out.output);
title('Angular Position- PID Controller'); % Simulated PID Controller
ylabel('Position (rad)');
xlim([1 2]); % adjust controller axis to remove 1 second initial delay

%% Experimental PID Controller
% Process
out = sim('mini_project_controller_PID_final_2'); % assign Simulink file to plot
                                           % results from experimental PID
                                           % controller
% adjust the Kp, Ki, and Kd gains in Simulink to match the experimental
% data
% Kp = 5.88235 V/s, Ki = 0, and Kd = 0.235294 V/rad*s

plot(out.output); % experimental PID Controller on same plot as simulated
ylabel('Position (rad)');
xlim([1 2]); % adjust controller axis to remove 1 second initial delay
legend('Simulated','Experimental');