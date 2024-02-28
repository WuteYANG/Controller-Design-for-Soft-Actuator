%% PID and LQR controller Design for the Soft Actuator
% Requirements: Hope the soft actuator has settling time below 1 sec.
% Soft actuators have compliance but slow responses compared to traditional
% robots, so hope to improve it by controller algorithm.
clear all;
close all;
clc;

%% SPA System Model
% Controllable Canonical Form
damping = 0.7; % damping ratio is obtained by 2nd-order sys id of SPA
wn = 1.812;    % natural frequency is obtained by mechanical analysis 
A = [0 1 0; 0 0 1; 0 -2*damping*wn -wn*wn];
B = [0;0;1];
C = [0.455 0 0];
D = 0;
sys = ss(A,B,C,D);
[b, a] = ss2tf(A,B,C,D);
T = tf(b, a);  % System transfer function

%% PID controller Design
% PID controller

% Method 1: Ziegler-Nicoles
% There are two approaches in Ziegler-Nicoles (time domain and freq domain
% method). This m file used time domain method to get Kd, Ki, Kp.
Kd = 0.25;
Ki = 1;
Kp = 3.6;
num = [Kd Kp Ki];
den = [1 0];
pid = tf(num, den);
Tcl = pid*T/(1+pid*T); % Closed-loop TF with PID controller

% Method 2: pidTuner in MATLAB
pidTuner(sys);
% The parameter can be adjusted in the popped window

%% LQR Controller Design
% LQR penalty function definition
p = 2000;  % p is a parameter to tune the weighting of Q matrix
Q = p*[1 0 0; 0 0.1 0; 0 0 0]; % The (1,1) element is to penalize theta and 
% the (2,2) element is to penalize theta_dot to avoid overshoot
R = 1;
% Solve the Ricatti Equation to get control gain K
[K,S,P] = lqr(A,B,Q,R); 
% Nbar aims to adjust the gain of B matrix which is penalized by Q&R matrices
Nbar = -inv(C*inv(A-B*K)*B);  
% System with LQR controller
sys_lqr = ss(A-B*K, B*Nbar, C, D);

%% Visualization
step_size = 200;
t = linspace(0,20,step_size);
sp = (pi/2)*ones(1, step_size);
figure();
PID = (pi/2)*step(Tcl,t);
lqr = (pi/2)*step(sys_lqr, t);
plot(t, sp, 'color', [0.5 0.5 0.5], 'linewidth', 2.5); hold on;
plot(t, PID, 'm--', 'linewidth', 3); hold on;
plot(t, lqr, 'g:', 'linewidth', 3);
xlabel('Time [sec]', 'fontsize', 11);
ylabel('Bending angle [deg]', 'fontsize', 11);
title('PID controller vs LQR controller', 'fontsize', 12);
legend({'Setpoint', 'PID controller', 'LQR controller'}, 'fontsize', 10);