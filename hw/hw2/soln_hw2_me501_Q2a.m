%% hw2_me501_Q2a.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('ME 501 HW2 simulation of pneumatic chambers dynamics.')

%% Setup: constants and initial conditions

tmax = 5;
dt = 0.01;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% The matrix for our linear dynamics is
A = [-3  1  0;
      1 -2  1;
      0  1 -3];

%%%%% Initial conditions:

p1_0 = 5;
p2_0 = 0;
p3_0 = 50;
x0 = [p1_0; p2_0; p3_0];

%% Simulate to obtain trajectory - Numerical, approximated!

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_traj = zeros(size(x0,1), n+1);
x_traj(:,1) = x0;

% actual simulation! Use Forward Euler for now.
for t=1:n
    %%%%% DYNAMICS
    bolddotx_t = A*x_traj(:,t);
    % Forward Euler
    x_traj(:,t+1) = x_traj(:,t) + dt*bolddotx_t;
end

%% Plot

% We can also plot the three time series trajectories of theta, dottheta,
% and u
figure;
hold on;
t = [0:dt:tmax];
subplot(3,1,1);
sgtitle('Trajectories of states');
% pressure 1
plot(t, x_traj(1,:), 'r');
hold on;
legend(["Fwd Euler"]);
xlabel('time (sec)');
ylabel('pressure 1 (Pa)');
% pressure 2
subplot(3,1,2);
plot(t, x_traj(2,:), 'r');
hold on;
legend(["Fwd Euler"]);
xlabel('time (sec)');
ylabel('Pressure 2 (Pa)');
% pressure 3
subplot(3,1,3);
plot(t, x_traj(3,:), 'r');
hold on;
legend(["Fwd Euler"]);
xlabel('time (sec)');
ylabel('Pressure 3 (Pa)');















