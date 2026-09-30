%% hw2_me501_Q2b.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('ME 501 HW2 simulation of pneumatic chambers dynamics, comparison with exact solution.')

%% Setup: constants and initial conditions

tmax = 5;
dt = 0.05;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% The matrix for our linear dynamics is
A = FILL_THIS_IN

%%%%% Initial conditions:

p1_0 = FILL_THIS_IN
p2_0 = FILL_THIS_IN
p3_0 = FILL_THIS_IN
x0 = [FILL_THIS_IN];

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
    x_traj(:,t+1) = FILL_THIS_IN
end

%% Exact Solution for comparison

% let's precalculate for a one-to-one comparison
x_traj_exact = zeros(size(x0,1), n+1);
x_traj_exact(:,1) = x0;

% precalculate via iteration
for k=1:n
    % The exact solution is x(t) = e^(At)*x0
    % Since "t" is inside the exponential, need to re-call this function
    % each iteration.
    t = k*dt; % here, k is index, dt is timestep, so total time is t.
    %%%%% If exponentiating at each timestep:
    Phi_t = FILL_THIS_IN
    x_traj_exact(:,k+1) = FILL_THIS_IN
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
plot(t, x_traj_exact(1,:), 'b')
legend(["Fwd Euler", "Exact Soln"]);
xlabel('time (sec)');
ylabel('pressure 1 (Pa)');
% pressure 2
subplot(3,1,2);
plot(t, x_traj(2,:), 'r');
hold on;
plot(t, x_traj_exact(2,:), 'b')
legend(["Fwd Euler", "Exact Soln"]);
xlabel('time (sec)');
ylabel('Pressure 2 (Pa)');
% pressure 3
subplot(3,1,3);
plot(t, x_traj(3,:), 'r');
hold on;
plot(t, x_traj_exact(3,:), 'b')
legend(["Fwd Euler", "Exact Soln"]);
xlabel('time (sec)');
ylabel('Pressure 3 (Pa)');















