% hw5_me501_Q2b.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('ME 501 HW5 Q2b - simulating state feedback.')

%% Setup: constants and initial conditions

% We examined this A and B, not in controllable canonical form:

A = [0 1 0;
     1 1 1;
     0 1 2];

B = [1;
     0;
     0];

% Choose an arbitrary initial condition (we should observe stability
% regardless of initial condition)
x0 = [1; 1; 1];

%% Control gains calculations

%%%% Ackermann's formula

Qinv = [1 0 -1;
        0 1 -1;
        0 0  1];

% The characteristic equation with our A but the desired a_bar
% coefficients is:
phi_d = [13 20 9;
         20 42 38
         9  38 71];

% the "row selector" vector is
e3 = [0; 0; 1];

% the gain matrix via this method is therefore:
K = (e3')*Qinv*phi_d

disp('Closed loop A_cl with Ackermanns formula:')
A_cl = A-B*K
[V_k, D_k] = eig(A_cl)


%% Simulate state feedback controller

% For now, we'll use Forward Euler, since you're more familiar with it,
% but we should also expect to try out the Zero Order Hold exact
% discretization for comparison.

tmax = 10;
dt = 0.01;
n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_traj = zeros(size(x0,1), n+1);
x_traj(:,1) = x0;
% Let's also save the u trajectory that we calculate, for later evaluation.
u_traj = zeros(1, n);

% actual simulation! Use Forward Euler for now.
for t=1:n
    % Calculate the control input at this time, u = -kx
    u_t = -K*x_traj(:,t);
    u_traj(t) = u_t; % save for plotting later
    %%%%% DYNAMICS
    bolddotx_t = A*x_traj(:,t) + B*u_t;
    % Forward Euler
    x_traj(:,t+1) = x_traj(:,t) + dt*bolddotx_t;
end

%% Plot

% We can also plot the three time series trajectories of x and u
figure;
hold on;
t = [0:dt:tmax];
subplot(4,1,1);
sgtitle('Simulation, State Feedback Control');
% x1
plot(t, x_traj(1,:), 'r');
hold on;
ylabel('x1');
% x2
subplot(4,1,2);
plot(t, x_traj(2,:), 'r');
ylabel('x2');
% x3
subplot(4,1,3);
plot(t, x_traj(3,:), 'r');
ylabel('x3');
% u
subplot(4,1,4);
plot(t(1:end-1), u_traj(:), 'r'); % there is one fewer control input than state, over time.
ylabel('u');
xlabel('time (sec)');









