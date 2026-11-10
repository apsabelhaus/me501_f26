% linear_pendulum_sim.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('Testing closed-loop linear feedback controllers for the linearized pendulum.')

%% Setup: constants and initial conditions

% We examined this A and B for the pendulum:

A = [0 1;
     4 -3];

B = [0;
     1];

% Choose an arbitrary initial condition (we should observe stability
% regardless of initial condition)
x0 = [1; 1];

%% Control gains 

% Chosen by hand for now. We then check and see what eigenvalues we get.

k1 = 10;
k2 = 2;
K = [k1, k2]; % this is a row vector!

disp('Closed loop A_cl with chosen gains:')
A_cl = A-B*K
[V, D] = eig(A_cl)


%% Simulate

% For now, we'll use Forward Euler, since you're more familiar with it,
% but we should also expect to try out the Zero Order Hold exact
% discretization for comparison.

tmax = 5;
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

% We can also plot the time series trajectories of x and u
figure;
hold on;
t = [0:dt:tmax];
subplot(3,1,1);
sgtitle('Simulation, Linearized Pendulum');
% x1
plot(t, x_traj(1,:), 'r');
ylabel('x1');
% x2
subplot(3,1,2);
plot(t, x_traj(2,:), 'r');
ylabel('x2');
% u
subplot(3,1,3);
plot(t(1:end-1), u_traj(:), 'r'); % there is one fewer control input than state, over time.
ylabel('u');
xlabel('time (sec)');










