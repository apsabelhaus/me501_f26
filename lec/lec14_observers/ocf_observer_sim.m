% ocf_observer_sim.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('Simulation of a linear observer for a system in observable canonical form (OCF).')

%% Setup: constants and initial conditions

% We examined this A, B, C, in observable canonical form:

A = [-6  1 0;
     -11 0 1;
     -6  0 0];

B = [0;
     0;
     1];

C = [1 0 0];

% Choose an arbitrary initial condition (we should observe convergence of
% observer regardless of initial condition)
x0 = [1; 1; 1];

% initial condition of the observer:
xhat_0 = zeros(size(x0,1),1);

%% Observer Gains

% We calculated this L by matching coefficients of the open-loop versus
% desired characteristic polynomials.

% The open-loop coefficients: pass in a matrix to the poly() function,
% returns as a vector.
a = poly(A);
% ...but we don't care about the n-th coefficient, it always equals 1, just
% need 0...n-1.
a = a(2:end);
% The closed-loop coefficients: pass in a vector to the poly() function,
% and MATLAB interprets as the desired eigenvalues. The function is
% overloaded.
lambda_bar = [-4; -5; -6];
a_bar = poly(lambda_bar);
% ...but we don't care about the n-th coefficient, it always equals 1, just
% need 0...n-1.
a_bar = a_bar(2:end);

disp('Gains are:')
% need to transpose this vector into a column
L = (a_bar-a)'
% L = flip(L)

% Should obtain [9; 63; 114].

disp('Closed loop observer A_ocl:')
A_ocl = A-L*C
[V, D] = eig(A_ocl)

%% Simulate observer under some arbitrary controller

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
% we'll also save the trajectory of outputs, and the trajectory of
% estimated (observed) states.
xhat_traj = zeros(size(x0,1), n+1);
xhat_traj(:,1) = xhat_0;
y_traj = zeros(1, n);

% actual simulation! Use Forward Euler for now.
for t=1:n
    % Use an arbitrary control input for now. We're not trying to control
    % the system, just to see that our state estimates converge.
    u_t = 20*sin(0.1*t);
    u_traj(t) = u_t; % save for plotting later
    %%%%% DYNAMICS
    bolddotx_t = A*x_traj(:,t) + B*u_t;
    % Forward Euler
    x_traj(:,t+1) = x_traj(:,t) + dt*bolddotx_t;
    %%%%% OBSERVER
    % The output at this timestep is from the ground-truth states
    y_t = C*x_traj(:,t);
    % The observer dynamics are:
    bolddotxhat_t = A*xhat_traj(:,t) + B*u_t + L*(y_t - C*xhat_traj(:,t));
    % Forward Euler:
    xhat_traj(:,t+1) = xhat_traj(:,t) + dt*bolddotxhat_t;
end

%% Plot

% We can also plot the three time series trajectories of x and u
figure;
hold on;
t = [0:dt:tmax];
subplot(4,1,1);
sgtitle('Simulation:');
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









