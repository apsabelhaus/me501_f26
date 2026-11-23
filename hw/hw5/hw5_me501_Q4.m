% hw5_me501_Q3b.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('ME 501 HW5 Q3b - simulating state observer.')

%% Setup: constants and initial conditions

% We examined this A and B, not in controllable canonical form:

A = [0 1 0;
     1 1 1;
     0 1 2];

B = [1;
     0;
     0];

C = [1 0 0];

% Choose an arbitrary initial condition (we should observe stability
% regardless of initial condition)
x0 = [1; 1; 1];

% initial condition of the observer:
xhat_0 = zeros(size(x0,1),1);

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


%% Observer gains calculations

%%%% Ackermann's formula

Ninv = [1 0 0;
        0 1 0;
        -1 -1  1];

% The characteristic equation with our A but the desired a_bar
% coefficients is:
phi_d = [5 9 6;
         9 20 21;
         6 21 35];

% the "row selector" vector is
e3 = [0; 0; 1];

% the gain matrix via this method is therefore:
L = phi_d*Ninv*e3

disp('Closed loop observer A_co with Ackermanns formula:')
A_co = A-L*C
[V_o, D_o] = eig(A_co)


%% Simulate observer state feedback (compensator)

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
% Let's also save the u trajectory that we calculate, for later evaluation.
u_traj = zeros(1, n);
x_traj(:,1) = x0;
% we'll also save the trajectory of outputs, and the trajectory of
% estimated (observed) states.
xhat_traj = zeros(size(x0,1), n+1);
xhat_traj(:,1) = xhat_0;
y_traj = zeros(1, n);

% actual simulation! Use Forward Euler for now.
for t=1:n
    % % Calculate the control input at this time, u = -kx
    u_t = -K*x_traj(:,t);
    u_traj(t) = u_t; % save for plotting later
    %%%%% DYNAMICS
    bolddotx_t = A*x_traj(:,t) + B*u_t;
    % Forward Euler
    x_traj(:,t+1) = x_traj(:,t) + dt*bolddotx_t;
    %%%%% OBSERVER
    % The output at this timestep is from the ground-truth states
    y_t = C*x_traj(:,t);
    y_traj(t) = y_t;
    % The observer dynamics are:
    bolddotxhat1_t = A*xhat_traj(:,t) + B*u_t + L*(y_t - C*xhat_traj(:,t));
    % Forward Euler:
    xhat_traj(:,t+1) = xhat_traj(:,t) + dt*bolddotxhat1_t;
end

%% Plot

% We can also plot the three time series trajectories of x and u
figure;
hold on;
t = [0:dt:tmax];
subplot(5,1,1);
sgtitle('Simulation: Observer State Feedback');
% x1
hold on;
plot(t, x_traj(1,:), 'r');
plot(t, xhat_traj(1,:), 'b');
hold on;
ylabel('x1');
legend(["True", "Estimate"]);
% x2
subplot(5,1,2);
hold on;
plot(t, x_traj(2,:), 'r');
plot(t, xhat_traj(2,:), 'b');
ylabel('x2');
% x3
subplot(5,1,3);
hold on;
plot(t, x_traj(3,:), 'r');
plot(t, xhat_traj(3,:), 'b');
ylabel('x3');
% u
subplot(5,1,4);
hold on;
plot(t(1:end-1), u_traj(:), 'r'); % there is one fewer control input than state, over time.
ylabel('u');
% y
subplot(5,1,5);
hold on;
plot(t(1:end-1), y_traj(:), 'r'); % there is one fewer observation than state, over time.
ylabel('y');
xlabel('time (sec)');








