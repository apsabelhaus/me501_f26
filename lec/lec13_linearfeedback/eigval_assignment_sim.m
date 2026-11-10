% eigval_assignment_sim.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('Simulation of linear state feedback and two eigenvalue assignment calculations: state transformation, or alternative, Ackermanns formula.')

%% Setup: constants and initial conditions

% We examined this A and B, not in controllable canonical form:

A = [0 1 0;
     2 -1 0;
     2 -1 -1];

B = [0;
     1;
     1];

% Choose an arbitrary initial condition (we should observe stability
% regardless of initial condition)
x0 = [1; 1; 1];

%% Control gains calculations

%%%% Approach 1: state transformation, then place eigenvalues in
%%%% controllable canonical form

T = [1 1 0;
     0 1 1;
     0 0 1];

Tinv = inv(T);

% We calculated this K for the corresponding controllable canonical form:
K_ccf = [26 27 7];

% The gain matrix via this method is therefore:
K_1 = K_ccf*Tinv

disp('Closed loop A_cl with the state transformation method:')
A_cl1 = A-B*K_1
[V_1, D_1] = eig(A_cl1)

%%%% Approach 2: Ackermann's formula

% Calculate the controllability matrix:
Q = [B, A*B, (A^2)*B];
% ...but we really need its inverse:
Qinv = inv(Q);

% The characteristic equation for the desired eigenvalues had these
% coefficients: [abar0, abar2, abar2]:
a_bar = [24; 26; 9];

% The characteristic equation with our A but the desired a_bar
% coefficients is:
phi_d = A^3 + 9*(A^2) + 26*A + 24*eye(3)

% the "row selector" vector is
e3 = [0; 0; 1];

% the gain matrix via this method is therefore:
K_2 = (e3')*Qinv*phi_d

disp('Closed loop A_cl with Ackermanns formula:')
A_cl2 = A-B*K_2
[V_2, D_2] = eig(A_cl2)


%% Simulate both controllers

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
x_traj1 = zeros(size(x0,1), n+1);
x_traj1(:,1) = x0;
% Let's also save the u trajectory that we calculate, for later evaluation.
u_traj1 = zeros(1, n);
% For Approach 2:
x_traj2 = zeros(size(x0,1), n+1);
x_traj2(:,1) = x0;
u_traj2 = zeros(1, n);

% actual simulation! Use Forward Euler for now.
% Approach 1:
for t=1:n
    % Calculate the control input at this time, u = -kx
    u_t = -K_1*x_traj1(:,t);
    u_traj1(t) = u_t; % save for plotting later
    %%%%% DYNAMICS
    bolddotx_t = A*x_traj1(:,t) + B*u_t;
    % Forward Euler
    x_traj1(:,t+1) = x_traj1(:,t) + dt*bolddotx_t;
end
% Approach 2:
for t=1:n
    % Calculate the control input at this time, u = -kx
    u_t = -K_2*x_traj2(:,t);
    u_traj2(t) = u_t; % save for plotting later
    %%%%% DYNAMICS
    bolddotx_t = A*x_traj2(:,t) + B*u_t;
    % Forward Euler
    x_traj2(:,t+1) = x_traj2(:,t) + dt*bolddotx_t;
end

%% Plot

% We can also plot the three time series trajectories of x and u
figure;
hold on;
t = [0:dt:tmax];
subplot(4,1,1);
sgtitle('Simulation, Approach 1');
% x1
plot(t, x_traj1(1,:), 'r');
hold on;
legend(["Approach 1 (State Transf.)"]);
ylabel('x1');
% x2
subplot(4,1,2);
plot(t, x_traj1(2,:), 'r');
ylabel('x2');
% x3
subplot(4,1,3);
plot(t, x_traj1(3,:), 'r');
ylabel('x3');
% x4
subplot(4,1,4);
plot(t(1:end-1), u_traj1(:), 'r'); % there is one fewer control input than state, over time.
ylabel('u');
xlabel('time (sec)');

% For the second approach:

figure;
hold on;
subplot(4,1,1);
sgtitle('Simulation, Approach 2');
% x1
plot(t, x_traj2(1,:), 'b');
hold on;
legend(["Approach 2 (Ackermanns Formula.)"]);
ylabel('x1');
% x2
subplot(4,1,2);
plot(t, x_traj2(2,:), 'b');
ylabel('x2');
% x3
subplot(4,1,3);
plot(t, x_traj2(3,:), 'b');
ylabel('x3');
% x4
subplot(4,1,4);
plot(t(1:end-1), u_traj2(:), 'b'); % there is one fewer control input than state, over time.
ylabel('u');
xlabel('time (sec)');









