% observer_eigval_assignment_sim.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('Simulation of linear observers for a system not in observable canonical form (OCF).')

%% Setup: constants and initial conditions

% We examined this A, B, C, not in observable canonical form:

A = [-12 1 12;
     -11 0 12;
     -6  0 6];

B = [1;
     0;
     1];

C = [1 0 -1];

% Choose an arbitrary initial condition (we should observe convergence of
% observer regardless of initial condition)
x0 = [1; 1; 1];

% initial condition of the observer:
xhat_0 = zeros(size(x0,1),1);

%% Observer Gains Calculations

%%%% Approach 1: state transformation, then place eigenvalues in
%%%% observable canonical form

P = [1 0 -1;
     0 1 0;
     0 0 1];

Pinv = inv(P);

% We calculated this L for the corresponding observable canonical form:
L_ocf = [9;
         63;
         114];


% The gain matrix via this method is therefore:
L_1 = Pinv*L_ocf

disp('Closed loop A-LC with the state transformation method:')
A_lc1 = A-L_1*C
[V_1, D_1] = eig(A_lc1)

%%%% Approach 2: Ackermann's formula

% Calculate the observability matrix:
N = [C;
     C*A;
     C*A^2];

% ...but we really need its inverse:
Ninv = inv(N);

% The characteristic equation with our A but the desired a_bar
% coefficients is:
phi_d = A^3 + 15*(A^2) + 74*A + 120*eye(3)

% the "row selector" vector is
e3 = [0; 0; 1];

% the gain matrix via this method is therefore:
L_2 = phi_d*Ninv*e3

disp('Closed loop A_-LC with Ackermanns formula:')
A_lc2 = A-L_2*C
[V_2, D_2] = eig(A_lc2)

%% Simulate observer under some arbitrary controller

% For now, we'll use Forward Euler, since you're more familiar with it,
% but we should also expect to try out the Zero Order Hold exact
% discretization for comparison.

tmax = 3;
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
xhat1_traj = zeros(size(x0,1), n+1);
xhat1_traj(:,1) = xhat_0;
y_traj = zeros(1, n);
% We'll simulate both methods at the same time
xhat2_traj = zeros(size(x0,1), n+1);
xhat2_traj(:,1) = xhat_0;

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
    y_traj(t) = y_t;
    % The observer dynamics are:
    bolddotxhat1_t = A*xhat1_traj(:,t) + B*u_t + L_1*(y_t - C*xhat1_traj(:,t));
    bolddotxhat2_t = A*xhat1_traj(:,t) + B*u_t + L_2*(y_t - C*xhat2_traj(:,t));
    % Forward Euler:
    xhat1_traj(:,t+1) = xhat1_traj(:,t) + dt*bolddotxhat1_t;
    xhat2_traj(:,t+1) = xhat2_traj(:,t) + dt*bolddotxhat2_t;
end

%% Plot

% We can also plot the three time series trajectories of x and u
figure;
hold on;
t = [0:dt:tmax];
subplot(5,1,1);
sgtitle('Simulation: Linear Observer, State Transformation');
% x1
hold on;
plot(t, x_traj(1,:), 'r');
plot(t, xhat1_traj(1,:), 'b');
hold on;
ylabel('x1');
legend(["True", "Estimate"]);
% x2
subplot(5,1,2);
hold on;
plot(t, x_traj(2,:), 'r');
plot(t, xhat1_traj(2,:), 'b');
ylabel('x2');
% x3
subplot(5,1,3);
hold on;
plot(t, x_traj(3,:), 'r');
plot(t, xhat1_traj(3,:), 'b');
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

% For the second simulation:
figure;
hold on;
t = [0:dt:tmax];
subplot(5,1,1);
sgtitle('Simulation: Linear Observer, Ackermanns Formula');
% x1
hold on;
plot(t, x_traj(1,:), 'r');
plot(t, xhat2_traj(1,:), 'g');
hold on;
ylabel('x1');
legend(["True", "Estimate"]);
% x2
subplot(5,1,2);
hold on;
plot(t, x_traj(2,:), 'r');
plot(t, xhat2_traj(2,:), 'g');
ylabel('x2');
% x3
subplot(5,1,3);
hold on;
plot(t, x_traj(3,:), 'r');
plot(t, xhat2_traj(3,:), 'g');
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







