% linear_pendulum_sim.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('Example of solving an ODE, comparing numerical integration versus the matrix exponential, using the linearized pendulum as an example.')

%% Setup: constants and initial conditions

tmax = 5;
dt = 0.2;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% The matrix for our linear dynamics is
% A = [0, 1;
%      -10, -2]; % With oscillations (complex eigenvalues, we'll do this later)
A = [0, 1;
    -10, -7]; % no oscillations (real eigenvalues)

%%%%% Initial conditions:

theta_0 = 2*pi/3; % mass angle from vertical. Positive is clockwise.
dottheta_0 = 0; % moving tangentially / rotation. tryL pi/24?
x0 = [theta_0; dottheta_0];

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

%% Exact Solution for comparison

% let's precalculate for a one-to-one comparison
x_traj_exact = zeros(size(x0,1), n+1);
x_traj_exact(:,1) = x0;

% For the specific A matrix above, we will learn how to perform the
% "diagonalization" procedure to obtain A = V*D*inv(V), with these
% matrices:
V = [1, 1;
     -2, -5];

invV = [5/3, 1/3;
        -2/3, -1/3];

% D = [-2, 0; 0, -5]; % we won't use this D matrix directly, see below.

% precalculate via iteration
for k=1:n
    % The exact solution is x(t) = e^(At)*x0
    % Since "t" is inside the exponential, need to re-call this function
    % each iteration.
    t = k*dt; % here, k is index, dt is timestep, so total time is t.
    %%%%% If exponentiating at each timestep:
    % Phi_t = expm(A*t);
    % x_traj_exact(:,k+1) = Phi_t*x0;
    %%%%% If we have an exact solution available as a function:
    eDt = [exp(-2*t), 0;
           0, exp(-5*t)];
    eAt = V*eDt*invV;
    x_traj_exact(:,k+1) = eAt*x0;
end

%% Plot

% We can also plot the three time series trajectories of theta, dottheta,
% and u
figure;
hold on;
t = [0:dt:tmax];
subplot(2,1,1);
sgtitle('Trajectories of states');
% theta
plot(t, x_traj(1,:), 'r');
hold on;
plot(t, x_traj_exact(1,:), 'b')
legend(["Fwd Euler", "Exact Soln"]);
xlabel('time (sec)');
ylabel('theta (rad)');
% velocity
subplot(2,1,2);
plot(t, x_traj(2,:), 'r');
hold on;
plot(t, x_traj_exact(2,:), 'b')
legend(["Fwd Euler", "Exact Soln"]);
xlabel('time (sec)');
ylabel('dtheta/dt (rad/sec)');
















