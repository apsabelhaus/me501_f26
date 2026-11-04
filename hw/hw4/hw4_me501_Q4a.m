%% hw4_me501_Q4a.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('Calculating the zero-order hold discretization of continuous-time non-autonomous systems.')

%% Setup

% Continuous-time A and B matrices:
A = [0 1;
     4 -3];
B = [0; 1];

% We need to specify a sampling time:
dt = 0.01;

% We will also need an initial state:
x0 = [pi/4; 0];

% For a comparison against the forward-euler integrated original system,
% specify a total time of the simulation:
tmax = 2;
n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

%% Calculate the discretized non-autonomous system via the zero order hold

% Use the trick from the problem set to calculate both Ad and Bd at once,
% using the matrix exponential.
M = dt*[A, B; zeros(1,3)];
eM = expm(M);

% Extract the top-left 2x2 matrix as Ad and the top-right 2x1 vector as Bd
Ad = eM(1:2,1:2)
Bd = eM(1:2,3)

%% Simulate

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_trajExact = zeros(size(x0,1), n+1);
x_trajExact(:,1) = x0;

% and a trajectory of the system in controllable canonical realization
x_trajFW = zeros(size(x0,1), n+1);
x_trajFW(:,1) = x0;

% Let's do a step control input, fixed value for all time.
u=-5;

% Simulation (rollout) of our exact discretization
for k=1:n
    %%%%% DYNAMICS
    x_kp1 = Ad*x_trajExact(:,k) + Bd*u;
    % save the result
    x_trajExact(:,k+1) = x_kp1;
end

% Forward euler, which works as a baseline integration method always
for t=1:n
    %%%%% DYNAMICS
    bolddotx_t = A*x_trajFW(:,t) + B*u;
    % Forward Euler
    x_trajFW(:,t+1) = x_trajFW(:,t) + dt*bolddotx_t;
end

%% Plot

% We want to see all three states
figure;
hold on;
sgtitle('Exact discretization comparison, non-autonomous, with the zero order hold')
t = [0:dt:tmax];

% number of states here will be N
N = size(x0,1);
for i=1:N
    subplot(N,1,i)
    hold on;
    plot(t, x_trajExact(i,:), 'r');
    plot(t, x_trajFW(i,:), 'b');
    % a neat trick to obtain the string "x1", "x2", etc
    ylabel(strcat("x", num2str(i)));
    legend("Exact", "FW Euler")
end
xlabel('time (sec)')










