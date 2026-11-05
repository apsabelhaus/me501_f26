%% hw4_me501_Q3a.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('Calculating minimum energy control sequences.')

%% Setup

% We're simulating a discrete-time system, so all we need is a total number
% of timesteps. Call that T.
T = 3;

% Our initial condition and goal final condition:
x0 = [1; 2; -2];
xf = [0; 0; 0];

% for looping later: pick out number of states.
n = size(x0,1);

% Please put the A and B matrices from problem 2 here:
A = FILL_THIS_IN;

B = FILL_THIS_IN;

%% Calculate a minimum energy sequence 

u_Tk = u_min_sequence(A, B, T, x0, xf);
% remember, the sequence of inputs is opposite in time!
% we need to apply u_kT rather than Tk.
u_kT = flip(u_Tk);

%% Simulate

% insert our initial condition as the 1st element in traj
% note that traj will have T+1 columns.
x_traj = zeros(size(x0,1), T+1);
x_traj(:,1) = x0;

% Forward simulation of a discretized system is just a rollout.
% Indexing is a bit off here, we're including the initial state as the
% start of x_traj, so it's really 0:T+1 rather than 1:T.
for k=1:T
    %%%%% DYNAMICS
    x_kp1 = A*x_traj(:,k) + B*u_kT(k);
    % save the result
    x_traj(:,k+1) = x_kp1;
end

%% Plot

% We want to see all three states
figure;
hold on;
sgtitle('Minimum energy control sequence test, open loop')
t = [0:1:T];

for i=1:n
    subplot(n,1,i)
    plot(t, x_traj(i,:), 'r');
    % a neat trick to obtain the string "x1", "x2", etc
    ylabel(strcat("x", num2str(i)));
end
xlabel('time (sec)')

%% Verify

disp('x_traj final state:')
x_traj(:,end)
disp('target final state:')
xf

% Energy for this input sequence:
norm(u_Tk)^2












