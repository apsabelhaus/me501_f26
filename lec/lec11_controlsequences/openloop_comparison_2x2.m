% openloop_comparison_2x2.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

% This displays decimal points past four digits. Useful for assessing
% numerical accuracy.
format long g;

disp('Calculating and comparing open-loop control sequences for a simple 2x2 linear discrete-time system.')

%% Setup: constants and initial conditions

A = [1 1;
     0 1];
B = [0;
     1];

x0 = [0; 1];

xf = [0; 0];

%% Control sequence calculations

%%%% For n=2:

Q = [B A*B];
W = Q*Q';
disp('Rank of Q and W for n=2:')
rank(Q)
rank(W)

u_10 = inv(Q)*(xf - (A^2)*x0)

%%%% for n=3:

Q3 = [B A*B A^2*B];
W3 = Q3*Q3';
disp('Rank of W3:')
rank(W3)

u_20 = Q3'*inv(W3)*(xf - (A^3)*x0)


%% Simulate both sequences

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
% for n=2:
x_traj_fast = zeros(size(x0,1), 3);
x_traj_fast(:,1) = x0;
% for n=3:
x_traj_efficient = zeros(size(x0,1), 4);
x_traj_efficient(:,1) = x0;

% actual simulation. Dynamics are already in discrete time.

% for purposes of indexing, it's useful to reverse the control input
% vectors. Better to do it once, now, rather than try to remember indexing
% later in the simulation loop.
u_01 = flip(u_10);
u_02 = flip(u_20);

% minimum time solution, n=2. REmember matlab starts indexing at 1 rather
% than zero.
for t=1:2
    x_traj_fast(:,t+1) = A*x_traj_fast(:,t) + B*u_01(t);
end

% minimum energy solution for n=3
for t=1:3
    x_traj_efficient(:,t+1) = A*x_traj_efficient(:,t) + B*u_02(t);
end

disp('Trajectories, n=2 versus n=3:')
x_traj_fast
x_traj_efficient

disp('Total energy consumption, n=2 versus n=3:')
norm(u_01)^2
norm(u_02)^2

















