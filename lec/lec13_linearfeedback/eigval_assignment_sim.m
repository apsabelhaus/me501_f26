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
x0 = [1; 1];

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

















