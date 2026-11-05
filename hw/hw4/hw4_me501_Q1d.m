%% hw4_me501_Q1d.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('Simulating input-output-equivalent state space systems.')

%% Setup

% We're going to use forward euler for now, then discuss exact
% discretization later in the problem set.

dt = 0.001;
tmax = 10;
n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

x0 = [0; 0; 0];

% Please put the original A,B,C here:
A1 = FILL_THIS_IN;

B1 = FILL_THIS_IN;

C1 = FILL_THIS_IN;

% Please put the A,B,C in controllable canonical form here:
A2 = FILL_THIS_IN;

B2 = FILL_THIS_IN;

C2 = FILL_THIS_IN;


%% Simulate

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_trajOrig = zeros(size(x0,1), n+1);
x_trajOrig(:,1) = x0;
y_trajOrig = zeros(1, n+1);
y_trajOrig(1) = C1*x0;

% and a trajectory of the system in controllable canonical realization
x_trajCC = zeros(size(x0,1), n+1);
x_trajCC(:,1) = x0;
y_trajCC = zeros(1, n+1);
y_trajCC(1) = C2*x0;

% Let's do a step control input. u=1 for all time.
u=1;

% Forward euler for the original dynamics
for t=1:n
    %%%%% DYNAMICS
    bolddotx_t = A1*x_trajOrig(:,t) + B1*u;
    % Forward Euler
    x_trajOrig(:,t+1) = x_trajOrig(:,t) + dt*bolddotx_t;
    % save the output too
    y_trajOrig(t+1) = C1*x_trajOrig(:,t+1);
end

% Forward euler for the controllable canonical form realization
for t=1:n
    %%%%% DYNAMICS
    bolddotx_t = FILL_THIS_IN;
    % Forward Euler
    x_trajCC(:,t+1) = FILL_THIS_IN;
    % save the output too
    y_trajCC(t+1) = FILL_THIS_IN;
end

%% Plot

% We want to see all three states, AND the outputs!
figure;
hold on;
t = [0:dt:tmax];

% Five rows: x1, x2, x3, u, y. Two columns: original, and controllable
% canonical.
subplot(5,2,1);
sgtitle('Trajectories of states, inputs, and output: original vs. controllable canonical');
% x1
plot(t, x_trajOrig(1,:), 'b');
ylabel('x1');
% x2
subplot(5,2,3);
plot(t, x_trajOrig(2,:), 'b');
ylabel('x2');
% x3
subplot(5,2,5);
plot(t, x_trajOrig(3,:), 'b');
ylabel('x3');
% u
subplot(5,2,7);
plot(t, u*ones(size(t)), 'g');
ylabel('u');
% y
subplot(5,2,9)
plot(t, y_trajOrig, 'r');
ylabel('y')
xlabel('time (sec)')

% In controllable canonical form:
% x1
subplot(5,2,2);
plot(t, x_trajCC(1,:), 'b');
ylabel('x1');
% x2
subplot(5,2,4);
plot(t, x_trajCC(2,:), 'b');
ylabel('x2');
% x3
subplot(5,2,6);
plot(t, x_trajCC(3,:), 'b');
ylabel('x3');
% u
subplot(5,2,8);
plot(t, u*ones(size(t)), 'g');
ylabel('u');
% y
subplot(5,2,10)
plot(t, y_trajCC, 'r');
ylabel('y')
xlabel('time (sec)')












