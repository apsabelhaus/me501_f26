% pendulum_sim.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('Controller design and simulation example for a single inverted pendulum.')

%% Setup: constants and initial conditions

tmax = 5;
dt = 0.0001;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

m = 5; % kg
g = 9.81;
c = 10; % damping, due to friction or air resistance or something similar
ell = 0.5; % length of pendulum
b = 0.1; % motor torque constant. Chosen arbitrarily for an example.
tol = 10^4; % numerical tolerance for instability: if f(x) is greater than this, in any element, assume our simulation has encountered a big issue and stop integrating

% we'll pass a struct of constants to a separate function that calculates
% the \dot x = f(x, u)
constants.m = m;
constants.g = g;
constants.c = c;
constants.b = b;
constants.ell = ell;
constants.tol = tol;

%%%%% Initial conditions:

theta_0 = pi/6; % mass two angle from mass one
dottheta_0 = 0; % moving tangentially / rotation. pi/24?
x0 = [theta_0; dottheta_0];

%% Controller Design:

% Unactuated, no input (also called "free response")
u = 0;

%% Simulate to obtain trajectory

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_traj = zeros(size(x0,1), n+1);
x_traj(:,1) = x0;

% actual simulation! Use Forward Euler for now.
for t=1:n
    %%%%% DYNAMICS
    % Undamped:
    bolddotx_t = f_pendulum(x_traj(:,t), u, constants);
    % Forward Euler
    x_traj(:,t+1) = x_traj(:,t) + dt*bolddotx_t;
    % basis vecs and force are only for plotting/visualization.
end

%% Plot

% Let's calculate the position of the mass corresponding to each theta.
% That will help with our plotting.

% "r" for me means "position of something in space." I reserve "x" for
% "system states."
r_traj = zeros(2, n+1);

for t=1:n+1
    r_traj(:,t) = cyl_pol_coords(ell, x_traj(1,t)); % this is "(r, theta)"
end

% We will skip by a certain number of steps in order for the visualization
% to go faster.
speedup = 100;
n_speedup = n/speedup;

% set up the figure
figure; hold on;
grid on;
title('Pendulum Simulation')
xlabel('x1 (meters)');
ylabel('x2 (meters)');
% set us up to see the limits of the particle's motion
% let's choose a square plot. Find the largest in each of the cartesian
% directions and use that.
% here's a trick: max/min over all the particle's positions
maxcoord = max([max(r_traj), -min(r_traj)]);
limits = [-maxcoord, maxcoord];
xlim(limits);
ylim(limits);
% Pendulum mass dot size
massdotsize = 50;

% plot some lines to visualize the x1, x2 axes
line(2*xlim, [0,0]);
line([0,0], 2*ylim);
% look at the plot from a nice angle
% for a view of the 2D plane E1-E2,
view(0, 90);

% Initialize the first point
r_handle = scatter(r_traj(1,1), r_traj(2,1), massdotsize, "r", "filled");

% force MATLAB to render the whole figure before moving forward - computers
% are funny
drawnow;

% plot over time - now, per speedup.
for j=2:n_speedup
    % take the frames only at a certain interval.
    i=j*speedup;
    % this removes the lines and points from our plot
    delete(r_handle);
    drawnow;
    % replot now at the i-th timestep in the trajectory
    r_handle = scatter(r_traj(1,i), r_traj(2,i), massdotsize, "r", "filled");
    drawnow;
end

% for visualization, plot the whole trajectory at once.
figure;
hold on;
grid on;
title('Trajectory - all timepoints')
xlabel('x1');
ylabel('x2');
xlim(limits);
ylim(limits);
line(2*xlim, [0,0]);
line([0,0], 2*ylim);
% for a view of the 2D plane E1-E2,
view(0, 90);
scatter(r_traj(1,:), r_traj(2,:), massdotsize, "r", "filled");
legend("Pendulum Mass");















