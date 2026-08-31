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
c = 2; % damping, due to friction or air resistance or something similar
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

theta_0 = 2*pi/3; % mass angle from vertical. Positive is clockwise.
dottheta_0 = 0; % moving tangentially / rotation. tryL pi/24?
x0 = [theta_0; dottheta_0];

%% Controller Design:

%%%% Unactuated, no input (also called "free response" or "unforced")
u = 0; % we'll overwrite this below if we choose to turn on feedback.

%%%% State Feedback using pole placement:

% Desired location of poles
lambda1 = -1;
lambda2 = -2;

% %%% ALTERNATIVE LOCATIONS OF POLES - LET'S EXPERIMENT!
% sigma = -2; % the real part, determines the exponential decay rate
% omega = 10;  % the imaginary part, determines oscillations
% lambda1 = sigma + omega*i;
% lambda2 = sigma - omega*i;
% %%%

% The linearized system dynamics are:
A = [0,         1;
     (g/ell),   -c];
B = [0; b];

% The controllability matrix for this system is
Q = [B, A*B];
% The change-of-basis matrix is
W = [1, c;
     0, 1];

% Get the coefficients for the desired characteristic polynomial, given our
% desired locations of poles. For a 2x2 system, the characteristic
% polynomial is of the form:
% det(sI - A) = s^2 + a1*s + s2
% Our desired characteristic polynomial, with the desired eigenvalues, is:
% p_d(s) = (s - \lambda_1)*(s - \lambda_2) = s^2 - (\lambda_1 + \lambda_2)s + \lambda_1*\lambda_2
% Therefore, desired ("d") coefficients are:
a1_d = -(lambda1 + lambda2);
a2_d = lambda1*lambda2;

% the coefficients for our unforced system are:
a1 = -(A(1,1) + A(2,2));             % this is trace(A)
a2 = A(1,1)*A(2,2) - A(1,2)*A(2,1);  % this is det(A);

% Ackermann's formula (via the Astrom and Murray book), also called the 
% Bass and Gura formula in the Friedland book,
coeff_diff = [(a1_d-a1); (a2_d-a2)];
K = inv((Q*W)')*coeff_diff;


%% Simulate to obtain trajectory

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_traj = zeros(size(x0,1), n+1);
x_traj(:,1) = x0;

% It will be interesting and useful for us to see how the applied control
% input (motor torque) changes over time, and with different choices of
% controller. So let's save the values of u(t) as well.
u_traj = zeros(1, n);

% actual simulation! Use Forward Euler for now.
for t=1:n
    %%%%% Calculate Control Input at this timestep
    % State Feedback
    u = -K'*x_traj(:,t);
    % record the control input we're about to apply - this is for plotting
    % later.
    u_traj(t) = u;
    %%%%% DYNAMICS
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
speedup = 50;
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
% put a fudge factor on the plot so we can see better
maxcoord = maxcoord *1.2;
limits = [-maxcoord, maxcoord];
xlim(limits);
ylim(limits);
axis square;
% Pendulum mass dot size
massdotsize = 200;

% plot some lines to visualize the x1, x2 axes
line(2*xlim, [0,0]);
line([0,0], 2*ylim);
% look at the plot from a nice angle
% for a view of the 2D plane E1-E2,
view(0, 90);

% Initialize the first point
r_handle = scatter(r_traj(1,1), r_traj(2,1), massdotsize, "r", "filled");
% draw a line indicating a "massless" rigid rod connecting the pendulum tip
% to the origin
rod_handle = line([0; r_traj(1,1)], [0; r_traj(2,1)], "Color", 'b');

% force MATLAB to render the whole figure before moving forward - computers
% are funny
drawnow;

% plot over time - now, per speedup.
for j=2:n_speedup
    % take the frames only at a certain interval.
    i=j*speedup;
    % Update the location of the mass
    set(r_handle, 'XData', r_traj(1,i), 'YData', r_traj(2,i));
    % Update the "massless rod" line
    set(rod_handle, 'XData', [0; r_traj(1,i)], 'YData', [0; r_traj(2,i)]);
    drawnow;
end

% for visualization, plot the whole trajectory at once.
figure;
hold on;
grid on;
title('Trajectory in space - all timepoints')
xlabel('x1');
ylabel('x2');
xlim(limits);
ylim(limits);
axis square;
line(2*xlim, [0,0]);
line([0,0], 2*ylim);
% for a view of the 2D plane,
view(0, 90);
scatter(r_traj(1,:), r_traj(2,:), massdotsize, "r", "filled");
legend(["","","Pendulum Mass"]);

% We can also plot the three time series trajectories of theta, dottheta,
% and u
figure;
hold on;
t = [0:dt:tmax];
subplot(3,1,1);
sgtitle('Trajectories of states and input');
% theta
plot(t, x_traj(1,:), 'r');
xlabel('time (sec)');
ylabel('theta (rad)');
% velocity
subplot(3,1,2);
plot(t, x_traj(2,:), 'b');
xlabel('time (sec)');
ylabel('dtheta/dt (rad/sec)');
% control input
subplot(3,1,3);
plot(t(1:end-1), u_traj, 'k'); % one fewer control input than timestep, there's no input at the end!
xlabel('time (sec)');
ylabel('control input (no units)');
















