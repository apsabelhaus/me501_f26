%% hw3_me501_Q6.m
% (C) Andrew Sabelhaus, 2026

close all;
clc;

disp('Numerically finding equilibrium points for the bendy spring-pendulum.')

%% Numerically find the roots of f(x)=0

% We will use two different guesses:
q_guess1 = [0.5; 0.5];
q_guess2 = [-0.5; 0.5];

% find the "closest" equilibrium point to each guess. print them.
q_eq1 = fsolve(@soln_bendyspringeq_2d, q_guess1)
q_eq2 = fsolve(@soln_bendyspringeq_2d, q_guess2)

% as position vectors... cartesian
r_eq1 = [0; q_eq1];
r_eq2 = [0; q_eq2];

%% Plot
    
% set up the figure
figure; hold on;
grid on;

title('ME501 - equilibria for the bendy spring pendulum')

xlabel('E1');
ylabel('E2');
zlabel('E3');
% find plot limits based on the particle position vectors we want to
% plot
r_all = [r_eq1, r_eq2];
maxcoord = max([max(r_all), -min(r_all)]);
limits = [-maxcoord, maxcoord];
xlim(limits);
ylim(limits);
zlim(limits);
% plot some lines to visualize the E1, E2, E3 axes
line(2*xlim, [0,0], [0,0]);
line([0,0], 2*ylim, [0,0]);
line([0,0], [0,0], 2*zlim);
% look at the plot from a nice angle
view(-37, 40);

scatter3(r_eq1(1,:), r_eq1(2,:), r_eq1(3,:), 30, "r", "filled");
scatter3(r_eq2(1,:), r_eq2(2,:), r_eq2(3,:), 30, "b", "filled");
legend("","","","Equilibrium found for guess 1", "Equilibrium found for guess 2");
    















