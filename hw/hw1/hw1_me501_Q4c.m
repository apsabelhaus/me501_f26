% hw1_me501_Q4c.m
% (C) Andrew Sabelhaus, 2024

clear all;
close all;
clc;

%% Q4(c): Numerical approximations to an integral, part 2

% Using the Forward Euler method, integrate the definite integral \int_{0}^2 e^(x^2) dx
% with a subinterval of \Delta x = 0.01

% range of the integral and discretization
x0 = 0;
xN = 2;
deltaX = 0.01;

% Initialize a counter variable to store our summation
x_integ = 0;

% step through until we reach our goal
nMax = (xN-x0)/deltaX; % generally this needs to be a whole number, you should check and confirm.
for n = 1:nMax
    % Our integration equation is
    % x_integ = \sum_{n=0}^nMax deltaX * stuff_inside_integral(x0+deltaX*n)
    % SOLUTION:
    x_integ = FILL_THIS_IN;
end

disp(strcat("Value of integral with a discretization of ", num2str(deltaX), " from ", num2str(x0), ...
    " to ", num2str(xN), " is ", num2str(x_integ)))

% Compare to the results in https://tutorial.math.lamar.edu/classes/calcii/ApproximatingDefIntegrals.aspx