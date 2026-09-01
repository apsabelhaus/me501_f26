% hw1_me501_Q4b.m
% (C) Andrew Sabelhaus, 2026

% Your name:
%

clear all;
close all;
clc;

%% Q4(b): Numerical approximation to an integral

% Using the Forward Euler method, integrate the definite integral \int_{2}^3 x^5 dx
% with a subinterval of \Delta x = ?

% range of the integral and discretization
x0 = 2;
xN = 3;

deltaX = 0.001;

% Initialize a counter variable to store our summation
x_integ = 0;

% step through until we reach our goal
nMax = (xN-x0)/deltaX; % generally this needs to be a whole number, you should check and confirm.
for n = 1:nMax
    % Our integration equation is
    % x_integrated = \sum_{n=0}^nMax deltaX * stuff_inside_integral(x0+deltaX*n)
    x_integ = x_integ + deltaX * (x0 + deltaX*n)^4;
end

disp(strcat("Value of integral with a discretization of ", num2str(deltaX), " from ", num2str(x0), ...
    " to ", num2str(xN), " is ", num2str(x_integ)))