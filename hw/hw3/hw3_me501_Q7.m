%% hw3_me501_Q7.m
% (C) Andrew Sabelhaus, 2026

close all;
clc;

disp('Checking stability of exact discretizations.')

%% Continuous time matrices for consideration

A1 = FILL_THIS_IN

A2 = FILL_THIS_IN

g = 10;
m = 1;
c = 1;

A3 = FILL_THIS_IN

%% Checking continuous time eigenvalues

[V1, D1] = FILL_THIS_IN
[V2, D2] = FILL_THIS_IN
[V3, D3] = FILL_THIS_IN

% Pull out the diagonal elements for ease of viewing:
disp('Diagonal elements of D for A1, A2, then A3:')
diag(D1)
diag(D2)
diag(D3)

% To calculate if we get enough eigenvectors to pass the multiplicity test,
% an easy way is to check the rank of the V that matlab produces. This
% tells us how many linearly independent columns of the matrix there are.
disp('Rank of V for A3:')
rank(V3)

%% Exact discretization and eigenvalues

% We hypothesize that the sampling time shouldn't matter under the exact
% discretization - try it out and confirm for yourself!

dt = 0.01;

% Our claim is that x_{k+1} = A_d x_k, where A_d = e^(A*deltaT)
disp('Exact discretization of A1, A2, and A3:')
A1d = FILL_THIS_IN
A2d = FILL_THIS_IN
A3d = FILL_THIS_IN

% Check the eigenvalues of the discretized dynamics
[V1d, D1d] = FILL_THIS_IN
[V2d, D2d] = FILL_THIS_IN
[V3d, D3d] = FILL_THIS_IN

% Pull out the diagonal elements for ease of viewing:
disp('Diagonal elements of D for A1d, A2d, then A3d:')
lambda1d = diag(D1d)
lambda2d = diag(D2d)
lambda3d = diag(D3d)

disp('Magnitude (modulus) of the discrete time eigenvalues:')
abs(lambda1d)
abs(lambda2d)
abs(lambda3d)

disp('Rank of V for A3d:')
rank(V3d)







