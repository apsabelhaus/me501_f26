%% hw3_me501_Q7.m
% (C) Andrew Sabelhaus, 2026

close all;
clc;

disp('Checking stability of exact discretizations.')

%% Continuous time matrices for consideration

A1 = [0 1; -20 0];

A2 = [0 1; -20 -9];

g = 10;
m = 1;
c = 1;

A3 = [0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1;
      0 0 -g -c/m 0 0;
      0 0 0 0 -c/m 0;
      0 0 0 0 0 0];

%% Checking continuous time eigenvalues

[V1, D1] = eig(A1);
[V2, D2] = eig(A2);
[V3, D3] = eig(A3);

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

% Our claim is that x(k) = A_d^k x(0), where A_d = e^(A*deltaT)
disp('Exact discretization of A1, A2, and A3:')
A1d = expm(A1*dt)
A2d = expm(A2*dt)
A3d = expm(A3*dt)

% Check the eigenvalues of the discretized dynamics
[V1d, D1d] = eig(A1d);
[V2d, D2d] = eig(A2d);
[V3d, D3d] = eig(A3d);

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







