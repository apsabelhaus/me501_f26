%% hw2_me501_Q4.m
% (C) Andrew Sabelhaus, 2026

clear all;
close all;
clc;

disp('Practice with eigenvector and eigenvalue calculations.')

%% Setup: our three matrices

B = [1 1 0;
     0 2 -1;
     0 1 2];

C = [2 1 0;
     1 2 0;
     0 0 3];

G = [2 1 0;
    -1 4 0;
     0 0 1];

%% Calcualion of eigenvectors and eigenvalues:

[VB, DB] = eig(B)
[VC, DC] = eig(C)
[VG, DG] = eig(G)
