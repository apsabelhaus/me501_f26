function [rhs] = bendyspringeq_2d(q)
%bendyspringeq The equilibrium condition for the bendy spring pendulum,
%partial U / partial q, for x = 0
%   
%   inputs: 
%       q = 2x1 vector of the y, z coordinates of the point mass
%
%   outputs: rhs = the right hand side of our equilibrium condition. At
%   equilibrium, rhs = 0, so we want to find bendyspringeq(q_eq) = 0

% hard-coded for now
Rbar = 1.0;
ks = 1000;
kb2 = 300;
m = 70;
g = 9.81;

y = q(1);
z = q(2);

rhs = FILL_THIS_IN;

% Aside: this is how I calculated the z^{eq} values.
% disp(-m*g/ks - Rbar)
% disp(-m*g/ks + Rbar)


end