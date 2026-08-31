function [bolddotx] = f_pendulum(x, u, constants)
%f_pendulum The dynamics function f(x), in state-space form, for an
%inverted pendulum with damping and no spring.
%
%   This function implements the right-hand-side of the ordinary
%   differential equation \dot x = f(x)
%
%   Inputs:
%       x == state at time t, [theta; dottheta]
%       u == control input at time t, presumably the motor torque. Scalar.
%       constants == a MATLAB struct containing the constants to plug into
%       the dynamics
%
%   Outputs:
%       bolddotx = time derivative, \dot \mathbf{x}

% pick out the constants
m = constants.m; % we're approximating away the mass term in our equations below, so m is unused.
g = constants.g;
c = constants.c;
b = constants.b;
ell = constants.ell;

% pick out variables from x so we don't get confused (just for convenience)
theta = x(1);
dottheta = x(2);

% In state-space form:
bolddotx = [dottheta;
            (g/ell)*sin(theta) - c*dottheta + b*u];

% prevent numerical issues
bolddotx = numerical_check(bolddotx, constants.tol);

end