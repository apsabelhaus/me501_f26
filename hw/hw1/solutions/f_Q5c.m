function [bolddotx] = f_Q5c(x, constants)
%f_Q5b The dynamics function f(x), in state-space form, for a spring-mass
%system with simple friction
%
%   This function implements the right-hand-side of the ordinary
%   differential equation given in ME 501 HW1 Q5c, representing a
%   point mass moving horizontally with a spring attached and fricton
%   between the point mass and its surface
%
%   Inputs:
%       x == state at time t
%       constants == a MATLAB struct containing the constants to plug into
%       the dynamics (here, that's mass, spring constant, spring neutral
%       length, friction coefficient).

g = 9.81;

% Calculate the direction of friction using the "sgn" function.
fric_dir = 0;
if x(2) > 0
    fric_dir = 1;
elseif x(2) < 0
    fric_dir = -1;
end

% Add a frictional force depending on the direction of velocity.
bolddotx = [x(2); (1/constants.m)*(-constants.k*(x(1) - constants.bar_x1)-fric_dir*constants.mu_d*constants.m*g)];

end