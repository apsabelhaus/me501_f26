function xy = cyl_pol_coords(r, theta)
%cyl_pol_coords The position vector \mathbf{r} for the cylindrical-polar coordinate
%system
%
%   This function implements the bold "r" position vector in the
%   cylindrical-polar coordinate system.
%
%   Inputs:
%       r == radius
%       theta = angle
%
%   Outputs:
%       xy = the position vector, now in Cartesian coordinates, in
%       \mathbb{R}^2

% bold_r = r*e_r 
xy = r*[cos(theta); sin(theta)];

end