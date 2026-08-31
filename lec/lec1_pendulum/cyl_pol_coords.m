function xy = cyl_pol_coords(r, theta)
%cyl_pol_coords The position vector \mathbf{r} for the cylindrical-polar coordinate
%system, rotated so that theta=0 is along the vertical axis (maintaining
%that positive theta is counterclockswise
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

% polar coordinates
xy = r*[cos(theta); sin(theta)];

% rotation matrix!
rot = pi/2;
rotmat = [cos(rot), -sin(rot);
          sin(rot),  cos(rot)];

xy = rotmat*xy;

end