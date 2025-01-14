function [x, y, z, vx, vy, vz] = cal_hill(x0, y0, z0, vx0, vy0, vz0, n, t)
% CAL_HILL calculates the position and velocity using the Hill's equations
%
% Inputs:
%   x0, y0, z0  - Initial position coordinates (km)
%   vx0, vy0, vz0 - Initial velocity components (km/s)
%   n           - Mean motion of the orbit (rad/s)
%   t           - Time elapsed (s)
%
% Outputs:
%   x, y, z     - Updated position coordinates (km)
%   vx, vy, vz  - Updated velocity components (km/s)
% Equations taken from Fundamentals of astrodynamics  and applications 
% Position equations
x = (4 - 3*cos(n*t))*x0 + (1/n)*sin(n*t)*vx0 + (2/n)*(1 - cos(n*t))*vy0;
y = 6*(sin(n*t) - n*t)*x0 + y0 - (2/n)*(1 - cos(n*t))*vx0 + (1/n)*((4*sin(n*t) - 3*n*t)*vy0);
z = cos(n*t)*z0 + (1/n)*sin(n*t)*vz0;

% Velocity equations
vx = 3*n*sin(n*t)*x0 + cos(n*t)*vx0 + 2*sin(n*t)*vy0;
vy = -6*n*(1 - cos(n*t))*x0 - 2*sin(n*t)*vx0 + (4*cos(n*t) - 3)*vy0;
vz = -n*sin(n*t)*z0 + cos(n*t)*vz0;

end