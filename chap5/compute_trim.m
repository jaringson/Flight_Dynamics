function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit


% add stuff here
dx0 =[0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; (Va/R)*cos(gamma); 0; 0; 0];
idx =[3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
ix = [];
u0 = [0; 0; 0; 1];
iu = [];
y0 = [Va; 0; 0];
iy = [1,3];

% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
norm(dx_trim(3:end)-dx0(3:end))

