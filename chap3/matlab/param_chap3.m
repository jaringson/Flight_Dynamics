P.gravity = 9.81; % m/s^2
   
%physical parameters of airframe
P.mass = 13.5; % kg
P.Jx   = 0.8244; % kg m^2
P.Jy   = 1.135; % kg m^2
P.Jz   = 1.759; % kg m^2
P.Jxz  = 0.1204; % kg m^2

% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

P.gamma = P.Jx*P.Jz-P.Jxz^2;
P.gamma_1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/P.gamma;
P.gamma_2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.gamma;
P.gamma_3 = P.Jz/P.gamma;
P.gamma_4 = P.Jxz/P.gamma;
P.gamma_5 = (P.Jz-P.Jx)/P.Jy;
P.gamma_6 = P.Jxz/P.Jy;
P.gamma_7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.gamma;
P.gamma_8 = P.Jx/P.gamma;
