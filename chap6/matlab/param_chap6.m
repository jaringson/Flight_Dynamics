P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%% physical parameters of airframe
P.mass = 13.5;
% P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
%% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

%% Gamma
P.gamma = P.Jx*P.Jz-P.Jxz^2;
P.gamma_1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/P.gamma;
P.gamma_2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.gamma;
P.gamma_3 = P.Jz/P.gamma;
P.gamma_4 = P.Jxz/P.gamma;
P.gamma_5 = (P.Jz-P.Jx)/P.Jy;
P.gamma_6 = P.Jxz/P.Jy;
P.gamma_7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.gamma;
P.gamma_8 = P.Jx/P.gamma;

%% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


%% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 25;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

%% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

%% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

%% Values for Transfer Functions and PID gains

% C_p_0 = P.gamma_3*P.C_ell_0+P.gamma_4*P.C_n_0;
% C_p_beta = P.gamma_3*P.C_ell_beta+P.gamma_4*P.C_n_beta;
C_p_p = P.gamma_3*P.C_ell_p+P.gamma_4*P.C_n_p;
% C_p_r = P.gamma_3*P.C_ell_r+P.gamma_4*P.C_n_r;
C_p_delta_a = P.gamma_3*P.C_ell_delta_a+P.gamma_4*P.C_n_delta_a;
% C_p_delta_r = P.gamma_3*P.C_ell_delta_r+P.gamma_4*P.C_ell_delta_r;
% C_r_0 = P.gamma_4*P.C_ell_0+P.gamma_8*P.C_n_0;
% C_r_beta = P.gamma_4*P.C_ell_beta+P.gamma_8*P.C_n_beta;
% C_r_p = P.gamma_4*P.C_ell_p+P.gamma_8*P.C_n_p;
% C_r_r = P.gamma_4*P.C_ell_0r+P.gamma_8*P.C_n_r;
% C_r_delta_a = P.gamma_4*P.C_ell_delta_a+P.gamma_8*P.C_n_delta_a;
% C_r_delta_r = P.gamma_4*P.C_ell_delta_r+P.gamma_8*P.C_n_delta_r;


P.Va_trim = (P.u0^2+P.v0^2+P.w0^2)^0.5;

P.a_phi1 = -(1/2)*P.rho*P.Va0^2*P.S_wing*P.b*C_p_p * P.b/(2*P.Va0);
P.a_phi2 =  (1/2)*P.rho*P.Va0^2*P.S_wing*P.b*C_p_delta_a;

P.a_beta1 = - (P.rho*P.Va0*P.S_wing/(2*P.mass)) * P.C_Y_beta ;
P.a_beta2 =   (P.rho*P.Va0*P.S_wing/(2*P.mass)) * P.C_Y_delta_r;

P.a_theta1 = - (P.rho*P.Va0^2*P.c*P.S_wing/(2*P.Jy)) * P.C_m_q  * (P.c / (2*P.Va0));
P.a_theta2 = - (P.rho*P.Va0^2*P.c*P.S_wing/(2*P.Jy)) * P.C_m_alpha;
P.a_theta3 =   (P.rho*P.Va0^2*P.c*P.S_wing/(2*P.Jy)) * P.C_m_delta_e;

P.alpha_trim = atan(P.w0/P.u0);
P.a_V1 = (P.rho*P.Va_trim*P.S_wing/P.mass)*(P.C_D_0+P.C_D_alpha*P.alpha_trim+P.C_D_delta_e*u_trim(1))...
        +(P.rho*P.S_prop/P.mass)*P.C_prop*P.Va_trim;
P.a_V2 = (P.rho*P.S_prop/P.mass)*P.C_prop*P.k_motor^2*u_trim(4);
P.a_V3 = P.gravity*cos(P.theta0-P.psi0);

% P.theta_trim = P.theta0;


%% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

%% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);


%% PID Values
% kd_phi and kp_phi parameters
e_phi_max = 15*pi/180;
delta_a_max = 45*pi/180;
omega_n_phi = sqrt(abs(P.a_phi2)*delta_a_max/e_phi_max);
zeta_phi = 1.0; % tune this parameter
P.kp_phi = delta_a_max/e_phi_max*sign(P.a_phi2);
P.kd_phi = (2*zeta_phi*omega_n_phi-P.a_phi1)/(P.a_phi2);

% kp_chi and ki_chi parameters
W_chi = 30; % design parameter usually bigger than 5
omega_n_chi = 1/W_chi*omega_n_phi;
zeta_chi = 1.0; % tune this parameters
P.kp_chi = 2*zeta_chi*omega_n_chi*P.Va_trim/P.gravity;
P.ki_chi = omega_n_chi^2*P.Va_trim/P.gravity;

% kp_theta and kd_theta parameters
delta_e_max = 45*pi/180;
e_theta_max = 10*pi/180;
omega_n_theta = sqrt(P.a_theta2 + delta_e_max/e_theta_max*abs(P.a_theta3));
zeta_theta = 0.5; % tune this parameter
P.kp_theta = delta_e_max/e_theta_max*sign(P.a_theta3);
P.kd_theta = (2*zeta_theta*omega_n_theta-P.a_theta1)/P.a_theta3;
K_theta_DC = P.kp_theta*P.a_theta3/(P.a_theta2 + P.kp_theta*P.a_theta3);

% ki_v and kp_v parameters
omega_n_v = 10; % tune this parameter
zeta_v = 1.0; % tune this parameter
P.ki_v = omega_n_v^2/P.a_V2;
P.kp_v = (2*zeta_v*omega_n_v-P.a_V1)/P.a_V2;

% kp_v2 and ki_v2
W_v2 = 10; % tune this parameter
omega_n_v2 = 1/W_v2*omega_n_theta;
zeta_v2 = 0.7; % tune this parameter
P.ki_v2 = -omega_n_v2^2/(K_theta_DC*P.gravity);
P.kp_v2 = (P.a_V1-2*zeta_v2*omega_n_v2)/(K_theta_DC*P.gravity);

% kp_h and ki_h parameters
W_h = 50; % tune this parameter
omega_n_h = 1/W_h*omega_n_theta;
zeta_h = 1.0; % tune this parameter
P.ki_h = omega_n_h^2/(K_theta_DC*P.Va_trim);
P.kp_h = (2*zeta_h*omega_n_h)/(K_theta_DC*P.Va_trim);

%% Altitude Values
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 2;

