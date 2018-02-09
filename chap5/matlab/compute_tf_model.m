function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% add stuff here
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

Va_trim = (P.u0^2+P.v0^2+P.w0^2)^0.5;

a_phi1 = -(1/2)*P.rho*P.Va0^2*P.S_wing*P.b*C_p_p * P.b/(2*P.Va0);
a_phi2 =  (1/2)*P.rho*P.Va0^2*P.S_wing*P.b*C_p_delta_a;

a_beta1 = - (P.rho*P.Va0*P.S_wing/(2*P.mass)) * P.C_Y_beta ;
a_beta2 =   (P.rho*P.Va0*P.S_wing/(2*P.mass)) * P.C_Y_delta_r;

a_theta1 = - (P.rho*P.Va0^2*P.c*P.S_wing/(2*P.Jy)) * P.C_m_q  * (P.c / (2*P.Va0));
a_theta2 = - (P.rho*P.Va0^2*P.c*P.S_wing/(2*P.Jy)) * P.C_m_alpha;
a_theta3 =   (P.rho*P.Va0^2*P.c*P.S_wing/(2*P.Jy)) * P.C_m_delta_e;

alpha_trim = atan(P.w0/P.u0);
a_V1 = (P.rho*Va_trim*P.S_wing/P.mass)*(P.C_D_0+P.C_D_alpha*alpha_trim+P.C_D_delta_e*u_trim(1))...
        +(P.rho*P.S_prop/P.mass)*P.C_prop*Va_trim;
a_V2 = (P.rho*P.S_prop/P.mass)*P.C_prop*P.k_motor^2*u_trim(2);
a_V3 = P.gravity*cos(P.theta0-P.psi0);

theta_trim = P.theta0;

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

