% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
    %% Simple Estimator
    
    persistent lpf_gyro_x
    persistent lpf_gyro_y
    persistent lpf_gyro_z
    persistent lpf_static
    persistent lpf_diff
    persistent lpf_accel_x
    persistent lpf_accel_y
    persistent lpf_accel_z
    persistent lpf_gps_n
    persistent lpf_gps_e
    persistent lpf_gps_Vg
    persistent lpf_gps_course
    persistent alpha
    persistent alpha1
    persistent alphatheta
    
    
    lpf_a = 50;
    lpf_a1 = 10;
    lpf_th = 10;
    if t==0

        alpha = exp(-lpf_a*P.Ts);
        alpha1 = exp(-lpf_a1*P.Ts);
        alphatheta = exp(-lpf_th*P.Ts);
        lpf_gyro_x = 0;
        lpf_gyro_y = 0;
        lpf_gyro_z = 0;
        lpf_static = P.rho*P.gravity*(-P.pd0);
        lpf_diff = 0.5*P.rho*P.gravity*P.Va0;
        lpf_accel_x = 0;
        lpf_accel_y = 0;
        lpf_accel_z = 0;
        lpf_gps_n = P.pn0;
        lpf_gps_e = P.pe0;
        lpf_gps_course = P.psi0;
        lpf_gps_Vg = P.Va0;       
        
    end
    
    lpf_gyro_x = alpha * lpf_gyro_x + (1-alpha)*y_gyro_x;
    lpf_gyro_y = alpha * lpf_gyro_y + (1-alpha)*y_gyro_y;
    lpf_gyro_z = alpha * lpf_gyro_z + (1-alpha)*y_gyro_z;
    lpf_static = alpha * lpf_static + (1-alpha)*y_static_pres;
    lpf_diff = alpha * lpf_diff + (1-alpha)*y_diff_pres;
    lpf_accel_x = alpha1 * lpf_accel_x + (1-alpha1)*y_accel_x;
    lpf_accel_y = alpha1 * lpf_accel_y + (1-alpha1)*y_accel_y;
    lpf_accel_z = alpha1 * lpf_accel_z + (1-alpha1)*y_accel_z;
    lpf_gps_n = alpha * lpf_gps_n + (1-alpha)*y_gps_n;
    lpf_gps_e = alpha * lpf_gps_e + (1-alpha)*y_gps_e;
    lpf_gps_course = alphatheta * lpf_gps_course + (1-alphatheta)*y_gps_course;
    lpf_gps_Vg = alpha * lpf_gps_Vg + (1-alpha)*y_gps_Vg;
    
    
    phat = lpf_gyro_x;
    qhat = lpf_gyro_y;
    rhat = lpf_gyro_z;
    
    hhat = lpf_static/(P.rho*P.gravity);
    
    Vahat = ((2/P.rho)*lpf_diff)^0.5;
    
    phihat = atan(lpf_accel_y/lpf_accel_z);
    thetahat = asin(lpf_accel_x/P.gravity);
    
    
    pnhat = lpf_gps_n;
    pehat = lpf_gps_e;
    chihat = lpf_gps_course;
    
    Vghat = lpf_gps_Vg;
    
    wnhat = 0;
    wehat = 0;
    psihat = lpf_gps_course;
    
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
    
    f = 0;
    
end
