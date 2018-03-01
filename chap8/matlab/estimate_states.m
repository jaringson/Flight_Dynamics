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
    alpha = 0.1;
    
    persistent lpf_y_gyro_x
    persistent lpf_y_gyro_y
    persistent lpf_y_gyro_z
    persistent lpf_y_static_pres
    persistent lpf_y_dynamic_pres
    persistent lpf_y_accel_x
    persistent lpf_y_accel_y
    persistent lpf_y_accel_z
    persistent lpf_y_gps_n
    persistent lpf_y_gps_e
    persistent lpf_y_gps_h
    persistent lpf_y_gps_Vg
    persistent lpf_y_gps_course
    
    
    if t == 0
       lpf_y_gyro_x = 0;
       lpf_y_gyro_y = 0;
       lpf_y_gyro_z = 0;
       lpf_y_static_pres = 0;
       lpf_y_dynamic_pres = 0;
       lpf_y_accel_x = 0;
       lpf_y_accel_y = 0;
       lpf_y_accel_z = 0;
       lpf_y_gps_n = 0;
       lpf_y_gps_e = 0;
       lpf_y_gps_course = 0;
    end
    
    lpf_y_gyro_x = alpha * lpf_y_gyro_x + (1-alpha)*y_gyro_x;
    lpf_y_gyro_y = alpha * lpf_y_gyro_y + (1-alpha)*y_gyro_y;
    lpf_y_gyro_z = alpha * lpf_y_gyro_z + (1-alpha)*y_gyro_z;
    lpf_y_static_pres = alpha * lpf_y_static_pres + (1-alpha)*y_static_pres;
    lpf_y_dynamic_pres = alpha * lpf_y_dynamic_pres + (1-alpha)*y_diff_pres;
    lpf_y_accel_x = alpha * lpf_y_accel_x + (1-alpha)*y_accel_x;
    lpf_y_accel_y = alpha * lpf_y_accel_y + (1-alpha)*y_accel_y;
    lpf_y_accel_z = alpha * lpf_y_accel_z + (1-alpha)*y_accel_z;
    lpf_y_gps_n = alpha * lpf_y_gps_n + (1-alpha)*y_gps_n;
    lpf_y_gps_e = alpha * lpf_y_gps_e + (1-alpha)*y_gps_e;
    lpf_y_gps_course = alpha * lpf_y_gps_course + (1-alpha)*y_gps_course;
    lpf_y_gps_Vg = alpha * lpf_y_gps_Vg + (1-alpha)*y_gps_Vg;
    
    
    phat = lpf_y_gyro_x;
    qhat = lpf_y_gyro_y;
    rhat = lpf_y_gyro_z;
    
    hhat = lpf_y_static_pres/(P.rho*P.gravity);
    
    Vahat = ((2/P.rho)*lpf_y_dynamic_pres)^0.5;
    
    phihat = atan(lpf_y_accel_y/lpf_y_accel_z);
    thetahat = asin(lpf_y_accel_x/P.gravity);
    chihat = lpf_y_gps_course;
    
    pnhat = lpf_y_gps_n;
    pehat = lpf_y_gps_e;
    
    Vghat = lpf_y_gps_Vg;
    
    wnhat = 0;
    wehat = 0;
    psihat = 0;
    
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
end
