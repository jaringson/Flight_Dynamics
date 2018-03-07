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
    persistent lpf_gps_h
    persistent lpf_gps_Vg
    persistent lpf_gps_course
    persistent alpha
    persistent alpha1
    persistent alphatheta
    
    persistent att_hat
    persistent gps_hat
    
    persistent att_P
    persistent att_y_old
    
    att_Q = 10^-5*diag([1,1]);
    
    
    if t==0
        
        lpf_a = 50;
        lpf_a1 = 10;
        lpf_th = 10;
    
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
        
        att_hat = [P.phi0;P.theta0];
        att_P = diag([(15*pi/180)^2,(15*pi/180)^2]);
        
        
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
    
    if t==0
        att_y_old = [lpf_accel_x;lpf_accel_y;lpf_accel_z];
    end
    
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
    
    
    N = 10;
    for i=0:N
        att_f = [phat + qhat*sin(phihat)*tan(thetahat)+rhat*cos(phihat)*tan(thetahat);
                qhat*cos(phihat)-rhat*sin(phihat)];
        att_hat = att_hat + (P.Ts/N) * att_f;
        
        att_A = [ qhat*cos(phihat)*tan(thetahat)-rhat*sin(phihat)*tan(thetahat), ...
                    (qhat*sin(phihat)+rhat*cos(phihat))/(cos(thetahat)^2);
                    -qhat*sin(phihat)-rhat*cos(phihat), ...
                    0];
                
        att_P = att_P + (P.Ts/N)*(att_A*att_P+att_P*att_A'+att_Q);
        
        phihat = att_hat(1);
        thetahat = att_hat(2);
    end
    
    
    att_y = [lpf_accel_x;lpf_accel_y;lpf_accel_z];
    
    for i=1:3
        if(abs(att_y(i) - att_y_old(i)) > 2)
        
            att_R = P.sigma_accel^2;
            att_C =  [0, qhat*Vahat*cos(thetahat)+P.gravity*cos(thetahat); 
              -P.gravity*cos(phihat)*cos(thetahat), ...
              -rhat*Vahat*sin(thetahat)-phat*Vahat*cos(thetahat)+P.gravity*sin(phihat)*sin(thetahat);
               P.gravity*sin(phihat)*cos(thetahat), (qhat*Vahat+P.gravity*cos(phihat))*sin(thetahat)];
            att_L = att_P*att_C(i,:)'*inv(att_R + att_C(i,:)*att_P*att_C(i,:)');
            att_P = (diag([1,1])-att_L*att_C(i,:))*att_P; 
            h_xhat_u = [qhat*Vahat*sin(thetahat)+P.gravity*sin(thetahat);
                        rhat*Vahat*cos(thetahat)-phat*Vahat*sin(thetahat)-P.gravity*cos(thetahat)*sin(phihat);
                        -qhat*Vahat*cos(thetahat)-P.gravity*cos(thetahat)*cos(phihat)];
            att_hat =  att_hat + att_L*(att_y(i) - h_xhat_u(i));

            phihat = att_hat(1);
            thetahat = att_hat(2);
        end
    
    end
    att_y_old = att_y;
    
        
  
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
