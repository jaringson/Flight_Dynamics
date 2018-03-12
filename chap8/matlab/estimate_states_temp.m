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
    persistent att_P
    persistent att_y_old
    
    persistent gps_hat    
    persistent gps_P
    persistent gps_y_old
    
    persistent pnhat
    persistent pehat
    persistent psihat
    persistent chihat
    persistent Vghat
    persistent wnhat
    persistent wehat
    
    persistent phihat
    persistent thetahat
    
    att_Q = 10^-5*diag([1,1]);
%     gps_Q = 10^-9*diag([10^5,10^5,10^2,10^6,1,1,10^1]);
    gps_Q = diag([.1,.1,.1,.0001,.1,.1,.000001]);
    
    
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
        
        gps_P = diag([10^2, 10^2, 1^2, (10*pi/180)^2, 0, 0, (5*pi/180)^2]);
        gps_hat = [P.pn0; P.pe0; P.Va0; P.psi0; 0; 0; P.psi0];
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
        gps_y_old = [lpf_gps_n, lpf_gps_e, lpf_gps_Vg, lpf_gps_course, 0, 0];
    end
    
    phat = lpf_gyro_x;
    qhat = lpf_gyro_y;
    rhat = lpf_gyro_z;
    
    hhat = lpf_static/(P.rho*P.gravity);
    
    Vahat = ((2/P.rho)*lpf_diff)^0.5;
    
    
    if t ==0
        phihat = atan(lpf_accel_y/lpf_accel_z);
        thetahat = asin(lpf_accel_x/P.gravity);

        pnhat = lpf_gps_n;
        pehat = lpf_gps_e;
        chihat = lpf_gps_course;

        Vghat = lpf_gps_Vg;

        psihat = chihat;

        wnhat = 0;
        wehat = 0;
    end
    
    
    
    N = 10;
    for i=0:N
        %%%% Attitude %%%%
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
        
        %%%% GPS %%%%
        psihat_d = qhat*(sin(phihat)/cos(thetahat)) + rhat*(cos(phihat)/cos(thetahat));
        Vghat_d = (Vahat/Vghat)*psihat_d*(-wnhat*sin(psihat)+wehat*cos(psihat));
        chihat_d = (P.gravity/Vghat)*tan(phihat)*cos(chihat-psihat);
        gps_f = [Vghat*cos(chihat); 
                 Vghat*sin(chihat);
                 Vghat_d;
                 chihat_d;
                 0;
                 0;
                 psihat_d];
        gps_hat = gps_hat + (P.Ts/N) * gps_f;
        
        dVgdot_dpsi = -psihat_d*Vahat*(wnhat*cos(psihat)+wehat*sin(psihat))/Vghat;
        dchidot_dVg = (-P.gravity/Vghat^2)*tan(phihat)*cos(chihat-psihat);
        dchidot_dchi = (-P.gravity/Vghat)*tan(phihat)*sin(chihat-psihat);
        dchidot_dpsi = (P.gravity/Vghat)*tan(phihat)*sin(chihat-psihat);
        
        gps_A = [0, 0, cos(chihat), -Vghat*sin(chihat), 0, 0, 0;
                 0, 0, sin(chihat),  Vghat*cos(chihat), 0, 0, 0;
                 0, 0, -Vghat_d/Vghat, 0, ...
                       -psihat_d*Vahat*sin(psihat)/Vghat, ...
                        psihat_d*Vahat*cos(psihat)/Vghat, ...
                        dVgdot_dpsi;
                 0, 0, dchidot_dVg, dchidot_dchi, 0, 0, dchidot_dpsi;
                 zeros(1,7);
                 zeros(1,7);
                 zeros(1,7)];
        gps_P = gps_P + (P.Ts/N)*(gps_A*gps_P+gps_P*gps_A'+gps_Q);
        
        pnhat = gps_hat(1);
        pehat = gps_hat(2);
        Vghat = gps_hat(3);
        chihat = wrapToPi(gps_hat(4));
        wnhat = gps_hat(5);
        wehat = gps_hat(6);
        psihat = wrapToPi(gps_hat(7));
        
                 
    end
    
    
    att_y = [lpf_accel_x;lpf_accel_y;lpf_accel_z];
    att_R = P.sigma_accel^2;
    
    for i=1:3
        if(abs(att_y(i) - att_y_old(i)) > 2)
        
            
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
    
    
   
    gps_y = [lpf_gps_n, lpf_gps_e, lpf_gps_Vg, lpf_gps_course, 0, 0];
    gps_R = [P.sigma_n_gps^2, P.sigma_e_gps^2, P.sigma_Vg_gps^2,  P.sigma_course_gps^2+.1, .7, .7];
%     gps_R = [100, 100, 200, .01, .01, .01];
    for i=1:6
        
        if(abs(gps_y(i) - gps_y_old(i)) > 0)
           gps_C = [1,0,0,0,0,0,0;
                    0,1,0,0,0,0,0;
                    0,0,1,0,0,0,0;
                    0,0,0,1,0,0,0; 
               0,0,-cos(chihat),Vghat*sin(chihat),1,0,-Vahat*sin(psihat);
               0,0,-sin(chihat),-Vghat*cos(chihat),0,1,Vahat*cos(psihat)];
           gps_L = gps_P*gps_C(i,:)'*inv(gps_R(i) + gps_C(i,:)*gps_P*gps_C(i,:)');
           gps_P = (eye(7)-gps_L*gps_C(i,:))*gps_P; 
           h_xhat_u = [pnhat; pehat; Vghat; chihat; Vahat*cos(psihat)+wnhat-Vghat*cos(chihat);
               Vahat*sin(psihat)+wehat-Vghat*sin(chihat)];
           
           gps_hat =  gps_hat + gps_L*(gps_y(i) - h_xhat_u(i));

           pnhat = gps_hat(1);
           pehat = gps_hat(2);
            
           Vghat = gps_hat(3);
           chihat = wrapToPi(gps_hat(4));
           wnhat = gps_hat(5);
           wehat = gps_hat(6);
           psihat = wrapToPi(gps_hat(7));
       end
    end
    gps_y_old = gps_y;
        
  
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
