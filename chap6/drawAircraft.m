
function drawAircraft(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-1000,1000,-1000,1000,-1000,1000]);
        grid on
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
V = 100*[...
    0.75, 0, 0;...          % pt 1
    0.5, -0.25, -0.25; ...  % pt 2
    0.5, 0.25, -0.25; ...   % pt 3
    0.5, 0.25, 0.25; ...    % pt 4
    0.5, -0.25, 0.25; ...   % pt 5 
    -1.25, 0, 0; ...        % pt 6
    0.25, -1, 0; ...     % pt 7
    -0.25, -1, 0; ...    % pt 8
    -0.25, 1, 0; ...     % pt 9
    0.25, 1, 0; ...      % pt 10
    -0.75, -0.5, 0; ...     % pt 11
    -1.25, -0.5, 0; ...     % pt 12
    -1.25, 0.5, 0; ...      % pt 13
    -0.75, 0.5, 0; ...      % pt 14
    -0.75, 0, 0; ...        % pt 15
    -1.25, 0, -0.5;         % pt 16
    ]';

% define faces as a list of vertices numbered above
  F = [...
    1,2,3, NaN;...  % Nose Top
    1,3,4, NaN; ... % Nose Left
    1,4,5, NaN; ... % Nose Bottom
    1,5,2, NaN; ... % Nose Right
    2,3,6, NaN; ... % Body Top
    3,4,6, NaN; ... % Body Left
    5,4,6, NaN; ... % Body Bottom
    5,2,6, NaN; ... % Body Right
    7,8,9,10; ...   % Wing
    11,12,13,14; ...% Elevator
    6,15,16, NaN;   % Rudder
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    myblue;  % Nose
    myblue;  % Nose
    myblue;  % Nose
    myblue;  % Nose
    myblue;  % Body
    myblue;  % Body
    myblue;  % Body
    myblue;  % Body
    mygreen; % Wing
    mygreen; % Elevator
    myblue;  % Rudder
    ];
end
  