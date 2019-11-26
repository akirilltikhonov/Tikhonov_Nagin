function [fi123_mas, myX_mas,FramePoint_mas,POINT_RPY3_mas] = Dynamic(Options,Point_estim)

for k = 1:Options.N_MODEL  
tt = k*Options.T;

%% Camera rotation
Ufi1 = deg2rad(Options.Ufi1deg);            % amplitude of turn relative to X
fi1 = Ufi1*sin(tt*(2*pi/Options.Tturn));    % angle of turn


Ufi2 = deg2rad(Options.Ufi2deg);            % amplitude of turn relative to Y
fi2 = Ufi2*sin(tt*(2*pi/Options.Tturn));    % angle of turn


Ufi3 = deg2rad(Options.Ufi3deg);            % amplitude of turn relative to Z
fi3 = Ufi3*sin(tt*(2*pi/Options.Tturn));    % angle of turn

fi123 = [fi1; fi2; fi3];                    % vector angles of turn
fi123_mas(:,k) = fi123;

ENU2RPY1 = [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]; % X
ENU2RPY2 = [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)]; % Y
ENU2RPY3 = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]; % Z

ENU2RPY = ENU2RPY3*ENU2RPY2*ENU2RPY1; % rotation matrix
%{ 
% %% ENU2RPY with error
% 
% %% Growing error with increasing number of measurement for all angles
% 
% %T_error=30;
% 
% Ufi_error_deg =1*k*(Options.error_deg/Options.N_MODEL);         % by the end of simulation time error wiil be "error_deg" deg
% Ufi_error = deg2rad(Ufi_error_deg);             % 
% fi_error = Ufi_error;                           %
% % +0*sin(tt*(2*pi/T_error))
% 
% fi1_with_error = Ufi1*sin(tt*(2*pi/Options.Tturn)) + 1*fi_error;
% fi2_with_error = Ufi2*sin(tt*(2*pi/Options.Tturn)) + 1*fi_error;
% fi3_with_error = Ufi3*sin(tt*(2*pi/Options.Tturn)) + 1*fi_error;
% 
% 
% ENU2RPY1_with_error = [1 0 0; 0 cos(fi1_with_error) sin(fi1_with_error); 0 -sin(fi1_with_error) cos(fi1_with_error)]; % X with error
% ENU2RPY2_with_error = [cos(fi2_with_error) 0 -sin(fi2_with_error); 0 1 0; sin(fi2_with_error) 0 cos(fi2_with_error)]; % Y with error
% ENU2RPY3_with_error = [cos(fi3_with_error) sin(fi3_with_error) 0; -sin(fi3_with_error) cos(fi3_with_error) 0; 0 0 1]; % Z with error
% 
% ENU2RPY_with_error = ENU2RPY3_with_error*ENU2RPY2_with_error*ENU2RPY1_with_error;   %rotation matrix with error
% 
% ENU2RPY_with_error_mas(:,3*k-2:3*k) = ENU2RPY_with_error;
%}

%% Camera dynamic
myX = [Options.Vku*1/Options.wVu*sin(Options.wVu*tt); Options.Vku*1/Options.wVu*cos(Options.wVu*tt); 0];    % coordinates
myV = [Options.Vku*cos(Options.wVu*tt); -Options.Vku*sin(Options.wVu*tt); 0];                               % velocity
myA = [-Options.Vku*Options.wVu*sin(Options.wVu*tt); -Options.Vku*Options.wVu*cos(Options.wVu*tt); 0];              % acceleration

myX_mas(:,k) = myX;     % all points camera position

 %% FramePoints without skoFrame
POINT_RPY1 = ENU2RPY*(Options.PointZ1-myX);        % coordinates of special point 1 in RPY
% POINT_RPY2 = ENU2RPY*(Options.PointZ2-myX);        % coordinates of special point 2 in RPY

POINT_RPY3_mas(:,k) = POINT_RPY1(3);

FramePoint1(1:2,1) = Point_estim.camera.Cam_F/POINT_RPY1(3)*[POINT_RPY1(1); POINT_RPY1(2)]; % true frame coordinates special point 1
% FramePoint2(1:2,1) = Point_estim.camera.Cam_F/POINT_RPY2(3)*[POINT_RPY2(1); POINT_RPY2(2)]; % true frame coordinates special point 2

FramePoint_mas(:,k) = FramePoint1;

end
