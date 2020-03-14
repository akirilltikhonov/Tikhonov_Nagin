function [fi123_mas, myX_mas, POINT_RPY3_mas, FramePoint_mas, Options] = Dynamic(Options,Point_estim)
a = 1;
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

% ENU2RPY1 = [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]; % X
% ENU2RPY2 = [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)]; % Y
% ENU2RPY3 = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]; % Z

%ENU2RPY = ENU2RPY3*ENU2RPY2*ENU2RPY1; % rotation matrix

ENU2RPY = rpy2mat(fi123);


%% Camera dynamic
% ZZZ = 2*Options.Vku*cos(tt/2); 
myX = [Options.Vku*1/Options.wVu*sin(Options.wVu*tt); Options.Vku*1/Options.wVu*cos(Options.wVu*tt); 0];    % coordinates
%myV = [Options.Vku*cos(Options.wVu*tt); -Options.Vku*sin(Options.wVu*tt); 0];                              % velocity
%myA = [-Options.Vku*Options.wVu*sin(Options.wVu*tt); -Options.Vku*Options.wVu*cos(Options.wVu*tt); 0];     % acceleration

myX_mas(:,k) = myX;     % all points camera position

%% FramePoints without skoFrame
N = 1;
while (N<=Options.Number_Z)
% coordinates of special points in RPY
POINT_RPY(3*N-2:3*N) = ENU2RPY*(Options.PointsZ(3*N-2:3*N) - myX);
POINT_RPY3_mas(N,k) = POINT_RPY(3*N);

% true frame coordinates special points
FramePoint_mas(2*N-1:2*N,k) = Point_estim.camera.Cam_F/POINT_RPY(3*N)*[POINT_RPY(3*N-2); POINT_RPY(3*N-1)];

N = N+1;
end 

end
