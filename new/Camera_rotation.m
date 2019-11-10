function [Y, FramePoint, ENU2RPY_with_error, POINT_RPY] = Camera_rotation(Tturn, Ufi1deg, Ufi2deg, Ufi3deg, error_deg, PointZ, myX, Point_estim, N_MODEL, tt, k)
%% Camera Rotation

Ufi1 = deg2rad(Ufi1deg);            % amplitude of turn relative to X
fi1 = Ufi1*sin(tt*(2*pi/Tturn));    % angle of turn


Ufi2 = deg2rad(Ufi2deg);            % amplitude of turn relative to Y
fi2 = Ufi2*sin(tt*(2*pi/Tturn));    % angle of turn


Ufi3 = deg2rad(Ufi3deg);            % amplitude of turn relative to Z
fi3 = Ufi3*sin(tt*(2*pi/Tturn));    % angle of turn

ENU2RPY1 = [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]; % X
ENU2RPY2 = [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)]; % Y
ENU2RPY3 = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]; % Z

ENU2RPY = ENU2RPY3*ENU2RPY2*ENU2RPY1; % rotation matrix

%% Pinhole camera model
POINT_RPY = ENU2RPY*(PointZ-myX); %true point coordinates in RPY frame
FramePoint(1:2,1) = Point_estim.camera.Cam_F/POINT_RPY(3)*[POINT_RPY(1); POINT_RPY(2)]; % true frame coordinates
Y = FramePoint + randn(2,1)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise

%% ENU2RPY with error
%% 1. Постоянная ошибка для всех углов

% Ufi_error_deg = 0.5;
% Ufi_error = deg2rad(Ufi_error_deg);
% fi_error = Ufi_error*sin(tt*(2*pi/T_error));

%% 2. Растущая ошибка с количеством измерений для всех углов

%T_error=30;
error_deg = 1;

Ufi_error_deg =1*k*(error_deg/N_MODEL);         % by the end of simulation time error wiil be "error_deg" deg
Ufi_error = deg2rad(Ufi_error_deg);             % 
fi_error = Ufi_error;                           %
% +0*sin(tt*(2*pi/T_error))
%%
fi1_with_error = Ufi1*sin(tt*(2*pi/Tturn)) + 1*fi_error;
fi2_with_error = Ufi2*sin(tt*(2*pi/Tturn)) + 1*fi_error;
fi3_with_error = Ufi3*sin(tt*(2*pi/Tturn)) + 1*fi_error;

%%
ENU2RPY1_with_error = [1 0 0; 0 cos(fi1_with_error) sin(fi1_with_error); 0 -sin(fi1_with_error) cos(fi1_with_error)]; % X with error
ENU2RPY2_with_error = [cos(fi2_with_error) 0 -sin(fi2_with_error); 0 1 0; sin(fi2_with_error) 0 cos(fi2_with_error)]; % Y with error
ENU2RPY3_with_error = [cos(fi3_with_error) sin(fi3_with_error) 0; -sin(fi3_with_error) cos(fi3_with_error) 0; 0 0 1]; % Z with error

ENU2RPY_with_error = ENU2RPY3_with_error*ENU2RPY2_with_error*ENU2RPY1_with_error;   %rotation matrix with error
