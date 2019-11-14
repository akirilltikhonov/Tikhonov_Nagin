%%%%
% point coordinates estimation
% we know our ENU coordinates
% we measure point position in video frame - Y1 , Y2
% we know relationship X -> Y -> Y1,Y2
% EKF:
% X = [Zx Zy Zz]
% Y = [Y1 Y2]
%%
clear all
close all
clc

%% OPTIONS
MODEL_TIME_SEC = 100;   % observation time
F_frame = 24;           % frames per second
T = 1/F_frame;          % frame duration
N_MODEL = ceil(MODEL_TIME_SEC/T);   % number of observations

sko_Coordinate_Meas = 0.05;         % RTK solution
wVu = 0.5;              % radial frequency
Vku = 2;                % max velocity
PointZ = [0;1;5];       % true point coordinates in ENU frame
ENU2RPY = eye(3);       % rotation matrix RPY to ENU
RPY2ENU = ENU2RPY';     % rotation matrix ENU to RPY

%% Filter and camera initialization
Point_estim = Point_estim_init(2);

%% Camera rotation and framepoint
Tturn=3;                % period of turn
Ufi1deg = 0;            % amplitude of turn (deg) relative to X
Ufi2deg = 0;            % amplitude of turn (deg) relative to Y
Ufi3deg = 60;           % amplitude of turn (deg) relative to Z

%% ENU2RPY error
%T_error = 30;           
error_deg = 1;          % by the end of simulation time error wiil be "error_deg" deg

%%
load('RTK.mat');
load('skoFrame.mat');

KK(1:3,1:2, 2*N_MODEL) = 1;

for k = 1:N_MODEL  
tt = k*T;

myX = [Vku*1/wVu*sin(wVu*tt); Vku*1/wVu*cos(wVu*tt); 0];    % coordinates
myV = [Vku*cos(wVu*tt); -Vku*sin(wVu*tt); 0];               % velocity
myA = [-Vku*wVu*sin(wVu*tt); -Vku*wVu*cos(wVu*tt); 0];      % acceleration

%% 

Xcam = myX+RTK(:,k)*sko_Coordinate_Meas;    %coordinate of camera with RTK solution
%%
myX_mas(:,k) = myX;     % all points camera position
Xcam_mas(:,k) = Xcam;   % all points camera position with RTK solution


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
Y = FramePoint + skoFrame(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise

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


%% Extended Kalman Flter 
%% Extrapolation
Point_estim = Point_estim_extrap(Point_estim);

%% Correction
% if special point hit into camera lens => correction stage
if (abs(FramePoint(1))<Point_estim.camera.L/2) && (abs(FramePoint(2))<Point_estim.camera.L/2) && (POINT_RPY(3)>0)
    [predicted_FramePoint, Point_estim] = Point_estim_correct(Point_estim,Xcam,ENU2RPY_with_error,Y);
    phantom = 0;

% else / if special point don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.xoc = Point_estim.filter.x_extr;
    Point_estim.filter.Dx = Point_estim.filter.Dx_extr; 
    phantom = 1;
end 


%% save for plot
Frame_Point_mas(:,k) = Y;
if phantom == 1
    Frame_Point_mas(1:2,k) = nan; % don't building phantom points
end

% Results
xoc_mas(:,k) = Point_estim.filter.xoc;            
error(:,k) = Point_estim.filter.xoc-PointZ;
x_extr_mas(:,k) = Point_estim.filter.x_extr;
Y_mas(:,k) = Y;
%predicted_FramePoint_mas(:,k) = predicted_FramePoint;


end


%%

t=1:k;          %all observations
l=t/F_frame; 

figure
plot(l,error)
legend ('Ошибка по координате x1','Ошибка по координате x2','Ошибка по координате x3')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error(:))-1 max(error(:))+1])


figure
plot(l,xoc_mas)
legend ('X','Y','Z')
xlabel('Время,с')
ylabel('координаты особой точки,м')
grid on
title('зависимость изменения координат особой точки от времени')

figure
plot(l,x_extr_mas)
legend ('X','Y','Z')
xlabel('Время,с')
ylabel('Экстр. координаты,м')
grid on
title('зависимость экстр. координат он времени')

% figure
% plot(l,K_mas)
% legend ('X','Y','Z')
% xlabel('Время,с')
% ylabel('Коэффициент фильтра Калмана,м')
% grid on
% title('зависимость коэффициента фильтра Калмана от времени')

% figure; plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:) ); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); grid on
% xlabel('X1')
% ylabel('X2')
% zlabel('X3')





%% Trajectory point on the screen

% figure
% comet(Frame_Point_mas(1,:), Frame_Point_mas(2,:));
% xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% 
% figure
% plot(Frame_Point_mas(1,:), Frame_Point_mas(2,:), '*');
% xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);