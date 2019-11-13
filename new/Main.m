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





%% Main algorithm

%% Camera dynamic
[Xcam_mas, myX_mas, seed_RTK] = Dynamic(Vku, wVu, sko_Coordinate_Meas, N_MODEL, T);

%% FramePoint (Pinhole camera model) and EKF
amount = 10;
for i = 1:1:amount
Point_estim = Point_estim_init(2);      % установка начального приближения для фильтра    
[Frame_Point_mas, xoc_mas, error,seed_skoFrame] = FramePoint_and_EKF(Tturn, Ufi1deg, Ufi2deg, Ufi3deg, error_deg, PointZ, myX_mas, Point_estim, N_MODEL, T, Xcam_mas);

error_XYZ(3*i-2:3*i, 1:N_MODEL) = error;

error_X(i,1:N_MODEL) = error(1,:);
error_Y(i,1:N_MODEL) = error(2,:);
error_Z(i,1:N_MODEL) = error(3,:);


seed_skoFrame_mas(2*i-1:2*i, 1:N_MODEL) = seed_skoFrame;

end

for j = 1:1:N_MODEL
%% RMSE on X, Y, Z for every time of simulation for 'amount' realizations
%expected value = 0
error_X_RMSE(j:N_MODEL) = sqrt((sum(error_X(:,j).^2)/(amount-1)));
error_Y_RMSE(j:N_MODEL) = sqrt((sum(error_Y(:,j).^2)/(amount-1)));
error_Z_RMSE(j:N_MODEL) = sqrt((sum(error_Z(:,j).^2)/(amount-1)));

error_XYZ_RMSE = [error_X_RMSE; error_Y_RMSE; error_Z_RMSE];
end

% %% Camera Movement: 
% % without RTK solution
% figure; plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:) ); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); grid on
% xlabel('X1')
% ylabel('X2')
% zlabel('X3')
% %% with RTK solution
% figure; plot3(Xcam_mas(1,:), Xcam_mas(2,:),Xcam_mas(3,:) ); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); grid on
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


t=1:N_MODEL;          %all observations
l=t/F_frame; 

%% X coordinate
figure
plot(l,error_X)
legend ('Ошибка по координате X для N прогонов')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error_X(:))-1 max(error_X(:))+1])

%% Y coordinate
figure
plot(l,error_Y)
legend ('Ошибка по координате Y для N прогонов')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error_Y(:))-1 max(error_Y(:))+1])

%% Z coordinate
figure
plot(l,error_Z)
legend ('Ошибка по координате Z для N прогонов')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error_Z(:))-1 max(error_Z(:))+1])

%% All 3 coordinates RMSE 
figure
plot(l,error_XYZ_RMSE)
legend ('СКОш по X', 'СКОш по Y', 'СКОш по Z для каждого момента времени для N реализаций')
xlabel('Время,с')
ylabel('СКО, м')
grid on
title('Зависимость СКОш координат особой точки от времени')
ylim([min(error_XYZ_RMSE(:))-1 max(error_XYZ_RMSE(:))+1])


