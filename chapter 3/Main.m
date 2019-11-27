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

%% MAIN OPTIONS
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
Point_estim = Point_estim_init(2,Vku,F_frame,sko_Coordinate_Meas);

%% Camera rotation and framepoint
Tturn=3;                % period of turn
Ufi1deg = 0;            % amplitude of turn (deg) relative to X
Ufi2deg = 60;           % amplitude of turn (deg) relative to Y
Ufi3deg = 60;           % amplitude of turn (deg) relative to Z

%% ENU2RPY error   
error_deg = 1;          % by the end of simulation time error wiil be "error_deg" deg

%% Main algorithm
%% Camera dynamic
[myX_mas,FramePoint_mas,POINT_RPY3_mas, ENU2RPY_with_error_mas] = Dynamic(Vku, wVu, N_MODEL, T, PointZ, Point_estim, Tturn, Ufi1deg, Ufi2deg, Ufi3deg, error_deg);


%% FramePoint (Y) and EKF
amount = 20;

% for i = 1:1:amount

%%

% seed(i) = rng();

RTK = randn(3,N_MODEL);
skoFrame = randn(2,N_MODEL);

%%

[Frame_Point_mas, xoc_mas, error, error1] = FramePoint_and_EKF(ENU2RPY_with_error_mas, POINT_RPY3_mas,FramePoint_mas, myX_mas, PointZ, sko_Coordinate_Meas, Point_estim, N_MODEL, T, RTK, skoFrame);

% %% Error
% error_XYZ(3*i-2:3*i, 1:N_MODEL) = error;
% 
% error_X(i,1:N_MODEL) = error(1,:);
% error_Y(i,1:N_MODEL) = error(2,:);
% error_Z(i,1:N_MODEL) = error(3,:);
% end
% 
% save('seed.mat', 'seed');       % save 'seed' random realizations of noise RTK and skoFrame
% 
% for j = 1:1:N_MODEL
% %% RMSE on X, Y, Z for every time of simulation for 'amount' realizations
% %expected value = 0
% error_X_RMSE(j:N_MODEL) = sqrt((sum(error_X(:,j).^2)/(amount-1)));
% error_Y_RMSE(j:N_MODEL) = sqrt((sum(error_Y(:,j).^2)/(amount-1)));
% error_Z_RMSE(j:N_MODEL) = sqrt((sum(error_Z(:,j).^2)/(amount-1)));
% 
% error_XYZ_RMSE = [error_X_RMSE; error_Y_RMSE; error_Z_RMSE];
% end

% %% Camera Movement: 
% %% without RTK solution
% figure; plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:)); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); hold on; plot3(myX_mas(1,1), myX_mas(2,1),myX_mas(3,1) ,'*');
% hold on; arrow3([myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)], [myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)+2]);
% text (myX_mas(1,1), myX_mas(2,1), myX_mas(3,1), '  Камера');
% text (PointZ(1),PointZ(2),PointZ(3), '  Особая точка');
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

% %% with RTK solution
% figure; plot3(Xcam_mas(1,:), Xcam_mas(2,:),Xcam_mas(3,:) ); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

% %% Trajectory point on the screen

% figure
% comet(Frame_Point_mas(1,:), Frame_Point_mas(2,:));
% xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% 
% figure
% plot(Frame_Point_mas(1,:), Frame_Point_mas(2,:), '-*');
% xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);


t=1:N_MODEL;          %all observations
l=t/F_frame; 

%% error X,Y,Z coordinates
figure
plot(l,error)
legend ('Ошибка по координате X','Y','Z')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error(:))-1 max(error(:))+1])

%% error Xcam coordinates
figure
plot(l,error1)
legend ('Ошибка по координате Xcam(1)','Xcam(2)', 'Xcam(3)')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат камеры от времени')
ylim([min(error1(:))-1 max(error1(:))+1])

% %% X coordinate
% figure
% plot(l,error_X)
% legend ('Ошибка по координате X для N прогонов')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error_X(:))-1 max(error_X(:))+1])
% 
% %% Y coordinate
% figure
% plot(l,error_Y)
% legend ('Ошибка по координате Y для N прогонов')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error_Y(:))-1 max(error_Y(:))+1])
% 
% %% Z coordinate
% figure
% plot(l,error_Z)
% legend ('Ошибка по координате Z для N прогонов')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error_Z(:))-1 max(error_Z(:))+1])
% 
% %% All 3 coordinates RMSE 
% figure
% plot(l,error_XYZ_RMSE)
% legend ('СКОш по X для каждого момента времени для N реализаций', 'СКОш по Y для каждого момента времени для N реализаций', 'СКОш по Z для каждого момента времени для N реализаций')
% xlabel('Время,с')
% ylabel('СКОш, м')
% grid on
% title('Зависимость СКОш координат особой точки от времени')
% ylim([min(error_XYZ_RMSE(:))-1 max(error_XYZ_RMSE(:))+1])
% 

