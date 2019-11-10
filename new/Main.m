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

%%
for k = 1:N_MODEL  
tt = k*T;
%% Camera dynamic
[Xcam, myX] = Dynamic(Vku, wVu, sko_Coordinate_Meas, tt);
myX_mas(:,k) = myX;     % all points camera position

%% Camera rotation and framepoint
Tturn=3;                % period of turn
Ufi1deg = 0;            % amplitude of turn (deg) relative to X
Ufi2deg = 0;            % amplitude of turn (deg) relative to Y
Ufi3deg = 60;           % amplitude of turn (deg) relative to Z

%% ENU2RPY error
%T_error = 30;           
error_deg = 1;          % by the end of simulation time error wiil be "error_deg" deg

%%
amount = 2;     %Number of runs. Minimum 2
for i = 1:1:amount

%Point_estim = Point_estim_init(2);

%% Pinhole camera model
[Y, FramePoint, ENU2RPY_with_error, POINT_RPY] = Camera_rotation(Tturn, Ufi1deg, Ufi2deg, Ufi3deg, error_deg, PointZ, myX, Point_estim, N_MODEL, tt, k);

%% EKF
[Point_estim, Frame_Point_Y] = EKF (Point_estim, FramePoint, POINT_RPY, Xcam, ENU2RPY_with_error, Y);
Frame_Point_mas(1:2,k) = Frame_Point_Y;

%% Results
xoc_mas(:,k)=Point_estim.filter.xoc;            
error(:,k)=Point_estim.filter.xoc-PointZ;

error_XYZ(3*i-2:3*i, k) = error(:,k);
error_X(i, k) = error(1,k);
error_Y(i, k) = error(2,k);
error_Z(i, k) = error(3,k);
end


      
end

for j = 1:1:N_MODEL
%% RMSE (EV = 0) on X, Y, Z for every time of simulation for 'amount' realizations
%expected value = 0
error_X_std_EV0(j:N_MODEL) = sqrt((sum(error_X(:,j).^2)/(amount-1)));
error_Y_std_EV0(j:N_MODEL) = sqrt((sum(error_Y(:,j).^2)/(amount-1)));
error_Z_std_EV0(j:N_MODEL) = sqrt((sum(error_Z(:,j).^2)/(amount-1)));

error_XYZ_std_EV0 = [error_X_std_EV0; error_Y_std_EV0; error_Z_std_EV0];
end



%%
t=1:N_MODEL;          %all observations
l=t/F_frame; 


%% All 3 coordinates RMSE 
figure
plot(l,error_XYZ_std_EV0)
legend ('???? ?? X', '???? ?? Y', '???? ?? Z ??? ??????? ??????? ??????? ??? N ??????????')
xlabel('?????,?')
ylabel('???, ?')
grid on
title('??????????? ???? ????????? ?????? ????? ?? ???????')
ylim([min(error_XYZ_std_EV0(:))-1 max(error_XYZ_std_EV0(:))+1])


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



%% Camera Movement 
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
% 
% %%
% 
% t=1:k;          %all observations
% l=t/F_frame; 
% 
% figure
% plot(l,error)
% legend ('Ошибка по координате x1','Ошибка по координате x2','Ошибка по координате x3')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error(:))-1 max(error(:))+1])
% 
% figure
% plot(l,xoc_mas)
% legend ('x1','x2','x3')
% xlabel('Время,с')
% ylabel('координаты особой точки,м')
% grid on
% title('зависимость изменения координат особой точки от времени')


