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
[Options] = Main_options();

%% Filter and camera initialization
Point_estim = Point_estim_init(2, Options);

%% Main algorithm
%% Camera dynamic
[fi123_mas,myX_mas,FramePoint_mas,POINT_RPY3_mas] = Dynamic(Options,Point_estim);


%% FramePoint (Y) and EKF
% amount = 20;
% for i = 1:1:amount

%%
% seed(i) = rng();
Options.RTKcam = randn(3,Options.N_MODEL);
Options.RTKpointZ1 = randn(3,Options.N_MODEL);
Options.RTKpointZ2 = randn(3,Options.N_MODEL);
Options.skoFrame1 = randn(2,Options.N_MODEL);
Options.skoFrame2 = randn(2,Options.N_MODEL);
%%
[Y_mas, x2_mas, error, normQ, normX2] = FramePoint_and_EKF(fi123_mas, Options, FramePoint_mas, myX_mas, Point_estim, POINT_RPY3_mas);


% %% Error
% error_XYZ(3*i-2:3*i, 1:Options.N_MODEL) = error;
% 
% error_X(i,1:Options.N_MODEL) = error(1,:);
% error_Y(i,1:Options.N_MODEL) = error(2,:);
% error_Z(i,1:Options.N_MODEL) = error(3,:);
% end
% 
% save('seed.mat', 'seed');       % save 'seed' random realizations of noise RTK and skoFrame
% 
% for j = 1:1:Options.N_MODEL
% %% RMSE on X, Y, Z for every time of simulation for 'amount' realizations
% %expected value = 0
% error_X_RMSE(j:Options.N_MODEL) = sqrt((sum(error_X(:,j).^2)/(amount-1)));
% error_Y_RMSE(j:Options.N_MODEL) = sqrt((sum(error_Y(:,j).^2)/(amount-1)));
% error_Z_RMSE(j:Options.N_MODEL) = sqrt((sum(error_Z(:,j).^2)/(amount-1)));
% 
% error_XYZ_RMSE = [error_X_RMSE; error_Y_RMSE; error_Z_RMSE];
% end

%% Camera Movement: 
%% without RTK solution
figure; 
plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:)); hold on; 
plot3(Options.PointZ1(1),Options.PointZ1(2),Options.PointZ1(3),'*'); hold on; 
plot3(Options.PointZ2(1),Options.PointZ2(2),Options.PointZ2(3),'*'); hold on; 
plot3(myX_mas(1,1), myX_mas(2,1),myX_mas(3,1) ,'*'); hold on; 
arrow3([myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)], [myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)+2]);

text (myX_mas(1,1), myX_mas(2,1), myX_mas(3,1), '  Камера');
text (Options.PointZ1(1),Options.PointZ1(2),Options.PointZ1(3), '  Особая точка');
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

% %% with RTK solution
% figure; plot3(Xcam_mas(1,:), Xcam_mas(2,:),Xcam_mas(3,:) ); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

%% Trajectory point on the screen

% figure
% comet(Y_mas(1,:), Y_mas(2,:));
% xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);

figure
plot(Y_mas(1,:), Y_mas(2,:), '-*');
xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);

figure
plot(Y_mas(3,:), Y_mas(4,:), '-*');
xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);

t=1:Options.N_MODEL;          %all observations
l=t/Options.F_frame; 

%% error state vector X2 coordinate
figure
plot(l,error)
legend ('Ошибка по 3 углам ориентации')
xlabel('Время,с')
ylabel('Ошибка оценивания, град')
grid on
title('Зависимость ошибки оценивания углов ориентации камеры от времени')
ylim([min(error(:))-1 max(error(:))+1])

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

