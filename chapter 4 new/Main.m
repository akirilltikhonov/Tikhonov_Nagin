%%%%
% camera's orientation estimation
% we know camera coorninetes and coordinates few special points in ENU with RTK error
% we measure points position in video frame - Y1 , Y2
% we know relationship X -> Y -> Y1,Y2
% Use EKF and Estimate camera's orientation (quaternion) X2=q

%%
clear all
close all
clc

addpath('Additionally');
%% MAIN OPTIONS
[Options] = Main_options();

%% FILTER AND CAMERA INITIALIZATION
Point_estim = Point_estim_init(2, Options);

%% MAIN ALGORITHM
%% Camera dynamic
[fi123_mas, myX_mas, POINT_RPY3_mas, FramePoint_mas, Options] = Dynamic(Options,Point_estim);
fi123_mas_deg = rad2deg(fi123_mas);
%% FramePoint (Y) and EKF
amount = 5;
for i = 1:1:amount
%% for load seed
load('seed');
rng(seed(i));
%% for new seed
%seed(i) = rng();

%%
Options.RTKcam = 1*randn(3,Options.N_MODEL);
Options.RTKpointsZ = 1*randn(3*Options.Number_Z,Options.N_MODEL);
Options.skoFrames = 1*randn(2*Options.Number_Z,Options.N_MODEL);

%%
[Y2_mas, x2_mas, error, normX2] = FramePoint_and_EKF(fi123_mas, Options, FramePoint_mas, myX_mas, Point_estim, POINT_RPY3_mas);

x2_mas_deg(3*i-2:3*i,1:Options.N_MODEL) = rad2deg(x2_mas);
%% Error
error_RPY(3*i-2:3*i, 1:Options.N_MODEL) = error;

error_R(i,1:Options.N_MODEL) = error(1,:);
error_P(i,1:Options.N_MODEL) = error(2,:);
error_Y(i,1:Options.N_MODEL) = error(3,:);

%% Norm
normX2_all(i, 1:Options.N_MODEL) = normX2;
end

%% for save new seed
%save('seed.mat', 'seed');       % save 'seed' random realizations of noise RTK and skoFrame

%%
for j = 1:1:Options.N_MODEL
%% RMSE on Roll, Pitch, Yaw for every time of simulation for 'amount' realizations
%expected value = 0
error_R_RMSE(j:Options.N_MODEL) = sqrt((sum(error_R(:,j).^2)/(amount-1)));
error_P_RMSE(j:Options.N_MODEL) = sqrt((sum(error_P(:,j).^2)/(amount-1)));
error_Y_RMSE(j:Options.N_MODEL) = sqrt((sum(error_Y(:,j).^2)/(amount-1)));

error_RPY_RMSE = [error_R_RMSE; error_P_RMSE; error_Y_RMSE];
end

%%
t=1:Options.N_MODEL;          %all observations
l=t/Options.F_frame; 

%% error state vector X2 coordinate
figure
plot(l,error)
legend ('Ошибка по углу Roll', 'Ошибка по углу Pitch', 'Ошибка по углу Yaw')
xlabel('Время,с')
ylabel('Ошибка оценивания, град')
grid on
title('Зависимость ошибки оценивания углов ориентации камеры от времени')
% ylim([min(error(:))-1 max(error(:))+1])

%%
figure
plot(l,normX2_all)
legend ('Норма (кватерниона) вектора состояния X2')
xlabel('Время,с')
ylabel('Значение нормы')
grid on
title('Зависимость нормы (кватерниона) вектора состояния X2 от времени')
% ylim([min(normX2(:))-1 max(normX2(:))+1])


%% Camera Movement: 
% without RTK solution
figure;
G = 1;
while (G <= Options.Number_Z)
plot3(Options.PointsZ(3*G-2),Options.PointsZ(3*G-1),Options.PointsZ(3*G),'*'); hold on; 
text (0, -1, 25, 'Особые точки');
G = G + 1;
end
plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:));
plot3(myX_mas(1,1), myX_mas(2,1),myX_mas(3,1));
text (myX_mas(1,1), myX_mas(2,1), myX_mas(3,1), '  Камера');
arrow3([myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)], [myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)+2]);
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
%  
%% Trajectory point on the screen
F = 1;
while(F <= Options.Number_Z)
figure
plot(Y2_mas(2*F-1,:), Y2_mas(2*F,:), '-*');
xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
F = F + 1;
end

%% 
%% Roll angle
figure
plot(l,error_R)
legend ('Ошибка по углу Roll для N прогонов')
xlabel('Время, с')
ylabel('Ошибка оценивания, град')
grid on
title('Зависимость ошибки оценивания угла Roll ориентации камеры от времени')
% ylim([min(error_R(:))-1 max(error_R(:))+1])

%% Pitch angle
figure
plot(l,error_P)
legend ('Ошибка по углу Pitch для N прогонов')
xlabel('Время, с')
ylabel('Ошибка оценивания, град')
grid on
title('Зависимость ошибки оценивания угла Pitch ориентации камеры от времени')
% ylim([min(error_P(:))-1 max(error_P(:))+1])

%% Yaw angle
figure
plot(l,error_Y)
legend ('Ошибка по углу Yaw для N прогонов')
xlabel('Время, с')
ylabel('Ошибка оценивания, град')
grid on
title('Зависимость ошибки оценивания Yaw ориентации камеры от времени')
% ylim([min(error_Y(:))-1 max(error_Y(:))+1])

%% All 3 angles RMSE 
figure
plot(l,error_RPY_RMSE)
legend ('СКОш угла Roll для каждого момента времени для N реализаций', 'СКОш угла Pitch для каждого момента времени для N реализаций', 'СКОш угла Yaw для каждого момента времени для N реализаций')
xlabel('Время,с')
ylabel('СКОш, град')
grid on
title('Зависимость СКОш углов ориентации камеры от времени')
% ylim([min(error_RPY_RMSE(:))-1 max(error_RPY_RMSE(:))+1])