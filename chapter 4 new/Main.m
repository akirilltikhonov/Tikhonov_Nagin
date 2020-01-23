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

%% MAIN OPTIONS
[Options] = Main_options();

%% FILTER AND CAMERA INITIALIZATION
Point_estim = Point_estim_init(2, Options);

%% MAIN ALGORITHM
%% Camera dynamic
[fi123_mas, myX_mas, POINT_RPY3_mas, FramePoint_mas] = Dynamic(Options,Point_estim);

Options.RTKcam = 1*randn(3,Options.N_MODEL);
Options.RTKpointsZ = 1*randn(3*Options.Number_Z,Options.N_MODEL);
Options.skoFrames = 1*randn(2*Options.Number_Z,Options.N_MODEL);

[Y2_mas, x2_mas, error, normQ, normX2] = FramePoint_and_EKF(fi123_mas, Options, FramePoint_mas, myX_mas, Point_estim, POINT_RPY3_mas);

%%
t=1:Options.N_MODEL;          %all observations
l=t/Options.F_frame; 

%% error state vector X2 coordinate
figure
plot(l,error)
legend ('Ошибка по углу крена', 'Ошибка по углу тангажа', 'Ошибка по углу рыскания')
xlabel('Время,с')
ylabel('Ошибка оценивания, град')
grid on
title('Зависимость ошибки оценивания углов ориентации камеры от времени')
ylim([min(error(:))-1 max(error(:))+1])

%%
figure
plot(l,normX2)
legend ('Норма (кватерниона) вектора состояния X2')
xlabel('Время,с')
ylabel('Значение нормы')
grid on
title('Зависимость нормы (кватерниона) вектора состояния X2 от времени')
ylim([min(normX2(:))-1 max(normX2(:))+1])





%% Camera Movement: 
% % without RTK solution
% figure;
% G = 1;
% while (G <= Options.Number_Z)
% plot3(Options.PointsZ(3*G-2),Options.PointsZ(3*G-1),Options.PointsZ(3*G),'*'); hold on; 
% %text (Options.PointsZ(3*G-2),Options.PointsZ(3*G-1),Options.PointsZ(3*G), 'Особая точка ');
% G = G + 1;
% end
% plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:));
% plot3(myX_mas(1,1), myX_mas(2,1),myX_mas(3,1) ,'*');
% text (myX_mas(1,1), myX_mas(2,1), myX_mas(3,1), '  Камера');
% arrow3([myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)], [myX_mas(1,1) myX_mas(2,1) myX_mas(3,1)+2]);
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
%  
% %% Trajectory point on the screen
% F = 1;
% while(F <= Options.Number_Z)
% figure
% plot(FramePoint_mas(2*F-1,:), FramePoint_mas(2*F,:), '-*');
% xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
% F = F + 1;
% end




