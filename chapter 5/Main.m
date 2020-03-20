%%%%
% estimate: camera coordinates Xcam and orientation qcam, coordinates of N keypoints in 3D Xn.
% we have observation vector Y3 on each frame (NumXY_frame)
% we know relationship Y3 -> Xcam, qcam, Xn
% Use EKF for estimation

%%
clear all
close all
clc

addpath('Additionally');
load('keypoints_numbers_database.mat')      % load keypoints on each frame

%% MAIN OPTIONS
Options.N_MODEL = length(NumXY_frame);      % amount of frames
Options.F_frame = 29.26;                    % frames per second
Options.T = 1/Options.F_frame;              % frame duration
Options.MODEL_TIME_SEC = Options.N_MODEL/Options.F_frame; % observation (video) time

[Point_estim] = Point_estim_init();

for i = 1:Options.N_MODEL

    NumXY = NumXY_frame{1, i};          % numbers and X,Y pixel coordinates of keypoints
    Number_Z = size(NumXY, 1)           % amount of keypoints on current frame
    
    if Number_Z ~= 0
        [Point_estim] = Point_estim_init_secondary(NumXY, Number_Z, Point_estim);

        %% Observation vector
        clear Y2
        for k = 1:Number_Z
            Y2(2*k-1:2*k, 1) = NumXY(k, 1:2)'.*Point_estim.camera.koef_pixel2meters;
        end

        %% Extrapolation stage
        [Point_estim] = Point_estim_extrap(Point_estim);
    
        %% Correction stage
        [Point_estim] = Point_estim_correct(Point_estim, Number_Z, Y2);
        
        %% Save estimation X3 and Dx3
        Point_estim.filter.x3_xcam = Point_estim.filter.x3(1:3);
        Point_estim.filter.x3_qcam = Point_estim.filter.x3(4:7)
        
        diagDx3 = diag(Point_estim.filter.Dx3);
        Point_estim.filter.initial_uncertainty_dispersion_x3_xcam = diagDx3(1:3);
        Point_estim.filter.initial_uncertainty_dispersion_x3_qcam = diagDx3(4:7);
        
        for j = 1:Number_Z
            Point_estim.filter.x3_xn_all(3*NumXY(j,3):3*NumXY(j,3)+2) = Point_estim.filter.x3(7+3*j-2:7+3*j);
            Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumXY(j,3):3*NumXY(j,3)+2) = diagDx3(7+3*j-2:7+3*j);
        end
        
    else
        %% if no one keypoints -> only extrapolation stage
        Point_estim.filter.initial_uncertainty_dispersion_x3_xcam = Point_estim.filter.initial_uncertainty_dispersion_x3_xcam + Point_estim.filter.ksi_x3_xcam_dispersion;
        Point_estim.filter.initial_uncertainty_dispersion_x3_qcam = Point_estim.filter.initial_uncertainty_dispersion_x3_qcam + Point_estim.filter.ksi_x3_qcam_dispersion;
       
    end
    
    
    Xcam(1:3, i) = Point_estim.filter.x3_xcam;
    normQcam (1, i) = norm(Point_estim.filter.x3_qcam);
    RPY(1:3, i) = rad2deg(q2rpy(Point_estim.filter.x3_qcam));
    Npoints(1, i) = Number_Z;
    
end

%%
t=1:Options.N_MODEL;          %all observations
l=t/Options.F_frame; 

%% Npoints
figure
plot(l,Npoints)
legend ('Количество особых точек на изображении')
xlabel('Время, с')
ylabel('Кол-во особых точек')
grid on
title('Зависимость количества особых точек от времени')

%%
figure
plot(l,normQcam)
legend ('Норма кватерниона')
xlabel('Время,с')
ylabel('Значение нормы')
grid on
title('Зависимость нормы кватерниона поворота камеры от времени')

%% RPY angles
figure
plot(l,RPY)
legend ('Roll', 'Pitch', 'Yaw')
xlabel('Время, с')
ylabel('Угол поворота, град')
grid on
title('Зависимость углов RPY ориентации камеры от времени')

%% Xcam
figure
plot(l,Xcam)
legend ('X', 'Y', 'Z')
xlabel('Время, с')
ylabel('Координаты камеры, м')
grid on
title('Зависимость координат камеры от времени')


