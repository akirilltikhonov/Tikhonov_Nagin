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
load('keypoints_numbers_database2.mat')      % load keypoints on each frame

%% MAIN OPTIONS
Options.N_MODEL = length(NumXY_frame);      % amount of frames
Options.F_frame = 29.26;                    % frames per second
Options.T = 1/Options.F_frame;              % frame duration
Options.MODEL_TIME_SEC = Options.N_MODEL/Options.F_frame; % observation (video) time


FirstNumber_Z = size(NumXY_frame{1, 1}, 1);     % amount keypoints in first frame
FirstNumXY = NumXY_frame{1, 1};                 % keypoints coordinates X,Y and their number in first frame
[Point_estim] = Point_estim_init(FirstNumber_Z, FirstNumXY);     % initialization

for i = 1:Options.N_MODEL

    NumXY = NumXY_frame{1, i};          % numbers and X,Y pixel coordinates of keypoints
    Number_Z = size(NumXY, 1);          % amount of keypoints on current frame
    
    if Number_Z ~= 0
        [Point_estim] = Point_estim_init_current(NumXY, Number_Z, Point_estim);

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
        Point_estim.filter.x3_qcam = Point_estim.filter.x3(4:7);
        
        diagDx3 = diag(Point_estim.filter.Dx3);
        Point_estim.filter.initial_uncertainty_dispersion_x3_xcam = diagDx3(1:3);
        Point_estim.filter.initial_uncertainty_dispersion_x3_qcam = diagDx3(4:7);
        
        for j = 1:Number_Z
            Point_estim.filter.x3_xn_all(3*NumXY(j,3)-2:3*NumXY(j,3)) = Point_estim.filter.x3(7+3*j-2:7+3*j);
            Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumXY(j,3)-2:3*NumXY(j,3)) = diagDx3(7+3*j-2:7+3*j);
        end
        
        %% Increase Dx3 for keypoints coordinates which aren't detected
        NumNoDetKP = setdiff((1:size(Point_estim.filter.x3_xn_all, 1)/3)', NumXY(:, 3));
        for j = 1:size(NumNoDetKP, 1)
            Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumNoDetKP(j)-2:3*NumNoDetKP(j), 1) = Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumNoDetKP(j)-2:3*NumNoDetKP(j), 1) + ones(3, 1)*Point_estim.filter.ksi_x3_xn_dispersion;
        end

    else
        %% if no one keypoints -> only extrapolation stage
        Point_estim.filter.initial_uncertainty_dispersion_x3_xcam = Point_estim.filter.initial_uncertainty_dispersion_x3_xcam + Point_estim.filter.ksi_x3_xcam_dispersion;
        Point_estim.filter.initial_uncertainty_dispersion_x3_qcam = Point_estim.filter.initial_uncertainty_dispersion_x3_qcam + Point_estim.filter.ksi_x3_qcam_dispersion;
        
        Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all = Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all + ones(size(Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all, 1), 1)*Point_estim.filter.ksi_x3_xn_dispersion;
    end

    Xcam(1:3, i) = Point_estim.filter.x3_xcam;
    normQcam (1, i) = norm(Point_estim.filter.x3_qcam);
    RPY(1:3, i) = rad2deg(q2rpy(Point_estim.filter.x3_qcam));
    Npoints(1, i) = Number_Z;
    X1(1:3, i) = Point_estim.filter.x3_xn_all(1:3, 1);
    if length(Point_estim.filter.x3_xn_all) >= 399*3;
        X399(1:3, i) = Point_estim.filter.x3_xn_all(399*3-2:399*3, 1);
    end
    if length(Point_estim.filter.x3_xn_all) >= 618*3;
        X618(1:3, i) = Point_estim.filter.x3_xn_all(618*3-2:618*3, 1);
    end    
end

%%
t=1:Options.N_MODEL;          %all observations
l=t/Options.F_frame; 

%% Npoints
figure
plot(l,Npoints)
legend ('���������� ������ ����� �� �����������')
xlabel('�����, �')
ylabel('���-�� ������ �����')
grid on
title('����������� ���������� ������ ����� �� �������')

%%
figure
plot(l,normQcam)
legend ('����� �����������')
xlabel('�����,�')
ylabel('�������� �����')
grid on
title('����������� ����� ����������� �������� ������ �� �������')

%% RPY angles
figure
plot(l,RPY)
legend ('Roll', 'Pitch', 'Yaw')
xlabel('�����, �')
ylabel('���� ��������, ����')
grid on
title('����������� ����� RPY ���������� ������ �� �������')

%% Xcam
figure
plot(l,Xcam)
legend ('X', 'Y', 'Z')
xlabel('�����, �')
ylabel('���������� ������, �')
grid on
title('����������� ��������� ������ �� �������')


