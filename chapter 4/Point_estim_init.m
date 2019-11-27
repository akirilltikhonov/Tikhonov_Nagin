function [Point_estim] = Point_estim_init(param, Options)

if param == 2
%% camera
Point_estim.camera.Cam_F = 0.0028; %focal length
Point_estim.camera.L = 2.7e-3; % camera matrix size, meters
Point_estim.filter.sko_Frame_Meas =10*Point_estim.camera.L/480;
%size/Npixels

%% filter init
%Dn
% Point_estim.filter.Dn2_1 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1]);
% Point_estim.filter.Dn2_2 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1]);
%variance matrix of observations on a two-dimensional (camera) image

%Dx2
Point_estim.filter.x2 = [0; 1; 0; 0];                   %a priori/starting state vector (quaternion camera orientation). 0 1 0 0 - camera rotate +180, 0, 0 deg relative X,Y,Z
Point_estim.filter.initial_uncertainty_sko_x2 = 1;     %initial_uncertainty
Point_estim.filter.Dx2 = (Point_estim.filter.initial_uncertainty_sko_x2)^2*diag([1,1,1,1]);
%variance matrix of the estimation vector state

%Dksi
Point_estim.filter.ksi1_x2_sko = 10*(deg2rad(Options.Ufi1deg))*sin((1/Options.F_frame)*(2*pi/Options.Tturn));
Point_estim.filter.ksi2_x2_sko = 10*(deg2rad(Options.Ufi2deg))*sin((1/Options.F_frame)*(2*pi/Options.Tturn));
Point_estim.filter.ksi3_x2_sko = 10*(deg2rad(Options.Ufi3deg))*sin((1/Options.F_frame)*(2*pi/Options.Tturn));
Point_estim.filter.ksi_x2_sko = 10*sqrt((Point_estim.filter.ksi1_x2_sko)^2 + (Point_estim.filter.ksi2_x2_sko)^2 + (Point_estim.filter.ksi3_x2_sko)^2);

Point_estim.filter.Dksi_x2 = diag([Point_estim.filter.ksi_x2_sko, Point_estim.filter.ksi1_x2_sko, Point_estim.filter.ksi2_x2_sko, Point_estim.filter.ksi3_x2_sko]);
% noise variance matrix of dinamic state vector
end

if param == 1
%% camera
Point_estim.camera.Cam_F = 0.01;    %focal lenght
Point_estim.camera.L = 2.7e-3;      %camera matrix size
Point_estim.filter.sko_Frame_Meas = 10*Point_estim.camera.L/480;
%size/Npixels

%% filter init
%Dn
Point_estim.filter.Dn = diag([Point_estim.filter.sko_Frame_Meas,Point_estim.filter.sko_Frame_Meas]).^2;
%observation on a two-dimensional (camera) image

%Dksi
Point_estim.filter.ksi_sko=0.01;    %rms deviation of model dinamic
Point_estim.filter.Dksi = diag([Point_estim.filter.ksi_sko, Point_estim.filter.ksi_sko, Point_estim.filter.ksi_sko]).^2;
% model dinamic noise variance matrix

%Dx
Point_estim.filter.xoc = [1; 1; 1];                 %a priori/starting coordinates special point
Point_estim.filter.initial_uncertainty_sko = 10;    %initial_uncertainty
Point_estim.filter.Dx = diag([Point_estim.filter.initial_uncertainty_sko, Point_estim.filter.initial_uncertainty_sko, Point_estim.filter.initial_uncertainty_sko]).^2;
%variance matrix of the estimation vector state
end

return