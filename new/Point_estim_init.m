function [Point_estim] = Point_estim_init(param)

if param == 2
%% camera
Point_estim.camera.Cam_F = 0.0028; %focal length
Point_estim.camera.L = 2.7e-3; % camera matrix size, meters
Point_estim.filter.sko_Frame_Meas =10*Point_estim.camera.L/480;
%size/Npixels

%% filter init
%Dn
Point_estim.filter.Dn = diag([Point_estim.filter.sko_Frame_Meas,Point_estim.filter.sko_Frame_Meas]).^2;
%observation on a two-dimensional (camera) image

%Dksi
Point_estim.filter.ksi_sko=0.05;    %rms deviation of model dinamic
Point_estim.filter.Dksi = diag([Point_estim.filter.ksi_sko, Point_estim.filter.ksi_sko, Point_estim.filter.ksi_sko]).^2;
% model dinamic noise variance matrix

%Dx
Point_estim.filter.xoc = [1; 1; 1];                 %a priori/starting coordinates special point
Point_estim.filter.initial_uncertainty_sko = 10;    %initial_uncertainty
Point_estim.filter.Dx = diag([Point_estim.filter.initial_uncertainty_sko, Point_estim.filter.initial_uncertainty_sko, Point_estim.filter.initial_uncertainty_sko]).^2;
%variance matrix of the estimation vector state
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