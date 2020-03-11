function [Point_estim] = Point_estim_init(param, Options)

%% CAMERA INIT
if param == 1
Point_estim.camera.Cam_F = 0.01;    %focal lenght
end

if param == 2
Point_estim.camera.Cam_F = 0.0028; %focal length
end

Point_estim.camera.L = 2.7e-3; % camera matrix size, meters
Point_estim.filter.sko_Frame_Meas =10*Point_estim.camera.L/480;
%size/Npixels

%% FILTER INIT
%Dn
% Point_estim.filter.Dn2_1 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1]);
% Point_estim.filter.Dn2_2 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1]);
%variance matrix of observations on a two-dimensional (camera) image

%Dx2
Point_estim.filter.x2 = [1; 0; 0; 0];   %a priori/starting state vector (quaternion camera orientation). 0 1 0 0 - camera rotate +180, 0, 0 deg relative X,Y,Z
%[0.9; 0.435; 0; 0];  %[-9127; 0.2359; 0.2359; 0.2359];                   
Point_estim.filter.initial_uncertainty_sko_x2 = 0.1;     %initial_uncertainty
Point_estim.filter.Dx2 = (Point_estim.filter.initial_uncertainty_sko_x2)^2*diag([1,1,1,1]);
%variance matrix of the estimation vector state

%Dksi
ksi = 0.00001;     % Optimal order = 6 -- Tturn=300--> min RMSE  
                   % Optimal order = 5 -- Tturn=30--> min RMSE 
                   % Optimal order = 4 -- Tturn=30--> min RMSE
                   % less than 4 order - norm of quaternion feels not so good

Point_estim.filter.ksi_x2_sko =  ksi;
Point_estim.filter.ksi1_x2_sko = ksi;
Point_estim.filter.ksi2_x2_sko = ksi;
Point_estim.filter.ksi3_x2_sko = ksi;

Point_estim.filter.Dksi_x2 = diag([Point_estim.filter.ksi_x2_sko, Point_estim.filter.ksi1_x2_sko, Point_estim.filter.ksi2_x2_sko, Point_estim.filter.ksi3_x2_sko]);
% noise variance matrix of dinamic state vector

return