function [Point_estim] = Point_estim_init(param,Vku,F_frame,sko_Coordinate_Meas)

if param == 2
%% camera init
Point_estim.camera.Cam_F = 0.0028; %focal length
Point_estim.camera.L = 2.7e-3; % camera matrix size, meters
Point_estim.filter.sko_Frame_Meas =10*Point_estim.camera.L/480;
%size/Npixels

%% filter init
%Dn
Point_estim.filter.Dn1 = diag([Point_estim.filter.sko_Frame_Meas,Point_estim.filter.sko_Frame_Meas, sko_Coordinate_Meas, sko_Coordinate_Meas, sko_Coordinate_Meas]).^2;
%observation vector noise variance matrix

%Dksi
Point_estim.filter.ksi_sko_xoc = 0.05;                 %RMS of forming noise of special point's coordinates
Point_estim.filter.ksi_sko_xcam = 1*Vku/F_frame;     %RMS of forming noise of camera's coordinates
Point_estim.filter.Dksi1 = diag([Point_estim.filter.ksi_sko_xoc,Point_estim.filter.ksi_sko_xoc,Point_estim.filter.ksi_sko_xoc, Point_estim.filter.ksi_sko_xcam,Point_estim.filter.ksi_sko_xcam,Point_estim.filter.ksi_sko_xcam]).^2;
% model dinamic noise variance matrix

%Dx1
Point_estim.filter.xoc = [2; 2; 2];         %a priori/starting coordinates of special point
Point_estim.filter.xcam = [1; 1; 1];        %a priori/starting coordinates of camera
Point_estim.filter.x1 = [Point_estim.filter.xoc; Point_estim.filter.xcam];
%a priori/starting/initial approximation of the state vector

Point_estim.filter.initial_uncertainty_xoc = 10;    %initial uncertainty of special point
Point_estim.filter.initial_uncertainty_xcam = 5;    %initial uncertainty of camera's coordinates

Point_estim.filter.Dx1 = diag([Point_estim.filter.initial_uncertainty_xoc, Point_estim.filter.initial_uncertainty_xoc, Point_estim.filter.initial_uncertainty_xoc, Point_estim.filter.initial_uncertainty_xcam, Point_estim.filter.initial_uncertainty_xcam, Point_estim.filter.initial_uncertainty_xcam]).^2;
%variance matrix of the estimation vector state
end

if param == 1
    
%% camera init
Point_estim.camera.Cam_F = 0.01;    %focal lenght
Point_estim.camera.L = 2.7e-3;      %camera matrix size
Point_estim.filter.sko_Frame_Meas = 10*Point_estim.camera.L/480;
%size/Npixels

%% filter init
%Dn
Point_estim.filter.Dn1 = diag([Point_estim.filter.sko_Frame_Meas,Point_estim.filter.sko_Frame_Meas, sko_Coordinate_Meas, sko_Coordinate_Meas, sko_Coordinate_Meas]).^2;
%observation vector noise variance matrix

%Dksi
Point_estim.filter.ksi_sko_xoc = 0.001;              %RMS of forming noise of special point's coordinates
Point_estim.filter.ksi_sko_xcam = Vku/F_frame;      %RMS of forming noise of camera's coordinates
Point_estim.filter.Dksi1 = diag([Point_estim.filter.ksi_sko_xoc,Point_estim.filter.ksi_sko_xoc,Point_estim.filter.ksi_sko_xoc, Point_estim.filter.ksi_sko_xcam,Point_estim.filter.ksi_sko_xcam,Point_estim.filter.ksi_sko_xcam]).^2;
% model dinamic noise variance matrix

%Dx1
Point_estim.filter.xoc = [0; 1; 5];         %a priori/starting coordinates of special point
Point_estim.filter.xcam = [0; 4; 0];        %a priori/starting coordinates of camera
Point_estim.filter.x1 = [Point_estim.filter.xoc; Point_estim.filter.xcam];
%a priori/starting/initial approximation of the state vector

Point_estim.filter.initial_uncertainty_xoc = 0.1;    %initial uncertainty of special point
Point_estim.filter.initial_uncertainty_xcam = 0.1;    %initial uncertainty of camera's coordinates

Point_estim.filter.Dx1 = diag([Point_estim.filter.initial_uncertainty_xoc, Point_estim.filter.initial_uncertainty_xoc, Point_estim.filter.initial_uncertainty_xoc, Point_estim.filter.initial_uncertainty_xcam, Point_estim.filter.initial_uncertainty_xcam, Point_estim.filter.initial_uncertainty_xcam]).^2;
%variance matrix of the estimation vector state
end

return