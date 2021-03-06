function [Point_estim] = Point_estim_init(FirstNumber_Z, FirstNumXY, param, Options)
%% Nokia
if param == 1
    %% CAMERA INIT
 
    Point_estim.camera.Cam_F = 3.7e-3;          % focal lenght, meters

    Point_estim.camera.L = [5.9088e-3; 4.4316e-3]; % [5.76e-3; 4.29e-3]; % the nearest standart camera matrix size, meters
    Point_estim.camera.cam_res = [640; 480];    % camera resolution
    Point_estim.camera.koef_pixel2meters = (Point_estim.camera.L./Point_estim.camera.cam_res);

    %% FILTER INIT
    Point_estim.filter.sko_pixels = 7;          % rms observations noise in pixels
    Point_estim.filter.dispersion_Frame_Meas_X = (Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters(1))^2;   % rms observations noise X on frame
    Point_estim.filter.dispersion_Frame_Meas_Y = (Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters(2))^2;   % rms observations noise Y on frame

    % rms noise observation on frame if 1 keypoints on frame
    Point_estim.filter.dispersion_Frame_Meas = [Point_estim.filter.dispersion_Frame_Meas_X;
                                                Point_estim.filter.dispersion_Frame_Meas_Y];  

    %% X3_0 and Dx3
    %% x3_xcam, diagonal Dx3_xcam
    Point_estim.filter.x3_xcam = [0; 0; 0];                                                  % a priori/starting camera coordinates xcam
    Point_estim.filter.initial_uncertainty_dispersion_x3_xcam = [1; 1; 1].^2;                % initial_uncertainty xcam  
    %% q3_xcam, diagonal Dx3_qcam
    Point_estim.filter.x3_qcam = [1; 0; 0; 0];                                               % a priori/starting camera orientations qcam
    Point_estim.filter.initial_uncertainty_dispersion_x3_qcam = [0.1; 0.1; 0.1; 0.1].^2;     % initial_uncertainty qcam
    %% x3_xn, diagonal Dx3_xn
    Point_estim.filter.Xn3 = 2.2;        % approximate distance from camera lens to keypoints, m
    Point_estim.filter.Gxn = 1.1;       % rsm keypoints coordinates  in 3D, m
    for i = 1:FirstNumber_Z
        % aray a priori/starting keypoints coordinates in first frame xn
        Point_estim.filter.x3_xn_all(3*i-2:3*i, 1) = [(Point_estim.filter.Xn3/Point_estim.camera.Cam_F)*[FirstNumXY(i,1); FirstNumXY(i,2)].*Point_estim.camera.koef_pixel2meters; Point_estim.filter.Xn3]; 
        % initial_uncertainty xn
        Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*i-2:3*i, 1) = [Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters; Point_estim.filter.Gxn].^2;          
    end     

    %% diagonal Dksi3
    Point_estim.filter.ksi_x3_xcam_dispersion =  [0.0001; 0.0001; 0.0001].^2;
    Point_estim.filter.ksi_x3_qcam_dispersion =  [0.01; 0.00001; 0.01; 0.00001].^2;
    Point_estim.filter.ksi_x3_xn_dispersion = 0.0001^2;
end

%% Mi5S
if param == 2
    %% CAMERA INIT
 
    Point_estim.camera.Cam_F = 4.73e-3;          % focal lenght, meters

    Point_estim.camera.L = [6.25e-3; 4.65e-3];  % the nearest standart camera matrix size, meters
    Point_estim.camera.cam_res = [1920; 1080];    % camera resolution
    Point_estim.camera.koef_pixel2meters = (Point_estim.camera.L./Point_estim.camera.cam_res);

    %% FILTER INIT
    Point_estim.filter.sko_pixels = 20;          % rms observations noise in pixels
    Point_estim.filter.dispersion_Frame_Meas_X = (Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters(1))^2;   % rms observations noise X on frame
    Point_estim.filter.dispersion_Frame_Meas_Y = (Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters(2))^2;   % rms observations noise Y on frame

    % rms noise observation on frame if 1 keypoints on frame
    Point_estim.filter.dispersion_Frame_Meas = [Point_estim.filter.dispersion_Frame_Meas_X;
                                                Point_estim.filter.dispersion_Frame_Meas_Y];  

    %% X3_0 and Dx3
    %% x3_xcam, diagonal Dx3_xcam
    Point_estim.filter.x3_xcam = [0; 0; 0];                                                  % a priori/starting camera coordinates xcam
    Point_estim.filter.initial_uncertainty_dispersion_x3_xcam = [1; 1; 1].^2;                % initial_uncertainty xcam  
    %% q3_xcam, diagonal Dx3_qcam
    Point_estim.filter.x3_qcam = [1; 0; 0; 0];                                               % a priori/starting camera orientations qcam
    Point_estim.filter.initial_uncertainty_dispersion_x3_qcam = [0.1; 0.1; 0.1; 0.1].^2;     % initial_uncertainty qcam
    %% x3_xn, diagonal Dx3_xn
    Point_estim.filter.Xn3 = 2;        % approximate distance from camera lens to keypoints, m
    Point_estim.filter.Gxn = 2;       % rsm keypoints coordinates  in 3D, m
    for i = 1:FirstNumber_Z
        % aray a priori/starting keypoints coordinates in first frame xn
        Point_estim.filter.x3_xn_all(3*i-2:3*i, 1) = [(Point_estim.filter.Xn3/Point_estim.camera.Cam_F)*[FirstNumXY(i,1); FirstNumXY(i,2)].*Point_estim.camera.koef_pixel2meters; Point_estim.filter.Xn3]; 
        % initial_uncertainty xn
        Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*i-2:3*i, 1) = [Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters; Point_estim.filter.Gxn].^2;          
    end     

    %% diagonal Dksi3
    a = 0.00001
    a1 = 0.001
    a2 = 0.001
    a3 = 0.001
    Point_estim.filter.ksi_x3_xcam_dispersion =  [a; a; a].^2;
    Point_estim.filter.ksi_x3_qcam_dispersion =  [0.0001; 0.0001; 0.000001; 0.000001].^2;
    Point_estim.filter.ksi_x3_xn_dispersion = 0.0001^2;
    
    %% only roll xcam
    % a = 0.00001
    % [0.0001; 0.0001; 0.000001; 0.000001]
    %% only pitch xcam
    % a = 0.00001
    % [0.0001; 0.000001; 0.0001; 0.000001]
    %% only yaw xcam
    % a = 0.00001
    % [0.0001; 0.000001; 0.000001; 0.000001]
    %% motion_X_and_no_rotation
    % a = 0.00001
    % a1 = 0.001
    % [0.000001; 0.000001; 0.000001; 0.000001];
    %% motion_Y_and_no_rotation
    % a = 0.00001
    % a2 = 0.001
    % [0.00001; 0.00001; 0.00001; 0.00001];
    %% motion_Z_and_no_rotation
    % a = 0.00001
    % a3 = 0.001
    % [0.00001; 0.00001; 0.00001; 0.00001];
end
return