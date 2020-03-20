function [Point_estim] = Point_estim_init_secondary(NumXY, Number_Z, Point_estim)

%% FILTER INIT SECONDARY                                    Point_estim.filter.sko_Frame_Meas_Y];  
for i = 1:Number_Z
    %% Vector dispersion Frame Measurement for diagonal Dn3
    dispersion_Frame_Meas(2*i-1:2*i, 1) = [Point_estim.filter.dispersion_Frame_Meas];
    
    %% Starting\state vector X3_0\X3 with keypoits coordinates
    if isnan(Point_estim.filter.x3_xn_all(3*NumXY(i,3), 1))
        Point_estim.filter.x3_xn_all(3*NumXY(i,3):3*NumXY(i,3)+2, 1) = [(3/Point_estim.camera.Cam_F)*[NumXY(1,1); NumXY(1,2)].*Point_estim.camera.koef_pixel2meters; 3];
    end
    
    x3_xn(3*i-2:3*i, 1) = Point_estim.filter.x3_xn_all(3*NumXY(i,3):3*NumXY(i,3)+2, 1);
    
    %% Vector uncertainty dispersion for diagonal Dx3
    if isnan (Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumXY(i,3), 1))
        Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumXY(i,3):3*NumXY(i,3)+2, 1) = [Point_estim.filter.sko_pixels*Point_estim.camera.koef_pixel2meters; 1.5].^2;
    end
    
    initial_uncertainty_dispersion_x3_xn(3*i-2:3*i, 1) = Point_estim.filter.initial_uncertainty_dispersion_x3_xn_all(3*NumXY(i,3):3*NumXY(i,3)+2, 1);
end

%% Dn3 Variance matrix of observations on a two-dimensional (camera) image
Point_estim.filter.Dn3 = diag((dispersion_Frame_Meas)');

%% X3_0 Starting\state vector
Point_estim.filter.x3 = [Point_estim.filter.x3_xcam; 
                         Point_estim.filter.x3_qcam;
                         x3_xn];
                     
%% Dx3 variance matrix of the estimation vector state
Point_estim.filter.Dx3 = diag([
    Point_estim.filter.initial_uncertainty_dispersion_x3_xcam;
    Point_estim.filter.initial_uncertainty_dispersion_x3_qcam;
    initial_uncertainty_dispersion_x3_xn
                                ]');

%% Dksi3
% Dksi3 Noise variance matrix of dinamic state vector
Point_estim.filter.Dksi_x3 = diag([
Point_estim.filter.ksi_x3_xcam_dispersion;
Point_estim.filter.ksi_x3_qcam_dispersion;
Point_estim.filter.ksi_x3_xn_dispersion*ones(3*Number_Z,1)
                                    ]');

return