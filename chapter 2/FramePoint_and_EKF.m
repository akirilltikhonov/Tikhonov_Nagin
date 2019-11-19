function [Y_mas, xoc_mas, error] = FramePoint_and_EKF(ENU2RPY_with_error_mas, POINT_RPY3_mas, FramePoint_mas, myX_mas, PointZ, sko_Coordinate_Meas, Point_estim, N_MODEL, T, RTK, skoFrame);


for k = 1:N_MODEL  
tt = k*T;

%% Coordinates of camera with RTK solution
Xcam = myX_mas(:,k) + RTK(:,k)*sko_Coordinate_Meas;

%% Pinhole camera model
Y = FramePoint_mas(:,k) + skoFrame(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise

%% Extended Kalman Flter 
%% Extrapolation
Point_estim = Point_estim_extrap(Point_estim);

%% Correction
% if special point hit into camera lens => correction stage
if (abs(FramePoint_mas(1,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(2,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(:,k)>0)
    [Point_estim] = Point_estim_correct(Point_estim,Xcam,ENU2RPY_with_error_mas,Y,k);
    phantom = 0;

% else / if special point don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.xoc = Point_estim.filter.x_extr;
    Point_estim.filter.Dx = Point_estim.filter.Dx_extr; 
    phantom = 1;
end 
%% save for plot
Y_mas(:,k) = Y;
if phantom == 1
    Y(1:2,k) = nan; % don't building phantom points
end

% Results
xoc_mas(:,k)=Point_estim.filter.xoc;            
error(:,k)=Point_estim.filter.xoc-PointZ; 

end