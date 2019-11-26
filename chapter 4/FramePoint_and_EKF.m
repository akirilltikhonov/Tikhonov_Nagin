function [Y_mas, x2_mas, error] = FramePoint_and_EKF(fi123_mas, Options, FramePoint_mas, myX_mas, Point_estim, POINT_RPY3_mas)

for k = 1:Options.N_MODEL  
tt = k*Options.T;

%% Coordinates of camera with RTK solution
Xcam = myX_mas(:,k) + Options.RTKcam(:,k)*Options.sko_Coordinate_Meas;

%% Coordinates of special points with RTK solution
Options.PointZ1_RTK = Options.PointZ1 + Options.RTKpointZ1(:,k)*Options.sko_Coordinate_Meas;
% Options.PointZ2_RTK = Options.PointZ2 + Options.RTKpointZ2(:,k)*Options.sko_Coordinate_Meas;

%% Observation Vector
Y = FramePoint_mas(1:2,k) + Options.skoFrame1(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise

%% Extended Kalman Flter 
%% Extrapolation
Point_estim = Point_estim_extrap(Point_estim);

%% Correction
% if special point hit into camera lens => correction stage
if (abs(FramePoint_mas(1,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(2,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(:,k)>0)
    [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y);
    phantom = 0;

% else / if special point don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.x2 = Point_estim.filter.x2_extr;
    Point_estim.filter.Dx2 = Point_estim.filter.Dx2_extr; 
    phantom = 1;
end 
%% save for plot
Y_mas(:,k) = Y;
if phantom == 1
    Y(1:2,k) = nan; % don't building phantom points
end

% Results
x2_mas(:,k) = q2rotv(Point_estim.filter.x2);            
error(:,k) = rad2deg(q2rotv(Point_estim.filter.x2) - fi123_mas(:,k)); 

end