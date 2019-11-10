function [Point_estim, Frame_Point_Y] = EKF (Point_estim, FramePoint, POINT_RPY, Xcam, ENU2RPY_with_error, Y)
%% Extended Kalman Flter 

%% Extrapolation
Point_estim = Point_estim_extrap(Point_estim);

%% Correction
% if special point hit into camera lens => correction stage
if (abs(FramePoint(1))<Point_estim.camera.L/2) && (abs(FramePoint(2))<Point_estim.camera.L/2) && (POINT_RPY(3)>0)
    [Point_estim] = Point_estim_correct(Point_estim,Xcam,ENU2RPY_with_error,Y);
    phantom = 0;

% else / if special point don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.xoc = Point_estim.filter.x_extr;
    Point_estim.filter.Dx = Point_estim.filter.Dx_extr; 
    phantom = 1;
end 
%% save for plot
Frame_Point_Y = Y;
if phantom == 1
    Frame_Point_Y = nan; % don't building phantom points

end