function [Y2_mas, x2_mas, error, normX2] = FramePoint_and_EKF(fi123_mas, Options, FramePoint_mas, myX_mas, Point_estim, POINT_RPY3_mas)

for k = 1:1:Options.N_MODEL  
tt = k*Options.T;

%% Coordinates of camera with RTK solution
Xcam = myX_mas(:,k) + Options.RTKcam(:,k)*Options.sko_Coordinate_Meas;

%% Coordinates of special points with RTK solution
Options.PointsZ_RTK = Options.PointsZ + Options.RTKpointsZ(:,k)*Options.sko_Coordinate_Meas;

%% Observation Vector
Y2 = FramePoint_mas(:,k) + Options.skoFrames(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates of special points with noise

%% Extended Kalman Flter 
%% Extrapolation
Point_estim = Point_estim_extrap(Point_estim);

%% Correction
%  hit check special points into camera lens
N = 1;
while(N <= Options.Number_Z)
Options.phantomZ(N,1) = (abs(FramePoint_mas(2*N-1,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(2*N,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(N,k)>0);    
N = N+1;
end

%% if at least one special point hit into camera lens => correction stage
if (any(Options.phantomZ))
    [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y2);

% else no one special points don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.x2 = Point_estim.filter.x2_extr;
    Point_estim.filter.Dx2 = Point_estim.filter.Dx2_extr; 
end 

%% save for plot
    Y2_mas(:,k) = Y2;
for M = 1:1:Options.Number_Z
    if (Options.phantomZ(M) == 0)
        Y2_mas(2*M-1:2*M,k) = nan; % don't building phantom points
    end
end
   
    
% Results
x2_mas(:,k) = q2rpy(Point_estim.filter.x2);            

error(:,k) = rad2deg(q2rpy(Point_estim.filter.x2) - fi123_mas(:,k));

normX2 (:,k) = norm(Point_estim.filter.x2);

end