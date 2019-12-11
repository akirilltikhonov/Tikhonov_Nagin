function [Y2_mas, x2_mas, error, normQ, normX2] = FramePoint_and_EKF(fi123_mas, Options, FramePoint_mas, myX_mas, Point_estim, POINT_RPY3_mas)

for k = 1:1:Options.N_MODEL  
tt = k*Options.T;

%% Coordinates of camera with RTK solution
Xcam = myX_mas(:,k) + Options.RTKcam(:,k)*Options.sko_Coordinate_Meas;

%% Coordinates of special points with RTK solution
Options.PointZ1_RTK = Options.PointZ1 + Options.RTKpointZ1(:,k)*Options.sko_Coordinate_Meas;
Options.PointZ2_RTK = Options.PointZ2 + Options.RTKpointZ2(:,k)*Options.sko_Coordinate_Meas;
Options.PointZ3_RTK = Options.PointZ3 + Options.RTKpointZ3(:,k)*Options.sko_Coordinate_Meas;
Options.PointZ4_RTK = Options.PointZ4 + Options.RTKpointZ4(:,k)*Options.sko_Coordinate_Meas;
Options.PointZ5_RTK = Options.PointZ5 + Options.RTKpointZ5(:,k)*Options.sko_Coordinate_Meas;
Options.PointZ6_RTK = Options.PointZ6 + Options.RTKpointZ6(:,k)*Options.sko_Coordinate_Meas;
%% Observation Vector
Y_1 = FramePoint_mas(1:2,k) + Options.skoFrame1(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise
Y_2 = FramePoint_mas(3:4,k) + Options.skoFrame2(:,k)*Point_estim.filter.sko_Frame_Meas;
Y_3 = FramePoint_mas(5:6,k) + Options.skoFrame3(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise
Y_4 = FramePoint_mas(7:8,k) + Options.skoFrame4(:,k)*Point_estim.filter.sko_Frame_Meas;
Y_5 = FramePoint_mas(9:10,k) + Options.skoFrame5(:,k)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise
Y_6 = FramePoint_mas(11:12,k) + Options.skoFrame6(:,k)*Point_estim.filter.sko_Frame_Meas;

Y2 = [Y_1; Y_2; Y_3; Y_4; Y_5; Y_6];

%% Extended Kalman Flter 
%% Extrapolation
Point_estim = Point_estim_extrap(Point_estim);

%% Correction
%  hit check special point Z1 into camera lens
Options.phantomZ1 = (abs(FramePoint_mas(1,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(2,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(1,k)>0);

% hit check special point Z2 into camera lens
Options.phantomZ2 = (abs(FramePoint_mas(3,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(4,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(2,k)>0);

% hit check special point Z3 into camera lens
Options.phantomZ3 = (abs(FramePoint_mas(5,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(6,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(3,k)>0);

% hit check special point Z4 into camera lens
Options.phantomZ4 = (abs(FramePoint_mas(7,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(8,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(4,k)>0);

% hit check special point Z5 into camera lens
Options.phantomZ5 = (abs(FramePoint_mas(9,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(10,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(5,k)>0);

% hit check special point Z6 into camera lens
Options.phantomZ6 = (abs(FramePoint_mas(11,k))<Point_estim.camera.L/2) && (abs(FramePoint_mas(12,k))<Point_estim.camera.L/2) && (POINT_RPY3_mas(6,k)>0);

%% if at least one special point hit into camera lens => correction stage
if ((Options.phantomZ1 || Options.phantomZ2 || Options.phantomZ3 || Options.phantomZ4 || Options.phantomZ5 || Options.phantomZ6) == 1)
    [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y_1,Y_2,Y_3,Y_4,Y_5,Y_6);

% else / if no one special points don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.x2 = Point_estim.filter.x2_extr;
    Point_estim.filter.Dx2 = Point_estim.filter.Dx2_extr; 
end 

%% save for plot
    Y2_mas(:,k) = Y2;

    if Options.phantomZ1 == 0
        Y2(1:2,k) = nan; % don't building phantom points
    end

    if Options.phantomZ2 == 0
        Y2(3:4,k) = nan; % don't building phantom points
    end

    if Options.phantomZ3 == 0
        Y2(5:6,k) = nan; % don't building phantom points
    end
        
    if Options.phantomZ4 == 0
        Y2(7:8,k) = nan; % don't building phantom points
    end

    if Options.phantomZ5 == 0
        Y2(9:10,k) = nan; % don't building phantom points
    end
    
    if Options.phantomZ6 == 0
        Y2(11:12,k) = nan; % don't building phantom points
    end
    
% Results
x2_mas(:,k) = q2rotv(Point_estim.filter.x2);            
error(:,k) = rad2deg(q2rotv(Point_estim.filter.x2) - fi123_mas(:,k)); 

Q = rotv2q(fi123_mas(:,k));
normQ (:,k) = sqrt(Q(1)^2 + Q(2)^2 + Q(3)^2 + Q(4)^2);

normX2 (:,k) = sqrt(Point_estim.filter.x2(1)^2 + Point_estim.filter.x2(2)^2 + Point_estim.filter.x2(3)^2 + Point_estim.filter.x2(4)^2);

end