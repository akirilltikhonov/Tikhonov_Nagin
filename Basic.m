%function [error,MODEL_TIME_SEC,F_frame,T,N_MODEL] = Basic()
%%%%
% point coordinates estimation
% we know our ENU coordinates
% we measure point position in video frame - Y1 , Y2
% we know relationship X -> Y -> Y1,Y2
% EKF:
% X = [Zx Zy Zz]
% Y = [Y1 Y2]
%%
clear all
close all
clc

%% OPTIONS
MODEL_TIME_SEC = 100;   %observation time
F_frame = 24;           %frames per second
T = 1/F_frame;          %frame duration
N_MODEL = ceil(MODEL_TIME_SEC/T);   %number of observations

sko_Coordinate_Meas = 0.05;         % RTK solution
wVu = 0.5;              %radial frequency
Vku = 1;                %max velocity
PointZ = [0;1;5];       %true point coordinates in ENU frame
ENU2RPY = eye(3);       %rotation matrix RPY to ENU
RPY2ENU = ENU2RPY';     %rotation matrix ENU to RPY

%% filter and camera initialization
Point_estim = Point_estim_init(2);

%% memory allocation
myX_mas(1:3,1:N_MODEL) = 0;
myV_mas(1:3,1:N_MODEL) = 0;
myQ_mas(1:4,1:N_MODEL) = 0;
x_cam_oc_mas(1:10,1:N_MODEL) = 0;
error_X(1:3,1:N_MODEL)=0;
error_V(1:3,1:N_MODEL)=0;
error_Angle(1:3,1:N_MODEL)=0;
points_chosen_mas(1,1:N_MODEL)=0;

ENU2RPY_mas(3:3,1:3*N_MODEL) = 0;
%% Cycle
    
for k = 1:N_MODEL
    
tt = k*T;
%% user dynamics
myX = [Vku*1/wVu*sin(wVu*tt); Vku*1/wVu*cos(wVu*tt); 0];
myV = [Vku*cos(wVu*tt); -Vku*sin(wVu*tt); 0];
myA = [-Vku*wVu*sin(wVu*tt); -Vku*wVu*cos(wVu*tt); 0];
%% frame measurements
Xcam = myX+randn(3,1)*sko_Coordinate_Meas;    %coordinate of camera with rms deviation

%% camera rotation

Tturn=3;                %period of turn

Ufi1deg = 0;
Ufi1 = deg2rad(Ufi1deg);      %amplitude of turn relative to X
fi1 = Ufi1*sin(tt*(2*pi/Tturn));    %angle of turn

Ufi2deg = 60;
Ufi2 = deg2rad(Ufi2deg);            %amplitude of turn relative to Y
fi2 = Ufi2*sin(tt*(2*pi/Tturn));    %angle of turn

Ufi3deg = 60;
Ufi3 = deg2rad(Ufi3deg);            %amplitude of turn relative to Z
fi3 = Ufi3*sin(tt*(2*pi/Tturn));    %angle of turn

ENU2RPY1 = [1 0 0; 0 cos(fi1) sin(fi1); 0 -sin(fi1) cos(fi1)]; % X
ENU2RPY2 = [cos(fi2) 0 -sin(fi2); 0 1 0; sin(fi2) 0 cos(fi2)]; % Y
ENU2RPY3 = [cos(fi3) sin(fi3) 0; -sin(fi3) cos(fi3) 0; 0 0 1]; % Z

ENU2RPY = ENU2RPY3*ENU2RPY2*ENU2RPY1; %rotation matrix

%% pinhole camera model
POINT_RPY = ENU2RPY*(PointZ-myX); %true point coordinates in RPY frame
FramePoint(1:2,1) = Point_estim.camera.Cam_F/POINT_RPY(3)*[POINT_RPY(1); POINT_RPY(2)]; % true frame coordinates
Y = FramePoint + randn(2,1)*Point_estim.filter.sko_Frame_Meas; % measurements - frame coordinates with noise

%% ENU2RPY with error

T_error=30;
Ufi_error_deg = 0;
Ufi_error = deg2rad(Ufi_error_deg);
fi_error = Ufi_error*sin(tt*(2*pi/T_error));

fi1_with_error = Ufi1*sin(tt*(2*pi/Tturn)) + fi_error;
fi2_with_error = Ufi2*sin(tt*(2*pi/Tturn)) + fi_error;
fi3_with_error = Ufi3*sin(tt*(2*pi/Tturn)) + fi_error;


ENU2RPY1_with_error = [1 0 0; 0 cos(fi1_with_error) sin(fi1_with_error); 0 -sin(fi1_with_error) cos(fi1_with_error)]; % X with error
ENU2RPY2_with_error = [cos(fi2_with_error) 0 -sin(fi2_with_error); 0 1 0; sin(fi2_with_error) 0 cos(fi2_with_error)]; % Y with error
ENU2RPY3_with_error = [cos(fi3_with_error) sin(fi3_with_error) 0; -sin(fi3_with_error) cos(fi3_with_error) 0; 0 0 1]; % Z with error

ENU2RPY_with_error = ENU2RPY3_with_error*ENU2RPY2_with_error*ENU2RPY1_with_error; %rotation matrix with error

%% extrap
Point_estim = Point_estim_extrap(Point_estim);

%% correction
% if special point hit into camera lens => correction stage
if (abs(FramePoint(1))<Point_estim.camera.L/2) && (abs(FramePoint(2))<Point_estim.camera.L/2) && (POINT_RPY(3)>0)
    [Point_estim,predicted_FramePoint] = Point_estim_correct(Point_estim,Xcam,ENU2RPY_with_error,Y);
    phantom = 0;

% else / if special point don't hit into camera lens => accept extrapolation values 
% as a priori/starting coordinates special point and variance matrix of the estimation vector state
else
    Point_estim.filter.xoc = Point_estim.filter.x_extr;
    Point_estim.filter.Dx = Point_estim.filter.Dx_extr; 
    phantom = 1;
end 
%% save for plot
Frame_Point_mas(:,k) = Y;
if phantom == 1
    Frame_Point_mas(1:2,k) = nan; % don't building phantom points
end

Frame_Point_number = N_MODEL-length(find(isnan(Frame_Point_mas)))/2;    %Number of point on frame

Point_rpy_mas(:,k) = POINT_RPY;
Xcam_mas(:,k) = Xcam;
myX_mas(:,k) = myX;
myV_mas(:,k) = myV;
myA_mas(:,k) = myA;
xoc_mas(:,k)=Point_estim.filter.xoc;
error(:,k)=Point_estim.filter.xoc-PointZ;
error_max = max(error(:));  %max error value
error_min = min(error(:));  %min error value

%number=240;
%error_mean = [ mean(error(1, [number:k])); mean(error(2, [number:k])); mean(error(3, [number:k]))];
%error_std = [ std(error(1, [number:k])); std(error(2, [number:k])); std(error(3, [number:k]))];

end

%% Trajectory point on the screen
%{
figure
comet(Frame_Point_mas(1,:), Frame_Point_mas(2,:));
xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);

figure
plot(Frame_Point_mas(1,:), Frame_Point_mas(2,:), '*');
xlim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
ylim([-Point_estim.camera.L/2,Point_estim.camera.L/2]);
%}
%%

t=1:k;          %all observations
l=t/F_frame; 

figure
plot(l,error)
legend ('������ �� ���������� x1','������ �� ���������� x2','������ �� ���������� x3')
xlabel('�����,�')
ylabel('������ ����������,�')
grid on
title('����������� ������ ���������� ��������� ������ ����� �� �������')
ylim([error_min-1 error_max+1])

%{
figure
plot(l,xoc_mas)
legend ('x1','x2','x3')
xlabel('�����,�')
ylabel('���������� ������ �����,�')
grid on
title('����������� ��������� ��������� ������ ����� �� �������')
%}
%{
figure; plot3(myX_mas(1,:), myX_mas(2,:),myX_mas(3,:) ); hold on; plot3(PointZ(1),PointZ(2),PointZ(3),'*'); grid on
xlabel('X1')
ylabel('X2')
zlabel('X3')
%}
