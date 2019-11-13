function [Xcam_mas, myX_mas, seed_RTK] = Dynamic(Vku, wVu, sko_Coordinate_Meas, N_MODEL, T)
%%
% load('RTK.mat');      % load seed RTK

for k = 1:N_MODEL  
tt = k*T;

%% camera dynamic
myX = [Vku*1/wVu*sin(wVu*tt); Vku*1/wVu*cos(wVu*tt); 0];    % coordinates
myV = [Vku*cos(wVu*tt); -Vku*sin(wVu*tt); 0];               % velocity
myA = [-Vku*wVu*sin(wVu*tt); -Vku*wVu*cos(wVu*tt); 0];      % acceleration

%%

%RTK(:,k)

RTK = randn(3,1); 
Xcam = myX+RTK*sko_Coordinate_Meas;    %coordinate of camera with RTK solution

seed_RTK(:,k) = RTK; 

%%
myX_mas(:,k) = myX;     % all points camera position
Xcam_mas(:,k) = Xcam;   % all points camera position with RTK solution
end
