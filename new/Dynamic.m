function [Xcam_mas, myX_mas] = Dynamic(Vku, wVu, sko_Coordinate_Meas, N_MODEL, T)
% 
for k = 1:N_MODEL  
tt = k*T;

%% camera dynamic
myX = [Vku*1/wVu*sin(wVu*tt); Vku*1/wVu*cos(wVu*tt); 0];    % coordinates
myV = [Vku*cos(wVu*tt); -Vku*sin(wVu*tt); 0];               % velocity
myA = [-Vku*wVu*sin(wVu*tt); -Vku*wVu*cos(wVu*tt); 0];      % acceleration

%% 
Xcam = myX+randn(3,1)*sko_Coordinate_Meas;    %coordinate of camera with RTK solution
%%
myX_mas(:,k) = myX;     % all points camera position
Xcam_mas(:,k) = Xcam;   % all points camera position with RTK solution
end
