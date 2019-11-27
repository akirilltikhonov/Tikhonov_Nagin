function [Point_estim] = Point_estim_extrap(Point_estim)
    %% extrap
    Point_estim.filter.x1_extr = Point_estim.filter.x1 + diag([0,0,0,1,1,1])*(Point_estim.filter.ksi_sko_xcam*randn(6,1));      %extrapolation estimation                          
    Point_estim.filter.Dx1_extr = Point_estim.filter.Dx1 + Point_estim.filter.Dksi1;             %variance error of extrapolation estimation
return