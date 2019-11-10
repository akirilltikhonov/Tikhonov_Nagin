function [Point_estim] = Point_estim_extrap(Point_estim)
    %% extrap
    Point_estim.filter.x_extr = Point_estim.filter.xoc;      %extrapolation estimation                          
    Point_estim.filter.Dx_extr = Point_estim.filter.Dx + Point_estim.filter.Dksi; %variance error of extrapolation estimation
return 