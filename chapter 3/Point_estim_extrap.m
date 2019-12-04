function [Point_estim] = Point_estim_extrap(Point_estim)
    %% extrap
    Point_estim.filter.x1_extr = Point_estim.filter.x1;      %extrapolation estimation                          
    Point_estim.filter.Dx1_extr = Point_estim.filter.Dx1 + Point_estim.filter.Dksi1;             %variance error of extrapolation estimation
return