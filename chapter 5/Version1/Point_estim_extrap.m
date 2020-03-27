function [Point_estim] = Point_estim_extrap(Point_estim)
    %%extrap
    Point_estim.filter.x3_extr = Point_estim.filter.x3;         %extrapolation estimation
    Point_estim.filter.Dx3_extr = Point_estim.filter.Dx3 + Point_estim.filter.Dksi_x3;      %variance error of extrapolation estimation
return