function [Point_estim] = Point_estim_extrap(Point_estim)
    %%extrap  
    Point_estim.filter.x2_extr = Point_estim.filter.x2;         %extrapolation estimation
    Point_estim.filter.Dx2_extr = Point_estim.filter.Dx2 + Point_estim.filter.Dksi_x2;      %variance error of extrapolation estimation
return