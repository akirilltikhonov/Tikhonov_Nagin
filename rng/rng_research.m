clear all
close all
clc

N = 3;

% rng('default')      % allways the same random number

for k=1:3

seed(k) = rng();
RTK = randn(3,N)
% skoFrame = randn(2,N)

end

rng(seed(2));
RTK = randn(3,N)

% rng(seed(3));
% skoFrame = randn(2,N)

% save('seed.mat', 'seed')

% scurr = rng; % turn off rng
% 
% randn(3,3)
% randn(3,3)
