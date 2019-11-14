clear all
close all
clc

for k=1:3
rng('shuffle');
seed(k) = rng();
randn(1,3)
end

rng(seed(2))
randn(1,3)

save('seed.mat','seed')