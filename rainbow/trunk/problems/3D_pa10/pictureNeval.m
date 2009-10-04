function [X,Y,Z] = pictureNeval(N,target_mean,target_var,f,udata)
% x = linspace(0.4,1.2,N);
% y = linspace(-1,0.5,N);
%x = linspace(0.05,1,N);
%y = linspace(-1,1,N);
x = linspace(0,1,N);
y = linspace(-.5,.5,N);
[X,Y] = meshgrid(x,y);

Z = map3DMetric(X,Y,target_mean(3),target_mean,target_var,f,udata);

%plotResults(X, Y, Z, target_mean, 'Image area fitness contour map');