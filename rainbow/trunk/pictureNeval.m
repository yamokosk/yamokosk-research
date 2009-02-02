function pictureNeval(f,udata)
N=50;
x = linspace(-.8,.8,N);
y = linspace(0,1.5,N);
[X,Y] = meshgrid(x,y);

target = [0;0;0;0;0;-1];

[Z_A,Z_phi,Z_theta] = map2DMetric(X,Y,target,f.neval,udata);

plotResults(X, Y, Z_A, target, 'Image area fitness contour map');
plotResults(X, Y, Z_phi, target, '\phi fitness contour map');
plotResults(X, Y, Z_theta, target, '\theta fitness contour map');

Z_t = Z_A .* Z_phi .* Z_theta;
plotResults(X, Y, Z_t, target, 'Fitness contour map');

Z_t = Z_A .* Z_phi;
plotResults(X, Y, Z_t, target, 'Fitness contour map without f_{\theta}');

function fig = plotResults(X, Y, Z, target, titleMsg)
v = [0.1 0.25 0.5 0.85]';
fig = figure();
plot(target(1),target(2),'k*')
%line([target(1); target(1) + 0.2*target(5)],[target(2); target(2) + 0.2*target(6)],'Color','black')
start = target(1:2,1);
stop = start + 0.2 * target(5:6,1);
arrow(start,stop,[],[],10);
text(start(1)+0.02,start(2), 't_i');
hold on
[C,h] = contour(X,Y,Z,v);
clabel(C,h,'LabelSpacing',72);
title(titleMsg);
xlabel('X [meters]');
ylabel('Y [meters]');