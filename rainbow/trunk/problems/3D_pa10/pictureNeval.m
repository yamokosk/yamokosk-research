function Z = pictureNeval(N,target_mean,target_var,f,udata)
% x = linspace(0.4,1.2,N);
% y = linspace(-1,0.5,N);
%x = linspace(0.05,1,N);
%y = linspace(-1,1,N);
x = linspace(0.4,1,N);
y = linspace(-.4,.4,N);
[X,Y] = meshgrid(x,y);

Z = map3DMetric(X,Y,target_mean(3),target_mean,target_var,f,udata);

plotResults(X, Y, Z, target_mean, 'Image area fitness contour map');


function fig = plotResults(X, Y, Z, target, titleMsg)
%v = [.1:.1:.9]';
v = linspace(0,180,10);
fig = figure();
plot(target(1),target(2),'k*')
%line([target(1); target(1) + 0.2*target(5)],[target(2); target(2) + 0.2*target(6)],'Color','black')
start = target(1:2,1);
stop = start + 0.2 * target(5:6,1);
% arrow(start,stop,[],[],10);
text(start(1)+0.02,start(2), 't_i');
hold on
[C,h] = contour(X,Y,Z);
clabel(C,h,'LabelSpacing',72,'FontSize',12);
title(titleMsg);
xlabel('X (meters)','FontSize',12)
ylabel('Y (meters)','FontSize',12)

set(gca,'FontSize',12)
set(fig,'Color',[1 1 1])

title('')

% axis([-0.2, 1.2, -0.5, 0.5])
% axis([-0, 1, -1, 1])
axis([0.4 1 -0.4 0.4])