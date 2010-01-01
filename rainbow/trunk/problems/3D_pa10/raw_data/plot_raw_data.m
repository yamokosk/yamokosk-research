function plot_raw_data(data,ind1,ind2)

X = linspace(0,100,101);

% figure(1)
% set(1,'Color',[1,1,1]);
% myerrorregion(X,p_knee(:,1),p_knee(:,4));
% hold on;
% plot(X,p_knee(:,1),'k-','LineWidth',1.5);
% xlabel('Percent HS to HS');
% ylabel('X-Coordinate [m]');
% 
% figure(2)
% set(2,'Color',[1,1,1]);
% myerrorregion(X,p_knee(:,2),p_knee(:,5));
% hold on;
% plot(X,p_knee(:,2),'k-','LineWidth',1.5);
% xlabel('Percent HS to HS');
% ylabel('Y-Coordinate [m]');
% 
% figure(3)
% set(3,'Color',[1,1,1]);
% myerrorregion(X,p_knee(:,3),p_knee(:,6));
% hold on;
% plot(X,p_knee(:,3),'k-','LineWidth',1.5);
% xlabel('Percent HS to HS');
% ylabel('Z-Coordinate [m]');

data(:,1) = data(:,1) - mean(data(:,1));
data(:,2) = data(:,2) - mean(data(:,2));

figure(4)
set(4,'Color',[1,1,1]);
plot3(data(:,1),data(:,2),data(:,3), 'k-', 'LineWidth', 1.5);
%grid on;
%axis([-0.1, 0.1, -.8, .6, 0, .6]);
axis([-0.3, 0.3, -.8, .6, 0, 1.2]);
axis square;

ind = ind1;
X=data(ind,1);
Y=data(ind,2);
Z=data(ind,3);
Sx = data(ind,4);
Sy = data(ind,5);
Sz = data(ind,6);
plot_variance_region(X,Y,Z,Sx,Sy,Sz);

ind = ind2;
X=data(ind,1);
Y=data(ind,2);
Z=data(ind,3);
Sx = data(ind,4);
Sy = data(ind,5);
Sz = data(ind,6);
plot_variance_region(X,Y,Z,Sx,Sy,Sz);


% figure(5)
% set(5,'Color',[1,1,1]);
% plot3(data(:,2),data(:,1),data(:,3), 'k-', 'LineWidth', 1.5);
% %grid on;
% %axis([-0.1, 0.1, -.8, .6, 0, .6]);
% axis([-0.6, 0.6, -.6, .6, 0, 1.2]);
% axis square;
% 
% for n = 1:3:101
%     X=data(n,1);
%     Y=data(n,2);
%     Z=data(n,3);
%     Sx = data(n,4);
%     Sy = data(n,5);
%     Sz = data(n,6);
%     ellipseXZ(Sy,Sz,0,Y,X,Z,'k');
% end
