function drawCoordinateSystem(h, T, str)
origin = T(1:3,4);
xpt = T(1:3,1); 
xaxis = [origin'; origin'+xpt'];

ypt = T(1:3,2);
yaxis = [origin'; origin'+ypt'];

zpt = T(1:3,3); 
zaxis = [origin'; origin'+zpt'];

figure(h);
line(xaxis(:,1), xaxis(:,2), xaxis(:,3), 'Color', 'r');
line(yaxis(:,1), yaxis(:,2), yaxis(:,3), 'Color', 'g');
line(zaxis(:,1), zaxis(:,2), zaxis(:,3), 'Color', 'b');
text(origin(1), origin(2), origin(3), str);
