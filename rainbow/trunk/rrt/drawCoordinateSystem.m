function drawCoordinateSystem(h, T, str)
% TODO: Write help for this function

% SceneML, Copyright (C) 2007, 2008  J.D. Yamokoski
% All rights reserved.
% Email: yamokosk at gmail dot com
%
% This library is free software; you can redistribute it and/or
% modify it under the terms of the GNU Lesser General Public License as
% published by the Free Software Foundation; either version 2.1 of the License, 
% or (at your option) any later version. The text of the GNU Lesser General 
% Public License is included with this library in the file LICENSE.TXT.
%
% This library is distributed in the hope that it will be useful, but WITHOUT 
% ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
% or FITNESS FOR A PARTICULAR PURPOSE. See the file LICENSE.TXT for 
% more details.

origin = T(1:3,4);
xpt = T * [.1;0;0;1];
xaxis = [origin'; xpt(1:3)'];

ypt = T * [0;.1;0;1];
yaxis = [origin'; ypt(1:3)'];

zpt = T * [0;0;.1;1];
zaxis = [origin'; zpt(1:3)'];

figure(h);
line(xaxis(:,1), xaxis(:,2), xaxis(:,3), 'Color', 'r');
line(yaxis(:,1), yaxis(:,2), yaxis(:,3), 'Color', 'g');
line(zaxis(:,1), zaxis(:,2), zaxis(:,3), 'Color', 'b');
text(origin(1), origin(2), origin(3), str);
