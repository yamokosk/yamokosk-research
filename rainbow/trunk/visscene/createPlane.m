function fv = createPlane(p, params)
% T - Transformation matrix
% p - [xmin, xmax, ymin, ymax]
% params - a,b,c,d of the plane equation
% fv - vertex/face structure
% NOTE - Body coordinate system placed and center of volume
xmin = p(1); xmax = p(2); ymin = p(3); ymax = p(4);
a = params(1);  b = params(2); c = params(3); d = params(4);
z = zeros(4);
z(1) = (d - a*xmax - b*ymax)/c;
z(2) = (d - a*xmin - b*ymax)/c;
z(3) = (d - a*xmin - b*ymin)/c;
z(4) = (d - a*xmax - b*ymin)/c;

% Non-homogeneous vertices
fv.vertices = [xmax, xmin, xmin, xmax; ...
               ymax, ymax, ymin, ymin; ...
                z(1), z(2), z(3), z(4)]';

% Create a new patch object
fv.faces = [1, 2, 3, 4];
