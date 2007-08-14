function fv = createCylinder(T, len, r)
% T - Transformation matrix
% len - length of the cylinder
% fv - vertex/face structure
% NOTE - Body coordinate system placed and center of volume
[x y z] = cylinder(r); 
fv = surf2patch(x,y,z,z);
NV = size(fv.vertices,1);

% Homogenous vertices - hv
hv = ones(4,NV);
hv(1:3,:) = fv.vertices';

% Scaling to go from unit length to len
S = eye(4); S(3,3) = len;

% Offset the body coordinate system to center of volume
F = eye(4); F(3,4) = -0.5*len;

% Transformed vertices
hv = T*F*S*hv;
fv.vertices = hv(1:3,:)';
