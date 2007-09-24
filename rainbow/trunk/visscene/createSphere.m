function fv = createSphere(T, r)
% T - Transformation matrix
% r - radius of the sphere
% fv - vertex/face structure
% NOTE - Body coordinate system placed and center of volume
[x y z] = sphere; 
fv = surf2patch(x,y,z,z);
NV = size(fv.vertices,1);

% Homogenous vertices - hv
hv = ones(4,NV);
hv(1:3,:) = fv.vertices';

% Scaling to go from unit sphere to r
S = eye(4);
S(1,1) = r; S(2,2) = r; S(3,3) = r;

% Transformed vertices
hv = T*S*hv;
fv.vertices = hv(1:3,:)';