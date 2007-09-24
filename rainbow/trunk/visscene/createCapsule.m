function fv = createCapsule(T, len, r)
% T - Transformation matrix
% len - length of the capsule
% r - radius of the capsule
% fv - vertex/face structure
% NOTE - Body coordinate system placed and center of volume
cyl = createCylinder(T,len,r);

% Place the spheres at the ends of the cylinder
F1 = eye(4); F2 = eye(4);
F1(3,4) = len/2; F2(3,4) = -len/2;
sp1 = createSphere(T*F1,r);
sp2 = createSphere(T*F2,r);

% Concatenate vertices
NVCYL = size(cyl.vertices,1);
NVSP1 = size(sp1.vertices,1);
fv.faces = [cyl.faces; sp1.faces + NVCYL; sp2.faces + NVCYL + NVSP1];
fv.vertices = [cyl.vertices; sp1.vertices; sp2.vertices];