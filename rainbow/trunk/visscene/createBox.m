function fv = createBox(T, dim)
% T - Transformation matrix
% dim - dimensions of the box
% fv - vertex/face structure
% NOTE - Body coordinate system placed and center of volume

lx = dim(1)*0.5;
ly = dim(2)*0.5;
lz = dim(3)*0.5;

% Homogeneous vertices
hv = [-lx,  lx,  lx, -lx, -lx,  lx, lx, -lx; ...
      -ly, -ly,  ly,  ly, -ly, -ly, ly,  ly; ...
      -lz, -lz, -lz, -lz,  lz,  lz, lz,  lz; ...
        1,   1,   1,   1,   1,   1,  1,   1];
v = T*hv; % Transformed

% Non-homogeneous vertices
fv.vertices = v(1:3,:)';

% Create a new patch object
fv.faces = [1, 2, 6, 5; ...
            2, 3, 7, 6; ...
            3, 4, 8, 7; ...
            4, 1, 5, 8; ...
            1, 2, 3, 4; ...
            5, 6, 7, 8];
