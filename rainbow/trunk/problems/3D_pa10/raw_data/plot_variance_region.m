function plot_variance_region(X,Y,Z,Sx,Sy,Sz)

gcf;

Dx = 0.05;
Dy = 0.1;
Dz = 0.1;

% X-Y plane, Z=C
fv.vertices = [X+Dx, Y+Dy, Z; ...
               X-Dx, Y+Dy, Z; ...
               X-Dx, Y-Dy, Z; ...
               X+Dx, Y-Dy, Z];
fv.faces = [1, 2, 3, 4];
patch(fv, 'FaceColor', [0.9,0.9,0.9], 'FaceAlpha', 0.8, 'EdgeColor', [0.1, 0.1, 0.1]);
ellipse(Sx,Sy,0,X,Y,Z,'r');

% X-Z plane, Y=C
fv.vertices = [X+Dx, Y, Z+Dz; ...
               X-Dx, Y, Z+Dz; ...
               X-Dx, Y, Z-Dz; ...
               X+Dx, Y, Z-Dz];
fv.faces = [1, 2, 3, 4];
patch(fv, 'FaceColor', [0.9,0.9,0.9], 'FaceAlpha', 0.8, 'EdgeColor', [0.1, 0.1, 0.1]);
ellipseXZ(Sx,Sz,0,X,Y,Z,'r');