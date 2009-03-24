function [src,sen] = loadRobots(filename)

T_f_base = transl(1.343,0,0.601) * rotz(pi);
src = roboop(filename, 'PA10_DH', T_f_base);
src.name = 'src';
src.links(1).q = pi/2;
src.qmin = [-177, -64, -107, -255, -165, -255] * (pi/180);
src.qmax = [177, 124, 158, 255, 165, 255] * (pi/180);
src.qpmax = [1, 1, 2, 2*pi, 2*pi, 2*pi];
src.umax = [4.64, 4.64, 2.0, 0.29, 0.29, 0.29]*50;
src.ghandles = [];

T_f_base = transl(-1,0,0.601);
sen = roboop(filename, 'PA10_DH', T_f_base);
sen.name = 'sen';
sen.links(1).q = pi/2;
sen.qmin = [-177, -64, -107, -255, -165, -255] * (pi/180);
sen.qmax = [177, 124, 158, 255, 165, 255] * (pi/180);
sen.qpmax = [1, 1, 2, 2*pi, 2*pi, 2*pi];
sen.umax = [4.64, 4.64, 2.0, 0.29, 0.29, 0.29]*50;
sen.ghandles = [];
