function [src,sen] = planar_load_robots(filename)

T_f_base = transl(0,1.517,0) * rotz(-pi/2) * roty(pi/2);
src = roboop(filename, 'PA10_DH', T_f_base);
src.name = 'src';
src.links(1).q = pi/2;
src.qmin = [-64, -107, -165] * (pi/180);
src.qmax = [124, 158, 165] * (pi/180);
src.qpmax = [57, 114, 360] * (pi/180);
src.umax = [4.64, 2.0, 0.29]*50;
src.ghandles = [];

T_f_base = transl(0,-1.317,0) * rotz(pi/2) * roty(pi/2);
sen = roboop(filename, 'PA10_DH', T_f_base);
sen.name = 'sen';
sen.links(1).q = pi/2;
sen.qmin = [-64, -107, -165] * (pi/180);
sen.qmax = [124, 158, 165] * (pi/180);
sen.qpmax = [57, 114, 360] * (pi/180);
sen.umax = [4.64, 2.0, 0.29]*50;
sen.ghandles = [];

% function r = planar_load_robot(name,filename)
% r = roboop(filename,'PA10_DH');
% r.links(1).q = pi/2;
% r.name = name;
% % r.l0 = .317;
% % r.l1 = .45;
% % r.l2 = .48;
% % r.l3 = .2;
% % r.T_0 = eye(4);
% % r.T_t = eye(4);
% r.ghandles = [];
% % r.qmin = [-64, -107, -165] * (pi/180);
% % r.qmax = [124, 158, 165] * (pi/180);
% r.qpmax = [57, 114, 360] * (pi/180);
% r.umax = [4.64, 2.0, 0.29]*50;