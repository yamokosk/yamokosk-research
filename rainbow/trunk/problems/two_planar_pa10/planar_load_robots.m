function [r1,r2] = planar_load_robots()

r1 = planar_load_robot('top');
r2 = planar_load_robot('bottom');

r1.T_0 = transl(0,1.200,0) * rotz(-pi/2);
r1.T_t = transl(r1.l3,0,0);

r2.T_0 = transl(0,-1.00,0) * rotz(pi/2);
r2.T_t = transl(r2.l3,0,0);

function r = planar_load_robot(name)
r.name = name;
r.l0 = .317;
r.l1 = .45;
r.l2 = .48;
r.l3 = .2;
r.T_0 = eye(4);
r.T_t = eye(4);
r.ghandles = [];
r.qmin = [-64, -107, -165] * (pi/180);
r.qmax = [124, 158, 165] * (pi/180);
r.qpmax = [57, 114, 360] * (pi/180);
r.umax = [4.64, 2.0, 0.29]*50;