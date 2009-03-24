function x = jointToMetricSpaceMap(x_src, x_sen, udata)

T_src = kine(udata.rsrc,x_src(1:6,1));
T_sen = kine(udata.rsen,x_sen(1:6,1));
pos = T_src(1:3,4);
ori = irotk(T_src);

d = norm(pos - T_sen(1:3,4));

J = jacobian(udata.rsrc,x_src(1:6,1));
v_src_EE = J * x_src(7:12,1);

% J = jacobian(udata.rsen,x_sen(1:6,1));
% v_sen_EE = J * x_sen(7:12,1);

x = [pos; ori; v_src_EE; d];