function q = sen_target_distance_metric(state, target, udata)
% state: [q_src_1, q_src_2, q_src_3, qp_src_1, qp_src_2, qp_src_3,
%         q_sen_1, q_sen_2, q_sen_3, qp_sen_1, qp_sen_2, qp_sen_3, t]'
% target: [xd, yd, vx, vy, ux, uy]'
q_sen = state(7:9,1);
T_world_sen = kine(udata.sen, q_sen);
x = T_world_sen(1:2,4);

offset = 0.1;
d = target(1:2,1);
u = target(5:6,1);
c = d + offset * u;

q = gaussian_rbf(x, c, 5);