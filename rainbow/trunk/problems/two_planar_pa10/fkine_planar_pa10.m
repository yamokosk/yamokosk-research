function [T_w_t, T_w_1, T_w_2, T_w_3] = fkine_planar_pa10(q, rs)

T_w_t = kine(rs, q);

if nargout > 1
    T_w_1 = fkine(rs, q, 2);
    if nargout > 2
        T_w_2 = fkine(rs, q, 3);
        if nargout > 3
            T_w_3 = fkine(rs, q, 5);
        end
    end
end
% T_0_1 = rotz(q(1));
% T_1_2 = transl(rs.l1,0,0) * rotz(q(2));
% T_2_3 = transl(rs.l2,0,0) * rotz(q(3));
% 
% T_w_1 = rs.T_0 * T_0_1;
% T_w_2 = T_w_1 * T_1_2;
% T_w_3 = T_w_2 * T_2_3;
% T_w_t = rs.T_0 * T_0_1 * T_1_2 * T_2_3 * rs.T_t;